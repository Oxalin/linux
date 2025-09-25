/*
 * Copyright 2015 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <linux/pci.h>

#include "amdgpu.h"
#include "amdgpu_ih.h"
#include "si_ih.h"
#include "sid.h"

#include "bif/bif_3_0_d.h"
#include "bif/bif_3_0_sh_mask.h"

#include "oss/oss_1_0_d.h"
#include "oss/oss_1_0_sh_mask.h"

static void si_ih_set_interrupt_funcs(struct amdgpu_device *adev);

/**
 * si_ih_enable_interrupts - Enable the interrupt ring buffer
 *
 * @adev: amdgpu_device pointer
 *
 * Enable the interrupt ring buffer (SI).
 */
static void si_ih_enable_interrupts(struct amdgpu_device *adev)
{
	u32 ih_cntl = RREG32(mmIH_CNTL);
	u32 ih_rb_cntl = RREG32(mmIH_RB_CNTL);

	ih_cntl |= IH_CNTL__ENABLE_INTR_MASK;
	ih_rb_cntl |= IH_RB_CNTL__RB_ENABLE_MASK;
	WREG32(mmIH_CNTL, ih_cntl);
	WREG32(mmIH_RB_CNTL, ih_rb_cntl);
	adev->irq.ih.enabled = true;
}

/**
 * si_ih_disable_interrupts - Disable the interrupt ring buffer
 *
 * @adev: amdgpu_device pointer
 *
 * Disable the interrupt ring buffer (SI).
 */
static void si_ih_disable_interrupts(struct amdgpu_device *adev)
{
	u32 ih_rb_cntl = RREG32(mmIH_RB_CNTL);
	u32 ih_cntl = RREG32(mmIH_CNTL);

	ih_rb_cntl &= ~IH_RB_CNTL__RB_ENABLE_MASK;
	ih_cntl &= ~IH_CNTL__ENABLE_INTR_MASK;
	WREG32(mmIH_RB_CNTL, ih_rb_cntl);
	WREG32(mmIH_CNTL, ih_cntl);
	/* set rptr, wptr to 0 */
	WREG32(mmIH_RB_RPTR, 0);
	WREG32(mmIH_RB_WPTR, 0);
	adev->irq.ih.enabled = false;
	adev->irq.ih.rptr = 0;
}

/**
 * si_ih_irq_init - init and enable the interrupt ring
 *
 * @adev: amdgpu_device pointer
 *
 * Allocate a ring buffer for the interrupt controller,
 * enable the RLC, disable interrupts, enable the IH
 * ring buffer and enable it (SI).
 * Called at device load and reume.
 * Returns 0 for success, errors for failure.
 */
static int si_ih_irq_init(struct amdgpu_device *adev)
{
	struct amdgpu_ih_ring *ih = &adev->irq.ih;
	int rb_bufsz;
	u32 interrupt_cntl, ih_cntl, ih_rb_cntl;

	si_ih_disable_interrupts(adev);

	/* set dummy read address to dummy page address */
	WREG32(mmINTERRUPT_CNTL2, adev->dummy_page_addr >> 8);
	interrupt_cntl = RREG32(mmINTERRUPT_CNTL);
	/* INTERRUPT_CNTL__IH_DUMMY_RD_OVERRIDE_MASK=0 - dummy read disabled with msi, enabled without msi
	 * INTERRUPT_CNTL__IH_DUMMY_RD_OVERRIDE_MASK=1 - dummy read controlled by IH_DUMMY_RD_EN
	 */
	interrupt_cntl &= ~INTERRUPT_CNTL__IH_DUMMY_RD_OVERRIDE_MASK;
	/* INTERRUPT_CNTL__IH_REQ_NONSNOOP_EN_MASK=1 if ring is in non-cacheable memory, e.g., vram */
	interrupt_cntl &= ~INTERRUPT_CNTL__IH_REQ_NONSNOOP_EN_MASK;
	WREG32(mmINTERRUPT_CNTL, interrupt_cntl);

	WREG32(mmIH_RB_BASE, adev->irq.ih.gpu_addr >> 8);
	rb_bufsz = order_base_2(adev->irq.ih.ring_size / 4);

	ih_rb_cntl = (IH_RB_CNTL__WPTR_OVERFLOW_ENABLE_MASK |
		      IH_RB_CNTL__WPTR_OVERFLOW_CLEAR_MASK |
		      (rb_bufsz << 1));

	ih_rb_cntl |= IH_RB_CNTL__WPTR_WRITEBACK_ENABLE_MASK;

	WREG32(mmIH_RB_WPTR_ADDR_LO, lower_32_bits(ih->wptr_addr));
	WREG32(mmIH_RB_WPTR_ADDR_HI, upper_32_bits(ih->wptr_addr) & 0xFF);

	WREG32(mmIH_RB_CNTL, ih_rb_cntl);
	WREG32(mmIH_RB_RPTR, 0);
	WREG32(mmIH_RB_WPTR, 0);
	if (adev->irq.msi_enabled)
		ih_cntl |= IH_CNTL__RPTR_REARM_MASK;
	WREG32(mmIH_CNTL, ih_cntl);

	pci_set_master(adev->pdev);
	si_ih_enable_interrupts(adev);

	return 0;
}

/**
 * si_ih_irq_disable - disable interrupts
 *
 * @adev: amdgpu_device pointer
 *
 * Disable interrupts on the hw (SI).
 */
static void si_ih_irq_disable(struct amdgpu_device *adev)
{
	si_ih_disable_interrupts(adev);
	mdelay(1);
}

/**
 * si_ih_get_wptr - get the IH ring buffer wptr
 *
 * @adev: amdgpu_device pointer
 * @ih: IH ring buffer to fetch wptr
 *
 * Get the IH ring buffer wptr from either the register
 * or the writeback memory buffer (SI).  Also check for
 * ring buffer overflow and deal with it.
 * Used by cik_irq_process().
 * Returns the value of the wptr.
 */
static u32 si_ih_get_wptr(struct amdgpu_device *adev,
			  struct amdgpu_ih_ring *ih)
{
	u32 wptr, tmp;

	wptr = le32_to_cpu(*ih->wptr_cpu);

	if (wptr & IH_RB_WPTR__RB_OVERFLOW_MASK) {
		wptr &= ~IH_RB_WPTR__RB_OVERFLOW_MASK;
		dev_warn(adev->dev, "IH ring buffer overflow (0x%08X, 0x%08X, 0x%08X)\n",
			wptr, ih->rptr, (wptr + 16) & ih->ptr_mask);
		ih->rptr = (wptr + 16) & ih->ptr_mask;
		tmp = RREG32(mmIH_RB_CNTL);
		tmp |= IH_RB_CNTL__WPTR_OVERFLOW_CLEAR_MASK;
		WREG32(mmIH_RB_CNTL, tmp);

		/* Unset the CLEAR_OVERFLOW bit immediately so new overflows
		 * can be detected.
		 */
		tmp &= ~IH_RB_CNTL__WPTR_OVERFLOW_CLEAR_MASK;
		WREG32(mmIH_RB_CNTL, tmp);
	}
	return (wptr & ih->ptr_mask);
}

/*        SI IV Ring
 * Each IV ring entry is 128 bits:
 * [7:0]    - interrupt source id
 * [31:8]   - reserved
 * [59:32]  - interrupt source data
 * [63:60]  - reserved
 * [71:64]  - RINGID
 *            CP:
 *            ME_ID [1:0], PIPE_ID[1:0], QUEUE_ID[2:0]
 *            QUEUE_ID - for compute, which of the 8 queues owned by the dispatcher
 *                     - for gfx, hw shader state (0=PS...5=LS, 6=CS)
 *            ME_ID - 0 = gfx, 1 = first 4 CS pipes, 2 = second 4 CS pipes
 *            PIPE_ID - ME0 0=3D
 *                    - ME1&2 compute dispatcher (4 pipes each)
 *            SDMA:
 *            INSTANCE_ID [1:0], QUEUE_ID[1:0]
 *            INSTANCE_ID - 0 = sdma0, 1 = sdma1
 *            QUEUE_ID - 0 = gfx, 1 = rlc0, 2 = rlc1
 * [79:72]  - VMID
 * [95:80]  - reserved: PASID not used on SI
 * [127:96] - reserved
 */

 /**
 * si_ih_decode_iv - decode an interrupt vector
 *
 * @adev: amdgpu_device pointer
 *
 * Decodes the interrupt vector at the current rptr
 * position and also advance the position.
 */
static void si_ih_decode_iv(struct amdgpu_device *adev,
			    struct amdgpu_ih_ring *ih,
			    struct amdgpu_iv_entry *entry)
{
	u32 ring_index = ih->rptr >> 2;
	uint32_t dw[4];

	dw[0] = le32_to_cpu(ih->ring[ring_index + 0]);
	dw[1] = le32_to_cpu(ih->ring[ring_index + 1]);
	dw[2] = le32_to_cpu(ih->ring[ring_index + 2]);
	dw[3] = le32_to_cpu(ih->ring[ring_index + 3]);

	entry->client_id = AMDGPU_IRQ_CLIENTID_LEGACY;
	entry->src_id = dw[0] & 0xff;
	entry->src_data[0] = dw[1] & 0xfffffff;
	entry->ring_id = dw[2] & 0xff;
	entry->vmid = (dw[2] >> 8) & 0xff;

	ih->rptr += 16;
}

/**
 * si_ih_set_rptr - set the IH ring buffer rptr
 *
 * @adev: amdgpu_device pointer
 * @ih: IH ring buffer to set wptr
 *
 * Set the IH ring buffer rptr.
 */
static void si_ih_set_rptr(struct amdgpu_device *adev,
			   struct amdgpu_ih_ring *ih)
{
	WREG32(mmIH_RB_RPTR, ih->rptr);
}

static int si_ih_early_init(struct amdgpu_ip_block *ip_block)
{
	struct amdgpu_device *adev = ip_block->adev;

	si_ih_set_interrupt_funcs(adev);

	return 0;
}

static int si_ih_sw_init(struct amdgpu_ip_block *ip_block)
{
	int r;
	struct amdgpu_device *adev = ip_block->adev;

	r = amdgpu_ih_ring_init(adev, &adev->irq.ih, 64 * 1024, false);
	if (r)
		return r;

	return amdgpu_irq_init(adev);
}

static int si_ih_sw_fini(struct amdgpu_ip_block *ip_block)
{
	struct amdgpu_device *adev = ip_block->adev;

	amdgpu_irq_fini_sw(adev);

	return 0;
}

static int si_ih_hw_init(struct amdgpu_ip_block *ip_block)
{
	struct amdgpu_device *adev = ip_block->adev;

	return si_ih_irq_init(adev);
}

static int si_ih_hw_fini(struct amdgpu_ip_block *ip_block)
{
	si_ih_irq_disable(ip_block->adev);

	return 0;
}

static int si_ih_suspend(struct amdgpu_ip_block *ip_block)
{
	return si_ih_hw_fini(ip_block);
}

static int si_ih_resume(struct amdgpu_ip_block *ip_block)
{
	return si_ih_hw_init(ip_block);
}

static bool si_ih_is_idle(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;
	u32 tmp = RREG32(mmSRBM_STATUS);

	if (tmp & SRBM_STATUS__IH_BUSY_MASK)
		return false;

	return true;
}

static int si_ih_wait_for_idle(struct amdgpu_ip_block *ip_block)
{
	unsigned i;
	struct amdgpu_device *adev = ip_block->adev;

	for (i = 0; i < adev->usec_timeout; i++) {
		if (si_ih_is_idle(adev))
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

static int si_ih_soft_reset(struct amdgpu_ip_block *ip_block)
{
	struct amdgpu_device *adev = ip_block->adev;

	u32 srbm_soft_reset = 0;
	u32 tmp = RREG32(mmSRBM_STATUS);

	if (tmp & SRBM_STATUS__IH_BUSY_MASK)
		srbm_soft_reset |= SRBM_SOFT_RESET__SOFT_RESET_IH_MASK;

	if (srbm_soft_reset) {
		tmp = RREG32(mmSRBM_SOFT_RESET);
		tmp |= srbm_soft_reset;
		dev_info(adev->dev, "SRBM_SOFT_RESET=0x%08X\n", tmp);
		WREG32(mmSRBM_SOFT_RESET, tmp);
		tmp = RREG32(mmSRBM_SOFT_RESET);

		udelay(50);

		tmp &= ~srbm_soft_reset;
		WREG32(mmSRBM_SOFT_RESET, tmp);
		tmp = RREG32(mmSRBM_SOFT_RESET);

		udelay(50);
	}

	return 0;
}

static int si_ih_set_clockgating_state(void *handle,
					  enum amd_clockgating_state state)
{
	return 0;
}

static int si_ih_set_powergating_state(void *handle,
					  enum amd_powergating_state state)
{
	return 0;
}

static const struct amd_ip_funcs si_ih_ip_funcs = {
	.name = "si_ih",
	.early_init = si_ih_early_init,
	.sw_init = si_ih_sw_init,
	.sw_fini = si_ih_sw_fini,
	.hw_init = si_ih_hw_init,
	.hw_fini = si_ih_hw_fini,
	.suspend = si_ih_suspend,
	.resume = si_ih_resume,
	.is_idle = si_ih_is_idle,
	.wait_for_idle = si_ih_wait_for_idle,
	.soft_reset = si_ih_soft_reset,
	.set_clockgating_state = si_ih_set_clockgating_state,
	.set_powergating_state = si_ih_set_powergating_state,
};

static const struct amdgpu_ih_funcs si_ih_funcs = {
	.get_wptr = si_ih_get_wptr,
	.decode_iv = si_ih_decode_iv,
	.set_rptr = si_ih_set_rptr
};

static void si_ih_set_interrupt_funcs(struct amdgpu_device *adev)
{
	adev->irq.ih_funcs = &si_ih_funcs;
}

const struct amdgpu_ip_block_version si_ih_ip_block =
{
	.type = AMD_IP_BLOCK_TYPE_IH,
	.major = 1,
	.minor = 0,
	.rev = 0,
	.funcs = &si_ih_ip_funcs,
};
