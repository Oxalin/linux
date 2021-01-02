/*
 * Copyright 2013 Advanced Micro Devices, Inc.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * Authors: Christian König <christian.koenig@amd.com>
 */

#include <linux/firmware.h>

#include "amdgpu.h"
#include "amdgpu_vce.h"
#include "sid.h"

#include "vce/vce_1_0_d.h"
#include "vce/vce_1_0_sh_mask.h"

#include "smu/smu_7_0_1_d.h"
#include "smu/smu_7_0_1_sh_mask.h"

#include "oss/oss_1_0_d.h"
#include "oss/oss_1_0_sh_mask.h"


#define VCE_V1_0_FW_SIZE	(256 * 1024)
#define VCE_V1_0_STACK_SIZE	(64 * 1024)
#define VCE_V1_0_DATA_SIZE	(7808 * (AMDGPU_MAX_VCE_HANDLES + 1))
#define VCE_STATUS_VCPU_REPORT_FW_LOADED_MASK	0x02

/*
struct vce_v1_0_fw_signature
{
	int32_t off;
	uint32_t len;
	int32_t num;
	struct {
		uint32_t chip_id;
		uint32_t keyselect;
		uint32_t nonce[4];
		uint32_t sigval[4];
	} val[8];
};
*/
static void vce_v1_0_set_ring_funcs(struct amdgpu_device *adev);
static void vce_v1_0_set_irq_funcs(struct amdgpu_device *adev);
static int vce_v1_0_wait_for_idle(void *handle);
static int vce_v1_0_set_clockgating_state(void *handle,
					  enum amd_clockgating_state state);

static int vce_v1_0_hw_init(void *handle);
static void vce_v1_0_disable_cg(struct amdgpu_device *adev);

/**
 * vce_v1_0_ring_get_rptr - get read pointer
 *
 * @ring: amdgpu_ring pointer
 *
 * Returns the current hardware read pointer
 */
static uint64_t vce_v1_0_ring_get_rptr(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;

	if (ring == &adev->vce.ring[0])
		return RREG32(mmVCE_RB_RPTR);
	else
		return RREG32(mmVCE_RB_RPTR2);
}

/**
 * vce_v1_0_ring_get_wptr - get write pointer
 *
 * @ring: amdgpu_ring pointer
 *
 * Returns the current hardware write pointer
 */
static uint64_t vce_v1_0_ring_get_wptr(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;

	if (ring == &adev->vce.ring[0])
		return RREG32(mmVCE_RB_WPTR);
	else
		return RREG32(mmVCE_RB_WPTR2);
}

/**
 * vce_v1_0_ring_set_wptr - set write pointer
 *
 * @ring: amdgpu_ring pointer
 *
 * Commits the write pointer to the hardware
 */
static void vce_v1_0_ring_set_wptr(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;

	if (ring == &adev->vce.ring[0])
		WREG32(mmVCE_RB_WPTR, lower_32_bits(ring->wptr));
	else
		WREG32(mmVCE_RB_WPTR2, lower_32_bits(ring->wptr));
}

/* !!! To validate. This is taken from VCE2.0. While the logic is similar for start() and firmware_loaded(), this should be similar for stop() and its checkups. */
/* It is also similar at how it is done under UVD 3.1 */
static int vce_v1_0_lmi_clean(struct amdgpu_device *adev)
{
	int i, j;

	for (i = 0; i < 10; ++i) {
		for (j = 0; j < 100; ++j) {
			uint32_t status = RREG32(mmVCE_LMI_STATUS);

			if (status & 0x337f)
				return 0;
			mdelay(10);
		}
	}

	return -ETIMEDOUT;
}

static int vce_v1_0_firmware_loaded(struct amdgpu_device *adev)
{
	int i, j;

	for (i = 0; i < 10; ++i) {
		for (j = 0; j < 100; ++j) {
			uint32_t status = RREG32(mmVCE_STATUS);

			if (status & VCE_STATUS_VCPU_REPORT_FW_LOADED_MASK)
				return 0;
			mdelay(10);
		}

		DRM_ERROR("VCE not responding, trying to reset the ECPU!!!\n");
		WREG32_P(mmVCE_SOFT_RESET, VCE_ECPU_SOFT_RESET, ~VCE_ECPU_SOFT_RESET);
		mdelay(10);
		WREG32_P(mmVCE_SOFT_RESET, 0, ~VCE_ECPU_SOFT_RESET);
		mdelay(10);
	}

	return -ETIMEDOUT;
}

/* TODO !!! Validate taken from VCE2.0 */
static void vce_v1_0_disable_cg(struct amdgpu_device *adev)
{
	WREG32(mmVCE_CGTT_CLK_OVERRIDE, 7);
}

/* Not used for now since we are relying on original VCE1 enable_mgcg()*/
/* Based on VCE2, but with VCE1's mgcg code */
static void vce_v1_0_set_sw_cg(struct amdgpu_device *adev, bool gated)
{
	u32 tmp;

	if (gated) {
		tmp = RREG32(mmVCE_CLOCK_GATING_A);
		tmp |= CGC_DYN_CLOCK_MODE;
		WREG32(mmVCE_CLOCK_GATING_A, tmp);

		tmp = RREG32(mmVCE_UENC_CLOCK_GATING);
		tmp &= ~0x1ff000;
		tmp |= 0xff800000;
		WREG32(mmVCE_UENC_CLOCK_GATING, tmp);

		tmp = RREG32(mmVCE_UENC_REG_CLOCK_GATING);
		tmp &= ~0x3ff;
		WREG32(mmVCE_UENC_REG_CLOCK_GATING, tmp);

		WREG32(mmVCE_CGTT_CLK_OVERRIDE, 0);
	} else {
		tmp = RREG32(mmVCE_CLOCK_GATING_A);
		tmp &= ~CGC_DYN_CLOCK_MODE;
		WREG32(mmVCE_CLOCK_GATING_A, tmp);

		tmp = RREG32(mmVCE_UENC_CLOCK_GATING);
		tmp |= 0x1ff000;
		tmp &= ~0xff800000;
		/* both bitwise operation are the same as tmp &= ~0xff9ff000*/
		WREG32(mmVCE_UENC_CLOCK_GATING, tmp);

		tmp = RREG32(mmVCE_UENC_REG_CLOCK_GATING);
		tmp |= 0x3ff;
		WREG32(mmVCE_UENC_REG_CLOCK_GATING, tmp);
	}
}

/* Not used for now since we are relying on original VCE1 enable_mgcg()*/
/* Based on VCE2, but with VCE1's mgcg code */
static void vce_v1_0_set_dyn_cg(struct amdgpu_device *adev, bool gated)
{
	u32 orig, tmp;

	// Portage debug
	DRM_INFO("%s not tested and validated.\n", __FUNCTION__);

/* LMI_MC/LMI_UMC always set in dynamic,
 * set {CGC_*_GATE_MODE, CGC_*_SW_GATE} = {0, 0}
 */
	tmp = RREG32(mmVCE_CLOCK_GATING_B);
	tmp &= ~0x00060006; /* !!! Magic value */

/* Exception for ECPU, IH, SEM, SYS blocks needs to be turned on/off by SW */
	if (gated) {
		tmp |= 0xe100e1;  /* changed from 0xe10000 */
		WREG32(mmVCE_CLOCK_GATING_B, tmp);
	} else {
		tmp |= 0xe1;
		tmp &= ~0xe100e1;  /* changed from 0xe10000 */
		WREG32(mmVCE_CLOCK_GATING_B, tmp);
	}

	orig = tmp = RREG32(mmVCE_UENC_CLOCK_GATING);
	tmp &= ~0x1ff000;
	tmp &= ~0xff800000;
	/* both bitwise operation are the same as tmp &= ~0xff9ff000*/
	if (tmp != orig)
		WREG32(mmVCE_UENC_CLOCK_GATING, tmp);

	orig = tmp = RREG32(mmVCE_UENC_REG_CLOCK_GATING);
	tmp &= ~0x3ff;
	if (tmp != orig)
		WREG32(mmVCE_UENC_REG_CLOCK_GATING, tmp);

	/* set VCE_UENC_REG_CLOCK_GATING always in dynamic mode */
	WREG32(mmVCE_UENC_REG_CLOCK_GATING, 0x00);

	if(gated)
		WREG32(mmVCE_CGTT_CLK_OVERRIDE, 0);
}

void vce_v1_0_enable_mgcg(struct amdgpu_device *adev, bool enable,
								bool sw_cg)
{
	u32 tmp;

	if (enable && (adev->cg_flags & AMD_CG_SUPPORT_VCE_MGCG)) {
		tmp = RREG32(mmVCE_CLOCK_GATING_A);
		tmp |= CGC_DYN_CLOCK_MODE;
		WREG32(mmVCE_CLOCK_GATING_A, tmp);

		tmp = RREG32(mmVCE_UENC_CLOCK_GATING);
		tmp &= ~0x1ff000;
		tmp |= 0xff800000;
		WREG32(mmVCE_UENC_CLOCK_GATING, tmp);

		tmp = RREG32(mmVCE_UENC_REG_CLOCK_GATING);
		tmp &= ~0x3ff;
		WREG32(mmVCE_UENC_REG_CLOCK_GATING, tmp);
	} else {
		tmp = RREG32(mmVCE_CLOCK_GATING_A);
		tmp &= ~CGC_DYN_CLOCK_MODE;
		WREG32(mmVCE_CLOCK_GATING_A, tmp);

		tmp = RREG32(mmVCE_UENC_CLOCK_GATING);
		tmp |= 0x1ff000;
		tmp &= ~0xff800000;
		WREG32(mmVCE_UENC_CLOCK_GATING, tmp);

		tmp = RREG32(mmVCE_UENC_REG_CLOCK_GATING);
		tmp |= 0x3ff;
		WREG32(mmVCE_UENC_REG_CLOCK_GATING, tmp);
	}

/*
	if (enable && (adev->cg_flags & AMD_CG_SUPPORT_VCE_MGCG)) {
		if (sw_cg)
			vce_v1_0_set_sw_cg(adev, true);
		else
			vce_v1_0_set_dyn_cg(adev, true);
	} else {
		vce_v1_0_disable_cg(adev);

		if (sw_cg)
			vce_v1_0_set_sw_cg(adev, false);
		else
			vce_v1_0_set_dyn_cg(adev, false);
	}
*/
}

/* Keeping original code from radeon for now... */
static void vce_v1_0_init_cg(struct amdgpu_device *adev)
{
	u32 tmp;

	tmp = RREG32(mmVCE_CLOCK_GATING_A);
	tmp |= CGC_DYN_CLOCK_MODE;
	WREG32(mmVCE_CLOCK_GATING_A, tmp);

	tmp = RREG32(mmVCE_CLOCK_GATING_B);
	tmp |= 0x1e;
	tmp &= ~0xe100e1;
	WREG32(mmVCE_CLOCK_GATING_B, tmp);

	tmp = RREG32(mmVCE_UENC_CLOCK_GATING);
	tmp &= ~0xff9ff000;
	WREG32(mmVCE_UENC_CLOCK_GATING, tmp);

	tmp = RREG32(mmVCE_UENC_REG_CLOCK_GATING);
	tmp &= ~0x3ff;
	WREG32(mmVCE_UENC_REG_CLOCK_GATING, tmp);
}

/* !!! Still some variables to port */
/* From CIK under radeon and amdgpu, it seems mc_resume() is mostly resume() 
	without init_cg(). */
static void vce_v1_0_mc_resume(struct amdgpu_device *adev)
{
	uint64_t addr = adev->vce.gpu_addr;
	uint32_t size;

	WREG32_P(mmVCE_CLOCK_GATING_A, 0, ~(1 << 16));
	WREG32_P(mmVCE_UENC_CLOCK_GATING, 0x1FF000, ~0xFF9FF000);
	WREG32_P(mmVCE_UENC_REG_CLOCK_GATING, 0x3F, ~0x3F);
	WREG32(mmVCE_CLOCK_GATING_B, 0);

	WREG32_P(mmVCE_LMI_FW_PERIODIC_CTRL, 0x4, ~0x4);

	WREG32(mmVCE_LMI_CTRL, 0x00398000);
	WREG32_P(mmVCE_LMI_CACHE_CTRL, 0x0, ~0x1);
	WREG32(mmVCE_LMI_SWAP_CNTL, 0);
	WREG32(mmVCE_LMI_SWAP_CNTL1, 0);
	WREG32(mmVCE_LMI_VM_CTRL, 0);

	WREG32(mmVCE_VCPU_SCRATCH7, AMDGPU_MAX_VCE_HANDLES);

	addr += AMDGPU_VCE_FIRMWARE_OFFSET;
	size = VCE_V1_0_FW_SIZE;
	WREG32(mmVCE_VCPU_CACHE_OFFSET0, addr & 0x7fffffff);
	WREG32(mmVCE_VCPU_CACHE_SIZE0, size);

	addr += size;
	size = VCE_V1_0_STACK_SIZE;
	WREG32(mmVCE_VCPU_CACHE_OFFSET1, addr & 0x7fffffff);
	WREG32(mmVCE_VCPU_CACHE_SIZE1, size);

	addr += size;
	size = VCE_V1_0_DATA_SIZE;
	WREG32(mmVCE_VCPU_CACHE_OFFSET2, addr & 0x7fffffff);
	WREG32(mmVCE_VCPU_CACHE_SIZE2, size);

	WREG32_P(mmVCE_LMI_CTRL2, 0x0, ~0x100);
}

/*
int vce_v1_0_load_fw(struct radeon_device *rdev, uint32_t *data)
{
	struct vce_v1_0_fw_signature *sign = (void*)rdev->vce_fw->data;
	uint32_t chip_id;
	int i;

	switch (rdev->family) {
	case CHIP_TAHITI:
		chip_id = 0x01000014;
		break;
	case CHIP_VERDE:
		chip_id = 0x01000015;
		break;
	case CHIP_PITCAIRN:
	case CHIP_OLAND:
		chip_id = 0x01000016;
		break;
	case CHIP_ARUBA:
		chip_id = 0x01000017;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < le32_to_cpu(sign->num); ++i) {
		if (le32_to_cpu(sign->val[i].chip_id) == chip_id)
			break;
	}

	if (i == le32_to_cpu(sign->num))
		return -EINVAL;

	data += (256 - 64) / 4;
	data[0] = sign->val[i].nonce[0];
	data[1] = sign->val[i].nonce[1];
	data[2] = sign->val[i].nonce[2];
	data[3] = sign->val[i].nonce[3];
	data[4] = cpu_to_le32(le32_to_cpu(sign->len) + 64);

	memset(&data[5], 0, 44);
	memcpy(&data[16], &sign[1], rdev->vce_fw->size - sizeof(*sign));

	data += (le32_to_cpu(sign->len) + 64) / 4;
	data[0] = sign->val[i].sigval[0];
	data[1] = sign->val[i].sigval[1];
	data[2] = sign->val[i].sigval[2];
	data[3] = sign->val[i].sigval[3];

	rdev->vce.keyselect = le32_to_cpu(sign->val[i].keyselect);

	return 0;
}

unsigned vce_v1_0_bo_size(struct radeon_device *rdev)
{
	WARN_ON(VCE_V1_0_FW_SIZE < rdev->vce_fw->size);
	return VCE_V1_0_FW_SIZE + VCE_V1_0_STACK_SIZE + VCE_V1_0_DATA_SIZE;
}
*/
/*
int vce_v1_0_init(struct radeon_device *rdev)
{
	struct radeon_ring *ring;
	int r;

	r = vce_v1_0_start(rdev);
	if (r)
		return r;

	ring = &rdev->ring[TN_RING_TYPE_VCE1_INDEX];
	ring->ready = true;
	r = radeon_ring_test(rdev, TN_RING_TYPE_VCE1_INDEX, ring);
	if (r) {
		ring->ready = false;
		return r;
	}

	ring = &rdev->ring[TN_RING_TYPE_VCE2_INDEX];
	ring->ready = true;
	r = radeon_ring_test(rdev, TN_RING_TYPE_VCE2_INDEX, ring);
	if (r) {
		ring->ready = false;
		return r;
	}

	DRM_INFO("VCE initialized successfully.\n");

	return 0;
}
*/

/**
 * vce_v1_0_fw_validate - FW validation operation
 *
 * @adev: amdgpu_device pointer
 *
 * Initialate and check VCE validation.
 */
static int vce_v1_0_fw_validate(struct amdgpu_device *adev)
{
	int i;
	uint32_t keysel = adev->vce.keyselect;

	WREG32(mmVCE_LMI_FW_START_KEYSEL, keysel);

	for (i = 0; i < 10; ++i) {
		mdelay(10);
		if (RREG32(mmVCE_FW_REG_STATUS) & VCE_FW_REG_STATUS_DONE)
			break;
	}

	if (i == 10)
		return -ETIMEDOUT;

	if (!(RREG32(mmVCE_FW_REG_STATUS) & VCE_FW_REG_STATUS_PASS))
		return -EINVAL;

	for (i = 0; i < 10; ++i) {
		mdelay(10);
		if (!(RREG32(mmVCE_FW_REG_STATUS) & VCE_FW_REG_STATUS_BUSY))
			break;
	}

	if (i == 10)
		return -ETIMEDOUT;

	return 0;
}

/* !!! Same structure as under si_ih.c and variables defines as for VCE 2 and 3 */
static bool vce_v1_0_is_idle(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	return !(RREG32(mmSRBM_STATUS2) & SRBM_STATUS2__VCE_BUSY_MASK);
}

static int vce_v1_0_wait_for_idle(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;
	unsigned i;

	for (i = 0; i < adev->usec_timeout; i++) {
		if (vce_v1_0_is_idle(handle))
			return 0;
	}
	return -ETIMEDOUT;
}

/**
 * vce_v1_0_start - start VCE block
 *
 * @adev: amdgpu_device pointer
 *
 * Setup and start the VCE block
 */
static int vce_v1_0_start(struct amdgpu_device *adev)
{
	struct amdgpu_ring *ring;
	int r;

	// set BUSY flag
	WREG32_P(mmVCE_STATUS, 1, ~1);

/* Imported from VCE 2.0, may not be needed */
	vce_v1_0_init_cg(adev);
	vce_v1_0_disable_cg(adev);

	vce_v1_0_mc_resume(adev);

	ring = &adev->vce.ring[0];
	WREG32(mmVCE_RB_RPTR, lower_32_bits(ring->wptr));
	WREG32(mmVCE_RB_WPTR, lower_32_bits(ring->wptr));
	WREG32(mmVCE_RB_BASE_LO, ring->gpu_addr);
	WREG32(mmVCE_RB_BASE_HI, upper_32_bits(ring->gpu_addr));
	WREG32(mmVCE_RB_SIZE, ring->ring_size / 4);

	ring = &adev->vce.ring[1];
	WREG32(mmVCE_RB_RPTR2, lower_32_bits(ring->wptr));
	WREG32(mmVCE_RB_WPTR2, lower_32_bits(ring->wptr));
	WREG32(mmVCE_RB_BASE_LO2, ring->gpu_addr);
	WREG32(mmVCE_RB_BASE_HI2, upper_32_bits(ring->gpu_addr));
	WREG32(mmVCE_RB_SIZE2, ring->ring_size / 4);

	WREG32_P(mmVCE_VCPU_CNTL, VCE_CLK_EN, ~VCE_CLK_EN);

	WREG32_P(mmVCE_SOFT_RESET,
		 VCE_ECPU_SOFT_RESET |
		 VCE_FME_SOFT_RESET, ~(
		 VCE_ECPU_SOFT_RESET |
		 VCE_FME_SOFT_RESET));

	mdelay(100);

	WREG32_P(mmVCE_SOFT_RESET, 0, ~(
		 VCE_ECPU_SOFT_RESET |
		 VCE_FME_SOFT_RESET));

	r = vce_v1_0_firmware_loaded(adev);

	// clear BUSY flag
	WREG32_P(mmVCE_STATUS, 0, ~1);

	if (r) {
		DRM_ERROR("VCE not responding, giving up!!!\n");
		return r;
	}

	return 0;
}

/* !!! To validate. This is directly taken from VCE2.0. While the logic is similar for start(), the same should apply for stop(). */
/* This is also similar to how it is done under UVD 3.1 */
/**
 * vce_v1_0_stop - stop VCE block
 *
 * @adev: amdgpu_device pointer
 *
 * stop the VCE block
 */
static int vce_v1_0_stop(struct amdgpu_device *adev)
{
	int i;
	int status;

	if (vce_v1_0_lmi_clean(adev)) {
		DRM_INFO("VCE is not idle \n");
		return 0;
	}

	if (vce_v1_0_wait_for_idle(adev)) {
		DRM_INFO("VCE is busy, can't set clock gating");
		return 0;
	}

	/* Stall UMC and register bus before resetting VCPU */
	/* I think the magic values should be replaced by VCE_LMI_CTRL2__STALL_ARB_UMC_[MASK,_SHIFT] */
	WREG32_P(mmVCE_LMI_CTRL2, 1 << 8, ~(1 << 8));

	for (i = 0; i < 100; ++i) {
		status = RREG32(mmVCE_LMI_STATUS);
		if (status & 0x240)
			break;
		mdelay(1);
	}

	WREG32_P(mmVCE_VCPU_CNTL, 0, ~0x80001);

	/* put LMI, VCPU, RBC etc... into reset */
	/* Magic values that should be similar to the ones used under UVD 3.1 */
	WREG32_P(mmVCE_SOFT_RESET, 1, ~0x1);

	WREG32(mmVCE_STATUS, 0);

	return 0;
}

static int vce_v1_0_early_init(void *handle)
 {
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	adev->vce.num_rings = 2;

	vce_v1_0_set_ring_funcs(adev);
	vce_v1_0_set_irq_funcs(adev);

	return 0;
}

/* It seems to be right compared to other VCE versions and UVD 3.1 */
static int vce_v1_0_sw_init(void *handle)
{
	struct amdgpu_ring *ring;
	int r, i;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	/* VCE TRAP */
	r = amdgpu_irq_add_id(adev, AMDGPU_IRQ_CLIENTID_LEGACY, 167, &adev->vce.irq);
	if (r)
		return r;

	r = amdgpu_vce_sw_init(adev, VCE_V1_0_FW_SIZE +
		VCE_V1_0_STACK_SIZE + VCE_V1_0_DATA_SIZE);
	if (r)
		return r;

	r = amdgpu_vce_resume(adev);
	if (r)
		return r;

	for (i = 0; i < adev->vce.num_rings; i++) {
		ring = &adev->vce.ring[i];
		sprintf(ring->name, "vce%d", i);
		r = amdgpu_ring_init(adev, ring, 512,
				     &adev->vce.irq, 0,
				     AMDGPU_RING_PRIO_DEFAULT);
		if (r)
			return r;
	}

	r = amdgpu_vce_entity_init(adev);

	// Portage debug
	DRM_INFO("%s succeeded.\n", __FUNCTION__);

	return r;
}

// !!! Something is failing here for now
static int vce_v1_0_sw_fini(void *handle)
{
	int r;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	r = amdgpu_vce_suspend(adev);
	if (r) {
		DRM_INFO("%s failed.\n", "amdgpu_vce_suspend(adev)");
		return r;
	}

	r = amdgpu_vce_sw_fini(adev);

	// Portage debug
	if (r) {
		DRM_INFO("%s failed.\n", "amdgpu_vce_sw_fini(adev)");
	}
	else {
		DRM_INFO("%s succeeded.\n", __FUNCTION__);
	}

	return r;
}

/* !!! Ported from other VCE versions. */
/**
 * vce_v1_0_hw_init - start and test VCE block
 *
 * @adev: amdgpu_device pointer
 *
 * Initialize the hardware, boot up the VCPU and do some testing
 */
static int vce_v1_0_hw_init(void *handle)
{
	int r, i;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	r = vce_v1_0_fw_validate(adev);
	if (r) {
		DRM_ERROR("AMDGPU: VCE Firmware validate failed (%d).\n", r);
		return r;
	}

	amdgpu_asic_set_vce_clocks(adev, 10000, 10000);
	vce_v1_0_enable_mgcg(adev, true, false);

//	for (i = 0; i < adev->vce.num_rings; i++)
//		adev->vce.ring[i].ready = false;

	for (i = 0; i < adev->vce.num_rings; i++) {
		r = amdgpu_ring_test_ring(&adev->vce.ring[i]);
		if (r)
			return r;
//		else
//			adev->vce.ring[i].ready = true;
	}

	DRM_INFO("VCE initialized successfully.\n");

	return 0;
}

static int vce_v1_0_hw_fini(void *handle)
{
	return 0;
/* Taken from VCE 3 */
/* 	int r;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	r = vce_v1_0_wait_for_idle(handle);
	if (r)
		return r;

	vce_v1_0_stop(adev);
	return vce_v1_0_set_clockgating_state(adev, AMD_CG_STATE_GATE); */
}

static int vce_v1_0_suspend(void *handle)
{
	int r;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	r = vce_v1_0_hw_fini(adev);
	if (r)
		return r;

	return amdgpu_vce_suspend(adev);
}

/* Imported from VCE2 since it was also modified from radeon to amdgpu under VCE2 */
static int vce_v1_0_resume(void *handle)
{
	int r;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	r = amdgpu_vce_resume(adev);
	if (r)
		return r;

	return vce_v1_0_hw_init(adev);
}

/* !!! To valide. Ported from VCE2.0 and later */
/* However, VCE 3.0 uses an approach a lot similar to si_ih_soft_reset()... 
	 This could be an alternative... */
static int vce_v1_0_soft_reset(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	WREG32_FIELD(SRBM_SOFT_RESET, SOFT_RESET_VCE, 1);
	mdelay(5);

	return vce_v1_0_start(adev);
}

/* Ported from VCE2.0 and later */
static int vce_v1_0_set_interrupt_state(struct amdgpu_device *adev,
					struct amdgpu_irq_src *source,
					unsigned type,
					enum amdgpu_interrupt_state state)
{
	uint32_t val = 0;

	if (state == AMDGPU_IRQ_STATE_ENABLE)
		val |= VCE_SYS_INT_EN__VCE_SYS_INT_TRAP_INTERRUPT_EN_MASK;

	WREG32_P(mmVCE_SYS_INT_EN, val, ~VCE_SYS_INT_EN__VCE_SYS_INT_TRAP_INTERRUPT_EN_MASK);
	return 0;
}

/* Ported from VCE2.0 from both amdgpu and radeon (cik.c) driver. This was not done under radeon (si.c). */
static int vce_v1_0_process_interrupt(struct amdgpu_device *adev,
				      struct amdgpu_irq_src *source,
				      struct amdgpu_iv_entry *entry)
{
	DRM_DEBUG("IH: VCE\n");
	switch (entry->src_data[0]) {
	case 0:
	case 1:
		amdgpu_fence_process(&adev->vce.ring[entry->src_data[0]]);
		break;
	default:
		DRM_ERROR("Unhandled interrupt: %d %d\n",
			  entry->src_id, entry->src_data[0]);
		break;
	}

	return 0;
}

/* !!! Code taken from VCE 2 */
static int vce_v1_0_set_clockgating_state(void *handle,
					  enum amd_clockgating_state state)
{
	bool gate = false;
	bool sw_cg = false;

	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	if (state == AMD_CG_STATE_GATE) {
		gate = true;
		sw_cg = true;
	}

	vce_v1_0_enable_mgcg(adev, gate, sw_cg);

	return 0;
}

static int vce_v1_0_set_powergating_state(void *handle,
					  enum amd_powergating_state state)
{
	/* This doesn't actually powergate the VCE block.
	 * That's done in the dpm code via the SMC.  This
	 * just re-inits the block as necessary.  The actual
	 * gating still happens in the dpm code.  We should
	 * revisit this when there is a cleaner line between
	 * the smc and the hw blocks
	 */
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	if (state == AMD_PG_STATE_GATE)
		return vce_v1_0_stop(adev);
	else
		return vce_v1_0_start(adev);
}

static const struct amd_ip_funcs vce_v1_0_ip_funcs = {
	.name = "vce_v1_0",
	.early_init = vce_v1_0_early_init,
	.late_init = NULL,
	.sw_init = vce_v1_0_sw_init,
	.sw_fini = vce_v1_0_sw_fini,
	.hw_init = vce_v1_0_hw_init,
	.hw_fini = vce_v1_0_hw_fini,
	.suspend = vce_v1_0_suspend,
	.resume = vce_v1_0_resume,
	.is_idle = vce_v1_0_is_idle,
	.wait_for_idle = vce_v1_0_wait_for_idle,
	.soft_reset = vce_v1_0_soft_reset,
	.set_clockgating_state = vce_v1_0_set_clockgating_state,
	.set_powergating_state = vce_v1_0_set_powergating_state,

	// .early_init = vce_v1_0_early_init,
	// .late_init = NULL,
	// .sw_init = vce_v1_0_sw_init,
	// .sw_fini = vce_v1_0_sw_fini,
	// .hw_init = vce_v1_0_hw_init,
	// .hw_fini = vce_v1_0_hw_fini,
	// .suspend = vce_v1_0_suspend,
	// .resume = vce_v1_0_resume,
	// .is_idle = vce_v1_0_is_idle,
	// .wait_for_idle = vce_v1_0_wait_for_idle,
	// .soft_reset = vce_v1_0_soft_reset,
	// .set_clockgating_state = vce_v1_0_set_clockgating_state,
	// .set_powergating_state = vce_v1_0_set_powergating_state,
};

/* !!! Values validated */
static const struct amdgpu_ring_funcs vce_v1_0_ring_funcs = {
	.type = AMDGPU_RING_TYPE_VCE,
	.align_mask = 0xf,
	.nop = VCE_CMD_NO_OP,
	.support_64bit_ptrs = false,
	.get_rptr = vce_v1_0_ring_get_rptr,
	.get_wptr = vce_v1_0_ring_get_wptr,
	.set_wptr = vce_v1_0_ring_set_wptr,
	.parse_cs = amdgpu_vce_ring_parse_cs,
	.emit_frame_size = 6, /* amdgpu_vce_ring_emit_fence  x1 no user fence */
	.emit_ib_size = 4, /* amdgpu_vce_ring_emit_ib */
	.emit_ib = amdgpu_vce_ring_emit_ib,
	.emit_fence = amdgpu_vce_ring_emit_fence,
	.test_ring = amdgpu_vce_ring_test_ring,
	.test_ib = amdgpu_vce_ring_test_ib,
	.insert_nop = amdgpu_ring_insert_nop,
	.pad_ib = amdgpu_ring_generic_pad_ib,
	.begin_use = amdgpu_vce_ring_begin_use,
	.end_use = amdgpu_vce_ring_end_use,
};

static void vce_v1_0_set_ring_funcs(struct amdgpu_device *adev)
{
	int i;

	for (i = 0; i < adev->vce.num_rings; i++)
		adev->vce.ring[i].funcs = &vce_v1_0_ring_funcs;
}

static const struct amdgpu_irq_src_funcs vce_v1_0_irq_funcs = {
	.set = vce_v1_0_set_interrupt_state,
	.process = vce_v1_0_process_interrupt,
};

static void vce_v1_0_set_irq_funcs(struct amdgpu_device *adev)
{
	adev->vce.irq.num_types = 1;
	adev->vce.irq.funcs = &vce_v1_0_irq_funcs;
}

const struct amdgpu_ip_block_version vce_v1_0_ip_block =
{
		.type = AMD_IP_BLOCK_TYPE_VCE,
		.major = 1,
		.minor = 0,
		.rev = 0,
		.funcs = &vce_v1_0_ip_funcs,
};