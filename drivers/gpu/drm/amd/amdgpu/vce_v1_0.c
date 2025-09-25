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

#include "oss/oss_1_0_d.h"
#include "oss/oss_1_0_sh_mask.h"

#include "smu/smu_7_0_1_d.h"
#include "smu/smu_7_0_1_sh_mask.h"

#define VCE_V1_0_FW_SIZE	(256 * 1024)
#define VCE_V1_0_STACK_SIZE	(64 * 1024)
#define VCE_V1_0_DATA_SIZE	(7808 * (AMDGPU_MAX_VCE_HANDLES + 1))
#define VCE_STATUS_VCPU_REPORT_FW_LOADED_MASK	0x02

static void vce_v1_0_set_ring_funcs(struct amdgpu_device *adev);
static void vce_v1_0_set_irq_funcs(struct amdgpu_device *adev);

struct vce_v1_0_fw_signature
{
	int32_t offset;
	uint32_t length;
	int32_t number;
	struct {
		uint32_t chip_id;
		uint32_t keyselect;
		uint32_t nonce[4];
		uint32_t sigval[4];
	} val[8];
};

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

	if (ring->me == 0) {
		// DRM_INFO("%s - mmVCE_RB_RPTR %d", __func__, RREG32(mmVCE_RB_RPTR));
		return RREG32(mmVCE_RB_RPTR);
	}
	else {
		// DRM_INFO("%s - mmVCE_RB_RPTR2 %d", __func__, RREG32(mmVCE_RB_RPTR2));
		return RREG32(mmVCE_RB_RPTR2);
	}
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

	if (ring->me == 0) {
		// DRM_INFO("%s - mmVCE_RB_WPTR %d", __func__, RREG32(mmVCE_RB_WPTR));
		return RREG32(mmVCE_RB_WPTR);
	}
	else {
		// DRM_INFO("%s - mmVCE_RB_WPTR2 %d", __func__, RREG32(mmVCE_RB_WPTR2));
		return RREG32(mmVCE_RB_WPTR2);
	}
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

	if (ring->me == 0) {
		DRM_INFO("In %s - mmVCE_RB_WPTR", __func__);
		WREG32(mmVCE_RB_WPTR, lower_32_bits(ring->wptr));
	}
	else {
		DRM_INFO("In %s - mmVCE_RB_WPTR2", __func__);
		WREG32(mmVCE_RB_WPTR2, lower_32_bits(ring->wptr));
	}
}

static int vce_v1_0_lmi_clean(struct amdgpu_device *adev)
{
	int i, j;
	DRM_INFO("In %s", __func__);

	for (i = 0; i < 10; ++i) {
		for (j = 0; j < 100; ++j) {
			uint32_t status = RREG32(mmVCE_LMI_STATUS);

			if (status & 0x337f) {
				DRM_INFO("Out %s", __func__);
				return 0;
			}
			mdelay(10);
		}
	}

	DRM_INFO("Out %s", __func__);
	return -ETIMEDOUT;
}

static int vce_v1_0_firmware_loaded(struct amdgpu_device *adev)
{
	DRM_INFO("In %s", __func__);
	int i, j;

	for (i = 0; i < 10; ++i) {
		uint32_t status;
		for (j = 0; j < 100; ++j) {
			status = RREG32(mmVCE_STATUS);
			// DRM_INFO("mmVCE_STATUS value: %d", status);

			if (status & VCE_STATUS_VCPU_REPORT_FW_LOADED_MASK) {
				DRM_INFO("Out %s", __func__);
				return 0;
			}
			mdelay(10);
		}

		DRM_ERROR("VCE not responding, trying to reset the ECPU!!!\n");
		WREG32_P(mmVCE_SOFT_RESET,
			VCE_SOFT_RESET__ECPU_SOFT_RESET_MASK,
			~VCE_SOFT_RESET__ECPU_SOFT_RESET_MASK);
		mdelay(10);
		WREG32_P(mmVCE_SOFT_RESET, 0,
			~VCE_SOFT_RESET__ECPU_SOFT_RESET_MASK);
		mdelay(10);
	}

	DRM_INFO("Out %s", __func__);
	return -ETIMEDOUT;
}

static void vce_v1_0_disable_cg(struct amdgpu_device *adev)
{
	DRM_INFO("In %s", __func__);

	WREG32(mmVCE_CGTT_CLK_OVERRIDE, 7);

	DRM_INFO("Out %s", __func__);
}

/* from AMDGPU vce 2.0
static void vce_v1_0_init_cg(struct amdgpu_device *adev)
{
	u32 tmp;

	tmp = RREG32(mmVCE_CLOCK_GATING_A);
	tmp &= ~0xfff;
	tmp |= ((0 << 0) | (4 << 4));
	tmp |= 0x40000;
	WREG32(mmVCE_CLOCK_GATING_A, tmp);

	tmp = RREG32(mmVCE_UENC_CLOCK_GATING);
	tmp &= ~0xfff;
	tmp |= ((0 << 0) | (4 << 4));
	WREG32(mmVCE_UENC_CLOCK_GATING, tmp);

	tmp = RREG32(mmVCE_CLOCK_GATING_B);
	tmp |= 0x10;
	tmp &= ~0x100000;
	WREG32(mmVCE_CLOCK_GATING_B, tmp);
}
*/

// from Radeon vce 1.0
static void vce_v1_0_init_cg(struct amdgpu_device *adev)
{
	DRM_INFO("In %s", __func__);
	u32 tmp;

	tmp = RREG32(mmVCE_CLOCK_GATING_A);
	tmp |= CGC_DYN_CLOCK_MODE;
	DRM_INFO("Writting %d to mmVCE_CLOCK_GATING_A", tmp);
	WREG32(mmVCE_CLOCK_GATING_A, tmp);

	tmp = RREG32(mmVCE_CLOCK_GATING_B);
	tmp |= 0x1e;
	tmp &= ~0xe100e1;
	DRM_INFO("Writting %d to mmVCE_CLOCK_GATING_B", tmp);
	WREG32(mmVCE_CLOCK_GATING_B, tmp);

	tmp = RREG32(mmVCE_UENC_CLOCK_GATING);
	tmp &= ~0xff9ff000;
	DRM_INFO("Writting %d to mmVCE_UENC_CLOCK_GATING", tmp);
	WREG32(mmVCE_UENC_CLOCK_GATING, tmp);

	tmp = RREG32(mmVCE_UENC_REG_CLOCK_GATING);
	tmp &= ~0x3ff;
	DRM_INFO("Writting %d to mmVCE_UENC_REG_CLOCK_GATING", tmp);
	WREG32(mmVCE_UENC_REG_CLOCK_GATING, tmp);

	DRM_INFO("Out %s", __func__);
}


int vce_v1_0_find_keyselect(struct amdgpu_device *adev)
{
	DRM_INFO("In %s", __func__);
	const struct common_firmware_header *hdr= (const struct common_firmware_header *)adev->vce.fw->data;
	unsigned int ucode_offset = le32_to_cpu(hdr->ucode_array_offset_bytes);
	struct vce_v1_0_fw_signature *sign;
	uint32_t chip_id;
	int i, j;

	sign = (void*)adev->vce.fw->data + ucode_offset;

	switch (adev->asic_type) {
	case CHIP_TAHITI:
		chip_id = 0x01000014;
		break;
	case CHIP_VERDE:
		chip_id = 0x01000015;
		break;
	case CHIP_PITCAIRN:
		chip_id = 0x01000016;
		break;
	default:
		DRM_ERROR("asic_type %#010x was not found!", adev->asic_type);
		DRM_INFO("Out %s", __func__);
		return -EINVAL;
	}

	for (i = 0; i < le32_to_cpu(sign->number); ++i) {
		if (le32_to_cpu(sign->val[i].chip_id) == chip_id) {
			break;
		}
	}

	if (i == le32_to_cpu(sign->number)) {
		DRM_ERROR("Loop over sign->number (%d)", le32_to_cpu(sign->number));
		DRM_INFO("Out %s", __func__);
		return -EINVAL;
	}

	adev->vce.keyselect = le32_to_cpu(sign->val[i].keyselect);
	DRM_DEBUG("%s - VCE keyselect: %d", __func__, adev->vce.keyselect);

	DRM_INFO("Out %s", __func__);
	return 0;
}

int vce_v1_0_load_fw(struct amdgpu_device *adev, uint32_t *cpu_addr)
{
	DRM_INFO("In %s", __func__);
	struct vce_v1_0_fw_signature *sign;
	uint32_t chip_id;
	int i, j, r;

	const struct common_firmware_header *hdr;
	unsigned int ucode_offset;
	hdr = (const struct common_firmware_header *)adev->vce.fw->data;
	ucode_offset = le32_to_cpu(hdr->ucode_array_offset_bytes);

	sign = (void*)adev->vce.fw->data + ucode_offset;

	switch (adev->asic_type) {
	case CHIP_TAHITI:
		chip_id = 0x01000014;
		break;
	case CHIP_VERDE:
		chip_id = 0x01000015;
		break;
	case CHIP_PITCAIRN:
		chip_id = 0x01000016;
		break;
	default:
		DRM_ERROR("asic_type %#010x was not found!", adev->asic_type);
		DRM_INFO("Out %s", __func__);
		return -EINVAL;
	}

	for (i = 0; i < le32_to_cpu(sign->number); ++i) {
		if (le32_to_cpu(sign->val[i].chip_id) == chip_id) {
			DRM_INFO("chip_id %#010x match", chip_id);
			break;
		}
		else {
			DRM_DEBUG("chip_id %#010x no match", le32_to_cpu(sign->val[i].chip_id));
		}
	}

	if (i == le32_to_cpu(sign->number)) {
		DRM_ERROR("Loop over sign->number (%d)", le32_to_cpu(sign->number));
		DRM_INFO("Out %s", __func__);
		return -EINVAL;
	}

	DRM_INFO("cpu_addr pointing at %#010llx", cpu_addr);
	cpu_addr += (256 - 64) / 4;
	cpu_addr[0] = sign->val[i].nonce[0];
	cpu_addr[1] = sign->val[i].nonce[1];
	cpu_addr[2] = sign->val[i].nonce[2];
	cpu_addr[3] = sign->val[i].nonce[3];
	cpu_addr[4] = cpu_to_le32(le32_to_cpu(sign->length) + 64);

	memset(&cpu_addr[5], 0, 44);
	DRM_INFO("Size of hdr->ucode_size_bytes - sizeof(*sign): %d", hdr->ucode_size_bytes - sizeof(*sign));
	memcpy(&cpu_addr[16], &sign[1], hdr->ucode_size_bytes - sizeof(*sign));

	cpu_addr += (le32_to_cpu(sign->length) + 64) / 4;
	cpu_addr[0] = sign->val[i].sigval[0];
	cpu_addr[1] = sign->val[i].sigval[1];
	cpu_addr[2] = sign->val[i].sigval[2];
	cpu_addr[3] = sign->val[i].sigval[3];

	if (!adev->vce.keyselect) {
		r = vce_v1_0_find_keyselect(adev);
		if (r) {
			DRM_ERROR("Couldn't find the keyselec value (%d)!", r);
		}
		else {
			DRM_INFO("%s - VCE keyselect: %d", __func__, adev->vce.keyselect);
		}
	}

	DRM_INFO("Out %s", __func__);
	return r;
}

/**
 * vce_v1_0_fw_validate - FW validation operation
 *
 * @adev: amdgpu_device pointer
 *
 * Initiate and check VCE validation.
 */
static int vce_v1_0_fw_validate(struct amdgpu_device *adev)
{
	DRM_INFO("In %s", __func__);
	int i;
	uint32_t keyselect = adev->vce.keyselect;
	uint32_t max_tries = 20;
	uint32_t delay = 100;

	DRM_INFO("VCE keyselect: %d", keyselect);
	WREG32(mmVCE_LMI_FW_START_KEYSEL, keyselect);

	for (i = 0; i < max_tries; ++i) {
		mdelay(delay);
		if (RREG32(mmVCE_FW_REG_STATUS) & VCE_FW_REG_STATUS__DONE_MASK) {
			DRM_INFO("mmVCE_FW_REG_STATUS DONE under %d ms", i*delay);
			break;
		}
	}

	if (i == max_tries) {
		DRM_ERROR("mmVCE_FW_REG_STATUS hasn't done yet: %#010x. Time out after %d ms", RREG32(mmVCE_FW_REG_STATUS), i*delay);
		DRM_INFO("Out %s", __func__);
		return -ETIMEDOUT;
	}

	if (!(RREG32(mmVCE_FW_REG_STATUS) & VCE_FW_REG_STATUS__PASS_MASK)) {
		DRM_ERROR("mmVCE_FW_REG_STATUS didn't pass: %#010x", RREG32(mmVCE_FW_REG_STATUS));
		DRM_INFO("Out %s", __func__);
		return -EINVAL;
	}

	for (i = 0; i < max_tries; ++i) {
		mdelay(delay);
		if (!(RREG32(mmVCE_FW_REG_STATUS) & VCE_FW_REG_STATUS__BUSY_MASK)) {
			DRM_INFO("mmVCE_FW_REG_STATUS not busy anymore under %d ms", i*delay);
			break;
		}
	}

	if (i == max_tries) {
		DRM_ERROR("mmVCE_FW_REG_STATUS still busy: %#010x. Time out after %d ms", RREG32(mmVCE_FW_REG_STATUS), i*delay);
		DRM_INFO("Out %s", __func__);
		return -ETIMEDOUT;
	}

	DRM_INFO("Out %s", __func__);
	return 0;
}

/**
 * vce_v1_0_mc_resume - memory controller programming
 *
 * @adev: amdgpu_device pointer
 *
 * Let the VCE memory controller know its offsets
 */
static int  vce_v1_0_mc_resume(struct amdgpu_device *adev)
{
	DRM_INFO("In %s", __func__);
	uint64_t addr = adev->vce.gpu_addr;
	// uint64_t addr = 0;
	uint32_t offset;
	// uint32_t offset = adev->vce.gpu_addr;
	// uint64_t offset;
	uint32_t size;
	int r = 0;

	WREG32_P(mmVCE_CLOCK_GATING_A, 0, ~(1 << 16));
	WREG32_P(mmVCE_UENC_CLOCK_GATING, 0x1FF000, ~0xFF9FF000);
	WREG32_P(mmVCE_UENC_REG_CLOCK_GATING, 0x3F, ~0x3F);
	WREG32(mmVCE_CLOCK_GATING_B, 0);

	WREG32_P(mmVCE_LMI_FW_PERIODIC_CTRL, 0x4, ~0x4);

	/* initialize VCE memory controller */
	WREG32(mmVCE_LMI_CTRL, 0x00398000);

	WREG32_P(mmVCE_LMI_CACHE_CTRL, 0x0, ~0x1);
	WREG32(mmVCE_LMI_SWAP_CNTL, 0);
	WREG32(mmVCE_LMI_SWAP_CNTL1, 0);
	WREG32(mmVCE_LMI_VM_CTRL, 0);

	WREG32(mmVCE_VCPU_SCRATCH7, AMDGPU_MAX_VCE_HANDLES);

	// According to old exchange with Marek and Alex Deucher,
	// the VCPU cache 40bit BAR has to be dealt with in AMDGPU
	// Under Radeon, the VCE_LMI_VCPU_CACHE_40BIT_BAR wasn't set,
	// defaulting to 0 (disregarding the gpu address).
	// To compensate, adding the gpu address to the FIRMWARE offset
	// would indeed point to the good address. However, this would work
	// only for adresses in the first 32bit of the gpu address space, since 
	// CACHE_OFFSETs are 32bit limited.
	// VCE_LMI_VCPU_CACHE_40BIT_BAR + CACHE_OFFSETs = the desired address
	// Since the gpu address space is not forced to 0 under AMDGPU, we have to handle 
	// VCE_LMI_VCPU_CACHE_40BIT_BAR properly first and then add the proper CACHE_OFFSETs.
	// So, we have to set it first to gpu_addr.
	DRM_INFO("gpu_addr is %#018llx", adev->vce.gpu_addr);
	/* 40BIT_BAR consumes 256bit pages */
	// DRM_INFO("mmVCE_LMI_VCPU_CACHE_40BIT_BAR set to %#018llx (%#018llx >> %d)", addr >> 8, addr, 8);
	// WREG32(mmVCE_LMI_VCPU_CACHE_40BIT_BAR, (addr >> 8));
	// WREG32(mmVCE_LMI_VCPU_CACHE_40BIT_BAR, (adev->vce.gpu_addr  >> 8));

	offset = addr + AMDGPU_VCE_FIRMWARE_OFFSET;	// The long way.
	// offset += AMDGPU_VCE_FIRMWARE_OFFSET;	// The old fashion way. This is the way it was done under RADEON
	// offset = AMDGPU_VCE_FIRMWARE_OFFSET;	// IF 40BIT_BAR was working properly
	size = VCE_V1_0_FW_SIZE;
	DRM_INFO("VCE_VCPU_CACHE_OFFSET0: offset (%#018x), masked 0x7fffffff (%#018x) and VCE_VCPU_CACHE_SIZE0: size (%d)", offset, offset & 0x7fffffff, size);
	WREG32(mmVCE_VCPU_CACHE_OFFSET0, offset & 0x7fffffff);
	WREG32(mmVCE_VCPU_CACHE_SIZE0, size);

	offset += size;
	size = VCE_V1_0_STACK_SIZE;
	DRM_INFO("VCE_VCPU_CACHE_OFFSET1: offset (%#018x), masked 0x7fffffff (%#018x) and VCE_VCPU_CACHE_SIZE1: size (%d)", offset, offset & 0x7fffffff, size);
	WREG32(mmVCE_VCPU_CACHE_OFFSET1, offset & 0x7fffffff);
	WREG32(mmVCE_VCPU_CACHE_SIZE1, size);

	offset += size;
	size = VCE_V1_0_DATA_SIZE;
	DRM_INFO("VCE_VCPU_CACHE_OFFSET2: offset (%#018x), masked 0x7fffffff (%#018x) and VCE_VCPU_CACHE_SIZE2: size (%d)", offset, offset & 0x7fffffff, size);
	WREG32(mmVCE_VCPU_CACHE_OFFSET2, offset & 0x7fffffff);
	WREG32(mmVCE_VCPU_CACHE_SIZE2, size);

	WREG32_P(mmVCE_LMI_CTRL2, 0x0, ~0x100);

	// Taken from VCE2
	// WREG32_FIELD(VCE_SYS_INT_EN, VCE_SYS_INT_TRAP_INTERRUPT_EN, 1);

	// TODO: if possible, validate fw only at hw_init
	r = vce_v1_0_fw_validate(adev);
	if (r) {
		DRM_ERROR("VCE Firmware can't be validated!");
	}

	// TODO: if possible, separate init_cg from mc_resume
	vce_v1_0_init_cg(adev); // covered in mc_resume, as under RADEON

	DRM_INFO("Out %s", __func__);
	return r;
	// return;
}


static bool vce_v1_0_is_idle(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;
	DRM_INFO("In %s", __func__);

	DRM_INFO("Out %s", __func__);
	return !(RREG32(mmSRBM_STATUS2) & SRBM_STATUS2__VCE_BUSY_MASK);
}

static int vce_v1_0_wait_for_idle(struct amdgpu_ip_block *ip_block)
{
	DRM_INFO("In %s", __func__);
	struct amdgpu_device *adev = ip_block->adev;
	unsigned int i;

	for (i = 0; i < adev->usec_timeout; i++) {
		if (vce_v1_0_is_idle(adev)) {
			return 0;
		}
	}

	DRM_ERROR("%d usec reached. Timed out: %d.", adev->usec_timeout, -ETIMEDOUT);
	DRM_INFO("Out %s", __func__);
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
	DRM_INFO("In %s", __func__);
	struct amdgpu_ring *ring;
	int r;

	// Before or after vce_v1_0_mc_resume didn't make any difference...
	// And removing this and replacing it only by vce_v1_0_init_cg
	// after vce_v1_0_mc_resume didn't help either...
	// vce_v1_0_init_cg(adev); // covered in mc_resume, as under RADEON
	// vce_v1_0_disable_cg(adev); // doesn't exist under RADEON

	// vce_v1_0_mc_resume(adev);

	// r = vce_v1_0_fw_validate(adev);
	// if (r) {
	// 	DRM_ERROR("VCE Firmware can't be validated!");
	// 	DRM_INFO("Out %s", __func__);
	// 	return r;
	// }

	// vce_v1_0_init_cg(adev);

	/* set BUSY flag */
	WREG32_P(mmVCE_STATUS, 1, ~1);
	// WREG32_FIELD(VCE_STATUS, JOB_BUSY, 1);

	ring = &adev->vce.ring[0];
	DRM_INFO("mmVCE_RB_RPTR %#010x", lower_32_bits(ring->wptr));
	DRM_INFO("mmVCE_RB_WPTR %#010x", lower_32_bits(ring->wptr));
	DRM_INFO("Complete gpu_addr %#018llx", ring->gpu_addr);
	DRM_INFO("mmVCE_RB_BASE_LO %#010x", lower_32_bits(ring->gpu_addr));
	DRM_INFO("mmVCE_RB_BASE_HI %#010x", upper_32_bits(ring->gpu_addr));
	DRM_INFO("mmVCE_RB_SIZE %d", ring->ring_size / 4);
	WREG32(mmVCE_RB_RPTR, lower_32_bits(ring->wptr));
	WREG32(mmVCE_RB_WPTR, lower_32_bits(ring->wptr));
	WREG32(mmVCE_RB_BASE_LO, lower_32_bits(ring->gpu_addr));
	WREG32(mmVCE_RB_BASE_HI, upper_32_bits(ring->gpu_addr));
	WREG32(mmVCE_RB_SIZE, ring->ring_size / 4);

	ring = &adev->vce.ring[1];
	WREG32(mmVCE_RB_RPTR2, lower_32_bits(ring->wptr));
	WREG32(mmVCE_RB_WPTR2, lower_32_bits(ring->wptr));
	WREG32(mmVCE_RB_BASE_LO2, lower_32_bits(ring->gpu_addr));
	WREG32(mmVCE_RB_BASE_HI2, upper_32_bits(ring->gpu_addr));
	WREG32(mmVCE_RB_SIZE2, ring->ring_size / 4);

	WREG32_P(mmVCE_VCPU_CNTL, VCE_VCPU_CNTL__CLK_EN_MASK, ~VCE_VCPU_CNTL__CLK_EN_MASK);

	WREG32_P(mmVCE_SOFT_RESET,
		 VCE_SOFT_RESET__ECPU_SOFT_RESET_MASK |
		 VCE_FME_SOFT_RESET, ~(
		 VCE_SOFT_RESET__ECPU_SOFT_RESET_MASK |
		 VCE_FME_SOFT_RESET));

	mdelay(100);

	WREG32_P(mmVCE_SOFT_RESET, 0, ~(
		 VCE_SOFT_RESET__ECPU_SOFT_RESET_MASK |
		 VCE_FME_SOFT_RESET));

	r = vce_v1_0_firmware_loaded(adev);
	if (r) {
		DRM_ERROR("VCE not responding, giving up!!!\n");
		DRM_INFO("Out %s", __func__);
		return r;
	}

	/* clear BUSY flag */
	WREG32_P(mmVCE_STATUS, 0, ~1);
	// WREG32_FIELD(VCE_STATUS, JOB_BUSY, 0);

	DRM_INFO("Out %s", __func__);
	return r;
}

static int vce_v1_0_stop(struct amdgpu_device *adev)
{
	struct amdgpu_ip_block *ip_block;
	int i;
	int status;
	DRM_INFO("In %s", __func__);

	if (vce_v1_0_lmi_clean(adev)) {
		DRM_INFO("VCE is not idle \n");
		DRM_INFO("Out %s", __func__);
		return 0;
	}

	ip_block = amdgpu_device_ip_get_ip_block(adev, AMD_IP_BLOCK_TYPE_VCN);
	if (!ip_block) {
		DRM_ERROR("amdgpu_device_ip_get_ip_block() failed, no ip_block");
		DRM_INFO("Out %s", __func__);
		return -EINVAL;
	}

	if (vce_v1_0_wait_for_idle(ip_block)) {
		DRM_INFO("VCE is busy, can't set clock gating");
		DRM_INFO("Out %s", __func__);
		return 0;
	}

	/* Stall UMC and register bus before resetting VCPU */
	WREG32_P(mmVCE_LMI_CTRL2, 1 << 8, ~(1 << 8));

	for (i = 0; i < 100; ++i) {
		status = RREG32(mmVCE_LMI_STATUS);
		if (status & 0x240)
			break;
		mdelay(1);
	}

	WREG32_P(mmVCE_VCPU_CNTL, 0, ~0x80001);

	/* put LMI, VCPU, RBC etc... into reset */
	WREG32_P(mmVCE_SOFT_RESET, 1, ~0x1);

	WREG32(mmVCE_STATUS, 0);

	DRM_INFO("Out %s", __func__);
	return 0;
}

// from Radeon VCE1
void vce_v1_0_enable_mgcg(struct amdgpu_device *adev, bool enable)
{
	DRM_INFO("In %s", __func__);
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
	DRM_INFO("Out %s", __func__);
}


static int vce_v1_0_early_init(struct amdgpu_ip_block *ip_block)
{
	DRM_INFO("In %s", __func__);
	struct amdgpu_device *adev = ip_block->adev;

	adev->vce.num_rings = 2;

	vce_v1_0_set_ring_funcs(adev);
	vce_v1_0_set_irq_funcs(adev);

	DRM_INFO("Out %s", __func__);
	return 0;
}

static int vce_v1_0_sw_init(struct amdgpu_ip_block *ip_block)
{
	DRM_INFO("In %s, same as si_vce_init() and si_vce_start()", __func__);
	struct amdgpu_ring *ring;
	int r, i;
	struct amdgpu_device *adev = ip_block->adev;

	if (!adev->vce.num_rings) {
		return -EINVAL;
	}

	/* VCE */
	r = amdgpu_irq_add_id(adev, AMDGPU_IRQ_CLIENTID_LEGACY, 167, &adev->vce.irq);
	if (r) {
		DRM_ERROR("amdgpu_irq_add_id() failed with error %i", r);
		DRM_INFO("Out %s", __func__);
		return r;
	}

	r = amdgpu_vce_sw_init(adev, VCE_V1_0_FW_SIZE +
		VCE_V1_0_STACK_SIZE + VCE_V1_0_DATA_SIZE);
	if (r) {
		DRM_ERROR("amdgpu_vce_sw_init() failed with error %i", r);
		DRM_INFO("Out %s", __func__);
		return r;
	}

	r = amdgpu_vce_resume(adev);
	if (r) {
		DRM_ERROR("amdgpu_vce_resume() failed with error %i", r);
		DRM_INFO("Out %s", __func__);
		return r;
	}

	for (i = 0; i < adev->vce.num_rings; i++) {
		enum amdgpu_ring_priority_level hw_prio = amdgpu_vce_get_ring_prio(i);

		ring = &adev->vce.ring[i];
		sprintf(ring->name, "vce%d", i);
		r = amdgpu_ring_init(adev, ring, 512, &adev->vce.irq, 0,
				     hw_prio, NULL);
		if (r) {
			DRM_ERROR("amdgpu_ring_init() failed with error %i", r);
			DRM_INFO("Out %s", __func__);
			return r;
		}
	}

	DRM_INFO("Out %s", __func__);
	return r;
}

static int vce_v1_0_sw_fini(struct amdgpu_ip_block *ip_block)
{
	int r;
	struct amdgpu_device *adev = ip_block->adev;
	DRM_INFO("In %s", __func__);

	r = amdgpu_vce_suspend(adev);
	if (r) {
		DRM_ERROR("amdgpu_vce_suspend() failed with error %i", r);
		DRM_INFO("Out %s", __func__);
		return r;
	}

	DRM_INFO("Out %s", __func__);
	return amdgpu_vce_sw_fini(adev);
}

/**
 * vce_v1_0_hw_init - start and test VCE block
 *
 * @ip_block: Pointer to the amdgpu_ip_block for this hw instance.
 *
 * Initialize the hardware, boot up the VCPU and do some testing
 */
static int vce_v1_0_hw_init(struct amdgpu_ip_block *ip_block)
{
	DRM_INFO("In %s, same as RADEON vce_v1_0_init()", __func__);
	struct amdgpu_ring *ring;
	int r, i;
	struct amdgpu_device *adev = ip_block->adev;

	/* In VCE 2.0 */
	// vce_v1_0_init_cg(adev);
	// vce_v1_0_disable_cg(adev);

	// ***
	/* Power up VCE */
	vce_v1_0_enable_mgcg(adev, true);
	if (adev->pm.dpm_enabled)
               amdgpu_dpm_enable_vce(adev, true);
       else {
		amdgpu_asic_set_vce_clocks(adev, 53300, 40000); // taken from radeon_vce_note_usage
	        // amdgpu_asic_set_vce_clocks(adev, 10000, 10000);	// Used by other VCE implementation
       }

	r = vce_v1_0_mc_resume(adev);
	if (r) {
		dev_err(adev->dev, "failed VCE mc resume (%d).\n", r);
	}

       /* vce_v1_0_mc_resume() must be called prior to calling vce_v1_0_fw_validate()*/
       /* fw_validate() and init_cg() could be independant from mc_resume(), to be tested*/
	// r = vce_v1_0_fw_validate(adev);
	// if (r) {
	// 	DRM_ERROR("VCE Firmware can't be validated!");
	// }

	// vce_v1_0_init_cg(adev);
	// ***

	/* Based on RADEON and on AMDGPU UVD 3.1 */
	r = vce_v1_0_start(adev);
	if (r) {
		DRM_ERROR("vce_v1_0_start() failed");
		DRM_INFO("Out %s", __func__);
		return r;
	}

	for (i = 0; i < adev->vce.num_rings; i++) {
		ring = &adev->vce.ring[i];
		// ring->sched.ready = true;
		r = amdgpu_ring_test_helper(ring);
		if (r) {
			// ring->sched.ready = false;
			DRM_ERROR("amdgpu_ring_test_ring() failed with ring[%d].\n", i);
			DRM_INFO("Out %s", __func__);
			return r;
		}
	}

	DRM_INFO("VCE initialized successfully.\n");

	DRM_INFO("Out %s", __func__);
	return 0;
}

static int vce_v1_0_hw_fini(struct amdgpu_ip_block *ip_block)
{
	DRM_INFO("In %s", __func__);

	cancel_delayed_work_sync(&ip_block->adev->vce.idle_work);

	DRM_INFO("Out %s", __func__);
	return 0;
}

static int vce_v1_0_suspend(struct amdgpu_ip_block *ip_block)
{
	int r;
	struct amdgpu_device *adev = ip_block->adev;
	DRM_INFO("In %s", __func__);

	/*
	 * Proper cleanups before halting the HW engine:
	 *   - cancel the delayed idle work
	 *   - enable powergating
	 *   - enable clockgating
	 *   - disable dpm
	 *
	 * TODO: to align with the VCN implementation, move the
	 * jobs for clockgating/powergating/dpm setting to
	 * ->set_powergating_state().
	 */
	cancel_delayed_work_sync(&adev->vce.idle_work);

	if (adev->pm.dpm_enabled) {
		amdgpu_dpm_enable_vce(adev, false);
	} else {
		amdgpu_asic_set_vce_clocks(adev, 0, 0);
		amdgpu_device_ip_set_powergating_state(adev, AMD_IP_BLOCK_TYPE_VCE,
						       AMD_PG_STATE_GATE);
		amdgpu_device_ip_set_clockgating_state(adev, AMD_IP_BLOCK_TYPE_VCE,
						       AMD_CG_STATE_GATE);
	}

	r = vce_v1_0_hw_fini(ip_block);
	if (r) {
		DRM_ERROR("vce_v1_0_hw_fini() failed with error %i", r);
		DRM_INFO("Out %s", __func__);
		return r;
	}

	DRM_INFO("Out %s", __func__);
	return amdgpu_vce_suspend(adev);
}

static int vce_v1_0_resume(struct amdgpu_ip_block *ip_block)
{
	int r;
	DRM_INFO("In %s", __func__);

	r = amdgpu_vce_resume(ip_block->adev);
	if (r){
		DRM_ERROR("amdgpu_vce_resume() failed with error %i", r);
		DRM_INFO("Out %s", __func__);
		return r;
	}

	DRM_INFO("Out %s", __func__);
	return vce_v1_0_hw_init(ip_block);
}

static int vce_v1_0_soft_reset(struct amdgpu_ip_block *ip_block)
{
	struct amdgpu_device *adev = ip_block->adev;
	DRM_INFO("In %s", __func__);

	WREG32_FIELD(SRBM_SOFT_RESET, SOFT_RESET_VCE, 1);
	mdelay(5);

	DRM_INFO("Out %s", __func__);
	return vce_v1_0_start(adev);
}

static int vce_v1_0_set_interrupt_state(struct amdgpu_device *adev,
					struct amdgpu_irq_src *source,
					unsigned type,
					enum amdgpu_interrupt_state state)
{
	uint32_t val = 0;
	DRM_INFO("In %s", __func__);

	if (state == AMDGPU_IRQ_STATE_ENABLE)
		val |= VCE_SYS_INT_EN__VCE_SYS_INT_TRAP_INTERRUPT_EN_MASK;

	WREG32_P(mmVCE_SYS_INT_EN, val, ~VCE_SYS_INT_EN__VCE_SYS_INT_TRAP_INTERRUPT_EN_MASK);

	DRM_INFO("Out %s", __func__);
	return 0;
}

static int vce_v1_0_process_interrupt(struct amdgpu_device *adev,
				      struct amdgpu_irq_src *source,
				      struct amdgpu_iv_entry *entry)
{
	DRM_INFO("In %s", __func__);

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

	DRM_INFO("Out %s", __func__);
	return 0;
}

static int vce_v1_0_set_clockgating_state(void *handle,
					  enum amd_clockgating_state state)
{
	bool gate = false;
	// bool sw_cg = false;

	struct amdgpu_device *adev = (struct amdgpu_device *)handle;
	DRM_INFO("In %s", __func__);

	if (state == AMD_CG_STATE_GATE) {
		gate = true;
		// sw_cg = true;
	}

	vce_v1_0_enable_mgcg(adev, gate);

	DRM_INFO("Out %s", __func__);
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
	DRM_INFO("In %s", __func__);

	if (state == AMD_PG_STATE_GATE) {
		DRM_INFO("Out %s", __func__);
		return vce_v1_0_stop(adev);
	}
	else{
		DRM_INFO("Out %s", __func__);
		return vce_v1_0_start(adev);
	}
}

static const struct amd_ip_funcs vce_v1_0_ip_funcs = {
	.name = "vce_v1_0",
	.early_init = vce_v1_0_early_init,
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
};

static const struct amdgpu_ring_funcs vce_v1_0_ring_funcs = {
	.type = AMDGPU_RING_TYPE_VCE,
	.align_mask = 0xf,
	.nop = VCE_CMD_NO_OP,
	.support_64bit_ptrs = false,
	.no_user_fence = true,
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
	DRM_INFO("In %s", __func__);
	int i;

	for (i = 0; i < adev->vce.num_rings; i++) {
		adev->vce.ring[i].funcs = &vce_v1_0_ring_funcs;
		adev->vce.ring[i].me = i;
	}
	DRM_INFO("Out %s", __func__);
};

static const struct amdgpu_irq_src_funcs vce_v1_0_irq_funcs = {
	.set = vce_v1_0_set_interrupt_state,
	.process = vce_v1_0_process_interrupt,
};

static void vce_v1_0_set_irq_funcs(struct amdgpu_device *adev)
{
	DRM_INFO("In %s", __func__);

	adev->vce.irq.num_types = 1;
	adev->vce.irq.funcs = &vce_v1_0_irq_funcs;

	DRM_INFO("Out %s", __func__);
};

const struct amdgpu_ip_block_version vce_v1_0_ip_block =
{
		.type = AMD_IP_BLOCK_TYPE_VCE,
		.major = 1,
		.minor = 0,
		.rev = 0,
		.funcs = &vce_v1_0_ip_funcs,
};
