/*
 * Copyright © 2013 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include <linux/sysrq.h>
#include <linux/slab.h>
#include "i915_drm.h"
#include "drm_crtc.h"
#include "i915_drv.h"
#include "intel_drv.h"
#include "i915_trace.h"
#include "intel_drv.h"
u32 prev_level=0xffff;
/*
 * DPST (Display Power Savings Technology) is a power savings features
 * which reduces the backlight while enhancing the image such that the
 * user does not perceive any difference in the image quality. The backlight
 * reduction can provide power savings
 *
 * The DPST IOCTL implemented in this file can be used by the DPST a user-mode
 * module. The IOCTL provides methods to initialize the DPST hardware,
 * manage DPST interrupts, and to apply the new backlight and image enhancement
 * values.
 *
 * The user mode module will initialize the DPST hardware when it starts up.
 * The kernel will notify user mode module of any DPST histogram interrupts.
 * When the user mode module receives a notification of these interrupts, it
 * will query the kernel for all the DPST histogram data. Using this data,
 * the user mode module will calculate new backlight and image enhancement
 * values and provides those values to the kernel to program into the DPST
 * hardware.
 */

static int
i915_dpst_clear_hist_interrupt(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	I915_WRITE(BLC_HIST_GUARD,
			I915_READ(BLC_HIST_GUARD) | HISTOGRAM_EVENT_STATUS);
	return 0;
}

int
i915_dpst_enable_hist_interrupt(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 blm_hist_ctl;

	dev_priv->dpst.enabled = true;
	dev_priv->dpst.blc_adjustment = DPST_MAX_FACTOR;

	/* Enable histogram logic to collect data */
	blm_hist_ctl = I915_READ(BLC_HIST_CTL);
	blm_hist_ctl |= IE_HISTOGRAM_ENABLE | HSV_INTENSITY_MODE;
	blm_hist_ctl |= IE_MOD_TABLE_ENABLE | ENHANCEMENT_MODE_MULT;
	I915_WRITE(BLC_HIST_CTL, blm_hist_ctl);

	/* Wait for VBLANK since the histogram enabling logic takes affect
	 * at the next vblank */
	intel_wait_for_vblank(dev, PIPE_A);

	/* Clear pending interrupt bit. Clearing the pending interrupt bit
	 * must be not be done at the same time as enabling the
	 * interrupt. */
	I915_WRITE(BLC_HIST_GUARD,
			I915_READ(BLC_HIST_GUARD) | HISTOGRAM_EVENT_STATUS);

	/* Enable histogram interrupts */
	I915_WRITE(BLC_HIST_GUARD,
			I915_READ(BLC_HIST_GUARD) | HISTOGRAM_INTERRUPT_ENABLE);

	return 0;
}

int
i915_dpst_disable_hist_interrupt(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 blm_hist_guard, blm_hist_ctl;

	dev_priv->dpst.enabled = false;
	dev_priv->dpst.blc_adjustment = DPST_MAX_FACTOR;

	/* Disable histogram interrupts. It is OK to clear pending interrupts
	 * and disable interrupts at the same time. */
	blm_hist_guard = I915_READ(BLC_HIST_GUARD);
	blm_hist_guard |= HISTOGRAM_EVENT_STATUS; /* clear pending interrupts */
	blm_hist_guard &= ~HISTOGRAM_INTERRUPT_ENABLE;
	I915_WRITE(BLC_HIST_GUARD, blm_hist_guard);

	/* Disable histogram logic */
	blm_hist_ctl = I915_READ(BLC_HIST_CTL);
	blm_hist_ctl &= ~IE_HISTOGRAM_ENABLE;
	blm_hist_ctl &= ~IE_MOD_TABLE_ENABLE;
	I915_WRITE(BLC_HIST_CTL, blm_hist_ctl);

	/* Setting blc level to what it would be without dpst adjustment */
	intel_panel_actually_set_backlight(dev,
					dev_priv->backlight.level);

	return 0;
}

static int
i915_dpst_apply_luma(struct drm_device *dev,
		struct dpst_initialize_context *ioctl_data)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 diet_factor, i;
	u32 level;
	u32 blm_hist_ctl;

	if (dev_priv->early_suspended)
		return -EINVAL;

	/* Backlight settings */
	dev_priv->dpst.blc_adjustment =
	ioctl_data->ie_container.dpst_blc_factor;

	level = i915_dpst_compute_brightness(dev, dev_priv->backlight.level);
	if (dev_priv->is_mipi){
		if ((prev_level == level)){
			intel_panel_actually_set_mipi_backlight(dev, level);
		}
		else{
			if (level < prev_level){
				if (prev_level-level>10){
					intel_panel_actually_set_mipi_backlight(dev, level);
					prev_level = level;
				}
				else
					for(;prev_level>level;--prev_level){
						printk("songge-dpst: set level at %d\n", prev_level);
						intel_panel_actually_set_mipi_backlight(dev, prev_level);
					}
			}
			else{
				if (level-prev_level>10){
					intel_panel_actually_set_mipi_backlight(dev, level);
					prev_level = level;
				}
				else	
					for(;prev_level<level;++prev_level){
						printk("songge-dpst: set level at %d\n", prev_level);
						intel_panel_actually_set_mipi_backlight(dev, prev_level);
					}
			}		
		}
	}else
		intel_panel_actually_set_backlight(dev, level);

	/* Setup register to access image enhancement value from
	 * index 0.*/
	blm_hist_ctl = I915_READ(BLC_HIST_CTL);
	blm_hist_ctl |= BIN_REG_FUNCTION_SELECT_IE;
	blm_hist_ctl &= ~BIN_REGISTER_INDEX_MASK;
	I915_WRITE(BLC_HIST_CTL, blm_hist_ctl);

	/* Program the image enhancement data passed from user mode. */
	for (i = 0; i < DPST_DIET_ENTRY_COUNT; i++) {
		diet_factor = ioctl_data->ie_container.
			dpst_ie_st.factor_present[i] * 0x200 / 10000;
		I915_WRITE(BLC_HIST_BIN, diet_factor);
	}

	return 0;
}

static int
i915_dpst_get_bin_data(struct drm_device *dev,
		struct dpst_initialize_context *ioctl_data)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	u32 blm_hist_ctl, blm_hist_bin;
	int index;

	/* Setup register to access bin data from index 0 */
	blm_hist_ctl = I915_READ(BLC_HIST_CTL);
	blm_hist_ctl = blm_hist_ctl & ~(BIN_REGISTER_INDEX_MASK |
						BIN_REG_FUNCTION_SELECT_IE);
	I915_WRITE(BLC_HIST_CTL, blm_hist_ctl);

	/* Read all bin data */
	for (index = 0; index < HIST_BIN_COUNT; index++) {
		blm_hist_bin = I915_READ(BLC_HIST_BIN);

		if (!(blm_hist_bin & BUSY_BIT)) {
			ioctl_data->hist_status.histogram_bins.
				status[index] =	blm_hist_bin & BIN_COUNT_MASK;
		} else {
			/* Engine is busy. Reset index to 0 to grab
			 * fresh histogram data */
			index = -1;
			blm_hist_ctl = I915_READ(BLC_HIST_CTL);
			blm_hist_ctl = blm_hist_ctl & ~BIN_REGISTER_INDEX_MASK;
			I915_WRITE(BLC_HIST_CTL, blm_hist_ctl);
		}
	}
	ioctl_data->hist_status.dpst_disable = !dev_priv->dpst.enabled;

	return 0;
}

static int
i915_dpst_init(struct drm_device *dev,
		struct dpst_initialize_context *ioctl_data)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_crtc *crtc;
	struct drm_display_mode *mode = NULL;
	u32 blm_hist_guard, gb_val;

	/* Get information about current display mode */
	crtc = intel_get_crtc_for_pipe(dev, PIPE_A);
	if (crtc) {
		mode = intel_crtc_mode_get(dev, crtc);
		if (mode) {
			gb_val = (DEFAULT_GUARDBAND_VAL *
					mode->hdisplay * mode->vdisplay)/1000;

			ioctl_data->init_data.threshold_gb = gb_val;
			ioctl_data->init_data.image_res =
					mode->hdisplay*mode->vdisplay;
			/*mode is allocated by kzalloc, need be freed*/
			kfree(mode);
		}
	}

	/* Store info needed to talk to user mode */
	dev_priv->dpst.task = current;
	dev_priv->dpst.signal = ioctl_data->init_data.sig_num;

	/* Setup guardband delays and threshold */
	blm_hist_guard = I915_READ(BLC_HIST_GUARD);
	blm_hist_guard |= (ioctl_data->init_data.gb_delay << 22)
			| ioctl_data->init_data.threshold_gb;
	I915_WRITE(BLC_HIST_GUARD, blm_hist_guard);

	/* Enable histogram interrupts */
	i915_dpst_enable_hist_interrupt(dev);

	return 0;
}


u32
i915_dpst_get_brightness(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;

	if (!dev_priv->dpst.enabled)
		return 0;

	/* return the last (non-dpst) set backlight level */
	return dev_priv->backlight.level;
}

u32
i915_dpst_compute_brightness(struct drm_device *dev, u32 brightness_val)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	u32 backlight_level = brightness_val;

	if (!dev_priv->dpst.enabled)
		return backlight_level;

	/* Calculate the backlight after it has been reduced by "dpst
	 * blc adjustment" percent . blc_adjustment value is stored
	 * after multiplying by 100, so we have to divide by 100 2nd time
	 * to get to the correct value */
	backlight_level = ((brightness_val *
				dev_priv->dpst.blc_adjustment)/100)/100;

	return backlight_level;
}

void
i915_dpst_irq_handler(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;

	/* Notify user mode of the interrupt */
	if (dev_priv->dpst.task != NULL)
		send_sig_info(dev_priv->dpst.signal, SEND_SIG_FORCED,
							dev_priv->dpst.task);
}

int
i915_dpst_context(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	struct dpst_initialize_context *ioctl_data = NULL;
	struct drm_crtc *crtc  = intel_get_crtc_for_pipe(dev, PIPE_A);
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

	int ret = -EINVAL;

	if (!I915_HAS_DPST(dev))
		return -EINVAL;

	if (!data)
		return -EINVAL;

	if (!intel_crtc->active)
		return -EINVAL;

	ioctl_data = (struct dpst_initialize_context *) data;
	switch (ioctl_data->dpst_ioctl_type) {
	case DPST_ENABLE:
		ret = i915_dpst_enable_hist_interrupt(dev);
	break;

	case DPST_DISABLE:
		ret = i915_dpst_disable_hist_interrupt(dev);
	break;

	case DPST_INIT_DATA:
		ret = i915_dpst_init(dev, ioctl_data);
	break;

	case DPST_GET_BIN_DATA:
		ret = i915_dpst_get_bin_data(dev, ioctl_data);
	break;

	case DPST_APPLY_LUMA:
		ret = i915_dpst_apply_luma(dev, ioctl_data);
	break;

	case DPST_RESET_HISTOGRAM_STATUS:
		ret = i915_dpst_clear_hist_interrupt(dev);
	break;

	default:
		DRM_ERROR("Invalid DPST ioctl type\n");
	break;
	}

	return ret;
}
