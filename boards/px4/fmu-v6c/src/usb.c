/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file usb.c
 *
 * Board-specific USB functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_platform_common/px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <arm_internal.h>
#include <chip.h>
#include <stm32_gpio.h>
#include <stm32_otg.h>
#include "board_config.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the PX4FMU board.
 *
 ************************************************************************************/

__EXPORT void stm32_usbinitialize(void)
{
	/* The OTG FS has an internal soft pull-up */

	/* Configure the OTG FS VBUS sensing GPIO, Power On, and Overcurrent GPIOs */

#ifdef CONFIG_STM32H7_OTGFS
	stm32_configgpio(GPIO_OTGFS_VBUS);
#endif
}

/************************************************************************************
 * Name:  stm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the USBDEV driver is
 *   used.  This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc.
 *   while the USB is suspended.
 *
 ************************************************************************************/

__EXPORT void stm32_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
	uinfo("resume: %d\n", resume);
}

#ifdef CONFIG_NET_CDCNCM
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/cdcncm.h>
#include <nuttx/usb/composite.h>
#include <px4_platform_common/board_common.h>

int board_composite_initialize(int port)
{
	return port == 0 ? 0 : -EINVAL;
}

void *board_composite_connect(int port, int configid)
{
	if (port != 0 || configid != 0) {
		return NULL;
	}

	struct composite_devdesc_s dev[2];

	uuid_uint32_t uid;

	board_get_uuid32(uid);

	// FNV-1a folds the complete chip UID into a stable locally administered
	// address. The last bit distinguishes the two ends of this USB link.
	uint64_t hash = UINT64_C(14695981039346656037);

	for (unsigned i = 0; i < sizeof(uid); ++i) {
		hash = (hash ^ ((const uint8_t *)uid)[i]) * UINT64_C(1099511628211);
	}

	uint8_t device_mac[6] = {0x02};
	uint8_t host_mac[6];

	for (unsigned i = 1; i < sizeof(device_mac); ++i) {
		device_mac[i] = hash >> (8 * (i - 1));
	}

	memcpy(host_mac, device_mac, sizeof(host_mac));
	host_mac[5] ^= 1;
	cdcncm_setmac(device_mac, host_mac);

	cdcacm_get_composite_devdesc(&dev[0]);
	dev[0].classobject = cdcacm_classobject;
	dev[0].uninitialize = cdcacm_uninitialize;
	dev[0].minor = 0;
	dev[0].devinfo.ifnobase = 0;
	dev[0].devinfo.strbase = COMPOSITE_NSTRIDS;
	dev[0].devinfo.epno[CDCACM_EP_INTIN_IDX] = 1;
	dev[0].devinfo.epno[CDCACM_EP_BULKIN_IDX] = 2;
	dev[0].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = 3;

	cdcncm_get_composite_devdesc(&dev[1]);
	dev[1].minor = 0;
	dev[1].devinfo.ifnobase = 2;
	dev[1].devinfo.strbase = COMPOSITE_NSTRIDS + dev[0].devinfo.nstrings;
	dev[1].devinfo.epno[CDCNCM_EP_INTIN_IDX] = 4;
	dev[1].devinfo.epno[CDCNCM_EP_BULKIN_IDX] = 5;
	dev[1].devinfo.epno[CDCNCM_EP_BULKOUT_IDX] = 6;
	return composite_initialize(2, dev);
}
#endif
