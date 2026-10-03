/* SPDX-License-Identifier: BSD-3-Clause */
#include "cdcncm.c"
#include <stdio.h>

static struct usbdev_ep_s ep0;
static struct usbdev_s controller = {&ep0};
static struct cdcncm_driver_s *device;

static int request(unsigned type, unsigned req, unsigned value, unsigned index, unsigned len, uint8_t *out, size_t outlen)
{
	struct usb_ctrlreq_s c = {.type = type, .req = req};
	ncm_put16(c.value, value); ncm_put16(c.index, index); ncm_put16(c.len, len);
	return cdcncm_setup(&device->usbdev, &controller, &c, out, outlen);
}

static void descriptors(void)
{
	struct composite_devdesc_s desc;
	cdcncm_get_composite_devdesc(&desc);
	assert(desc.devinfo.ninterfaces == 2 && desc.devinfo.nendpoints == 3);
	uint8_t bytes[128] = {0};
	int len = desc.mkconfdesc(bytes, &device->devinfo);
	assert(len == desc.cfgdescsize && len == 85);
	const uint8_t iad[] = {8, 11, 2, 2, 2, 13, 0, 0};
	assert(memcmp(bytes, iad, sizeof(iad)) == 0);
	unsigned interfaces = 0, eps = 0, ncm = 0;

	for (int i = 0; i < len; i += bytes[i]) {
		assert(bytes[i] >= 2 && i + bytes[i] <= len);

		if (bytes[i + 1] == 4) {
			assert(bytes[i + 2] == (interfaces ? 3 : 2));

			if (interfaces == 1) { assert(bytes[i + 3] == 0 && bytes[i + 4] == 0); }

			if (interfaces == 2) { assert(bytes[i + 3] == 1 && bytes[i + 4] == 2); }

			++interfaces;
		}

		if (bytes[i + 1] == 5) {const uint8_t addr[] = {0x84, 0x85, 6}; assert(bytes[i + 2] == addr[eps++]);}

		if (bytes[i + 1] == 0x24 && bytes[i + 2] == 0x1a) {assert(ncm_get16(bytes + i + 3) == 0x100 && bytes[i + 5] == 1); ++ncm;}
	}

	assert(interfaces == 3 && eps == 3 && ncm == 1);
	uint8_t str[26];
	assert(desc.mkstrdesc(1, (struct usb_strdesc_s *)str) == 26);
	assert(str[0] == 26 && str[1] == 3 && str[2] == '0' && str[4] == '2');
}

static void configure_link(void)
{
	assert(request(0, 9, 1, 0, 0, NULL, 0) == 0);
	assert(request(0xa1, 0x80, 0, 2, 28, NULL, 0) == 0);
	assert(control_length == 28 && ncm_get32(control_reply + 4) == 2048 && ncm_get32(control_reply + 16) == 2048);
	assert(ncm_get16(control_reply + 20) == 4 && ncm_get16(control_reply + 22) == 0);
	uint8_t capacity[4] = {0, 8, 0, 0};
	assert(request(0x21, 0x86, 0, 2, 4, capacity, 3) < 0);
	assert(request(0x21, 0x86, 0, 2, 4, capacity, 4) == 0);
	capacity[1] = 4;
	assert(request(0x21, 0x86, 0, 2, 4, capacity, 4) < 0);
	assert(request(0x21, 0x84, 1, 2, 0, NULL, 0) < 0); /* NTB32 rejected */
	assert(request(0x21, 0x84, 0, 2, 0, NULL, 0) == 0);
	assert(request(0x01, 11, 2, 3, 0, NULL, 0) < 0);
	assert(request(0x01, 11, 1, 2, 0, NULL, 0) < 0);
	assert(request(0x01, 11, 1, 3, 0, NULL, 0) == 0);
	assert(request(0x81, 10, 0, 3, 1, NULL, 0) == 0 && control_reply[0] == 1);
	assert(request(0x21, 0x43, 0x20, 2, 0, NULL, 0) < 0);
	assert(request(0x21, 0x43, 0x0c, 2, 0, NULL, 0) == 0);
	run_work();
	assert(device->rxqueued && device->notifybusy && cdcncm_active(device));
	assert(device->notifyreq->buf[1] == 0x2a);
	complete(device->epint, 0, 16); run_work();
	assert(device->notifyreq->buf[1] == 0 && ncm_get16(device->notifyreq->buf + 2) == 1);
	complete(device->epint, 0, 8); run_work();
}

static void reset_during_input(struct net_driver_s *dev)
{
	dev->d_len = 60;
	cdcncm_disconnect(&device->usbdev, &controller);
}

static void pending_reset(void)
{
	uint8_t frame[60] = {0}; frame[12] = 8;
	unsigned len = ncm_encode(device->rdreq->buf, frame, sizeof(frame), 0);
	input_hook = reset_during_input;
	complete(device->epbulkout, 0, len); run_work();
	assert(input_packets == 1 && !device->txbusy && !device->rxqueued);
	assert(!cdcncm_active(device));
	input_hook = NULL;
	configure_link();
	assert(!device->replypending);
	device->dev.d_len = 60;
	assert(cdcncm_transmit(device));
	uint8_t saved[90]; memcpy(saved, device->wrreq->buf, 90);
	memset(device->dev.d_buf, 42, 60);
	assert(!cdcncm_transmit(device));
	assert(memcmp(saved, device->wrreq->buf, 90) == 0); /* in-flight buffer immutable */
	cdcncm_disconnect(&device->usbdev, &controller); run_work();
	assert(!device->txbusy && !device->rxqueued && !device->notifybusy);
	configure_link();
	cdcncm_suspend(&device->usbdev, &controller); run_work();
	assert(!(device->dev.d_flags & IFF_RUNNING));
	cdcncm_resume(&device->usbdev, &controller); run_work();
	assert(device->dev.d_flags & IFF_RUNNING);
}

int main(void)
{
	const uint8_t mac[6] = {2, 1, 2, 3, 4, 5}, host[6] = {2, 1, 2, 3, 4, 4};
	cdcncm_setmac(mac, host);
	struct usbdev_devinfo_s info = {.ifnobase = 2, .strbase = 6, .ninterfaces = 2, .epno = {4, 5, 6}};
	struct usbdevclass_driver_s *driver;
	assert(cdcncm_classobject(0, &info, &driver) == 0);
	/* Composite may fail before it reaches this member's bind callback. */
	irqstate_t early_flags = enter_critical_section();
	cdcncm_unbind(driver, &controller);
	leave_critical_section(early_flags);
	assert(work_count == 0);
	cdcncm_uninitialize(driver);
	cdcncm_uninitialize(driver);
	assert(cdcncm_classobject(0, &info, &driver) == 0);
	device = (struct cdcncm_driver_s *)driver;
	assert(cdcncm_bind(driver, &controller) == 0);
	descriptors(); configure_link(); pending_reset();
	cdcncm_schedule(device);
	cdcncm_uninitialize(driver); /* drains pending work before unregister */
	assert(work_count == 0);
	irqstate_t flags = enter_critical_section();
	cdcncm_unbind(driver, &controller); /* composite unbind holds IRQ lock */
	leave_critical_section(flags);
	assert(work_count == 0);
	cdcncm_uninitialize(driver);
	puts("NCM driver: descriptors, negotiation, alternate settings, filters, notifications, reset and teardown passed");
}
