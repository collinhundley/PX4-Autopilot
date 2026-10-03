/* SPDX-License-Identifier: BSD-3-Clause */
/* A deterministic host model of the USB controller and single LPWORK queue.
 * The firmware build separately checks the real NuttX types and ABI.
 */
#ifndef NCM_TEST_STUBS_H
#define NCM_TEST_STUBS_H
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>
#define FAR
#define OK 0
#define UNUSED(x) ((void)(x))
#define DEBUGASSERT assert
#define DEBUGPANIC() abort()
#define CONFIG_SCHED_LPNTHREADS 1
#define CONFIG_NET_GUARDSIZE 2
#define CONFIG_CDCNCM_COMPOSITE 1
#define CONFIG_COMPOSITE_IAD 1
#define CONFIG_USBDEV_MAXPOWER 100
#define USB_SPEED_FULL 1
#define USBDEV_REQFLAGS_NULLPKT 1
#define LPWORK 0
#define NET_LL_ETHERNET 1
#define IFF_UP 1
#define IFF_RUNNING 2
#define IFF_IS_UP(f) ((f) & IFF_UP)
#undef HTONS
#define HTONS htons
#define ETHTYPE_IP 0x0800
#define ETHTYPE_ARP 0x0806
#define LSBYTE(x) ((x) & 255)
#define MSBYTE(x) (((x) >> 8) & 255)
#define GETUINT16(p) ((p)[0] | ((p)[1] << 8))
#define USB_REQ_SETCONFIGURATION 9
#define USB_REQ_SETINTERFACE 11
#define USB_REQ_GETINTERFACE 10
#define ECM_SET_PACKET_FILTER 0x43
#define USB_DESC_TYPE_STRING 3
#define USB_DESC_TYPE_ENDPOINT 5
#define USB_DESC_TYPE_CONFIG 2
#define USB_DESC_TYPE_INTERFACE 4
#define USB_DESC_TYPE_INTERFACEASSOCIATION 11
#define USB_DESC_TYPE_CSINTERFACE 0x24
#define USB_SIZEOF_EPDESC 7
#define USB_SIZEOF_IFDESC 9
#define USB_SIZEOF_IADDESC 8
#define USB_SIZEOF_CFGDESC 9
#define USB_DIR_IN 0x80
#define USB_DIR_OUT 0
#define USB_EP_ATTR_XFER_INT 3
#define USB_EP_ATTR_XFER_BULK 2
#define USB_CLASS_CDC 2
#define USB_CLASS_CDC_DATA 10
#define CDC_PROTO_NONE 0
#define CDC_DSUBTYPE_HDR 0
#define CDC_DSUBTYPE_UNION 6
#define CDC_DSUBTYPE_ECM 15
#define SIZEOF_HDR_FUNCDESC 5
#define SIZEOF_UNION_FUNCDESC(n) (4 + (n))
#define SIZEOF_ECM_FUNCDESC 13
#define CDCNCM_EP_INTIN_IDX 0
#define CDCNCM_EP_BULKIN_IDX 1
#define CDCNCM_EP_BULKOUT_IDX 2
struct usb_ctrlreq_s {uint8_t type, req, value[2], index[2], len[2];};
struct usb_epdesc_s {uint8_t len, type, addr, attr, mxpacketsize[2], interval;};
struct usb_ifdesc_s {uint8_t len, type, ifno, alt, neps, classid, subclass, protocol, iif;};
struct usb_iaddesc_s {uint8_t len, type, firstif, nifs, classid, subclass, protocol, ifunction;};
struct usb_cfgdesc_s {uint8_t len, type, totallen[2], ninterfaces, cfgvalue, icfg, attr, mxpower;};
struct usb_strdesc_s {uint8_t len, type;};
struct cdc_hdr_funcdesc_s {uint8_t size, type, subtype, cdc[2];};
struct cdc_union_funcdesc_s {uint8_t size, type, subtype, master, slave[1];};
struct cdc_ecm_funcdesc_s {uint8_t size, type, subtype, mac, stats[4], maxseg[2], nmcflts[2], npwrflts;};
struct usbdev_ep_s;
struct usbdev_req_s {uint8_t *buf; unsigned len, xfrd, flags; int result; void (*callback)(struct usbdev_ep_s *, struct usbdev_req_s *);};
struct usbdev_ep_s {void *priv; bool enabled; uint8_t address; struct usbdev_req_s *pending;};
struct usbdev_s {struct usbdev_ep_s *ep0;};
struct usbdevclass_driver_s;
struct usbdevclass_driverops_s {
	int (*bind)(struct usbdevclass_driver_s *, struct usbdev_s *);
	void (*unbind)(struct usbdevclass_driver_s *, struct usbdev_s *);
	int (*setup)(struct usbdevclass_driver_s *, struct usbdev_s *, const struct usb_ctrlreq_s *, uint8_t *, size_t);
	void (*disconnect)(struct usbdevclass_driver_s *, struct usbdev_s *);
	void (*suspend)(struct usbdevclass_driver_s *, struct usbdev_s *);
	void (*resume)(struct usbdevclass_driver_s *, struct usbdev_s *);
};
struct usbdevclass_driver_s {const struct usbdevclass_driverops_s *ops; int speed;};
struct usbdev_devinfo_s {uint8_t ifnobase, strbase, ninterfaces, nstrings, nendpoints, epno[3];};
struct composite_devdesc_s {
	int16_t (*mkconfdesc)(uint8_t *, struct usbdev_devinfo_s *);
	int (*mkstrdesc)(uint8_t, struct usb_strdesc_s *);
	int (*classobject)(int, struct usbdev_devinfo_s *, struct usbdevclass_driver_s **);
	void (*uninitialize)(struct usbdevclass_driver_s *);
	int nconfigs, configid, cfgdescsize;
	struct usbdev_devinfo_s devinfo;
};
struct net_driver_s {
	uint8_t *d_buf; void *d_private; unsigned d_len, d_flags;
	int (*d_ifup)(struct net_driver_s *), (*d_ifdown)(struct net_driver_s *), (*d_txavail)(struct net_driver_s *);
	struct {struct {uint8_t ether_addr_octet[6];} ether;} d_mac;
};
struct eth_hdr_s {uint8_t dst[6], src[6]; uint16_t type;};
struct work_s {void (*worker)(void *); void *arg;};
typedef unsigned irqstate_t;
typedef unsigned sem_t;
static unsigned critical_depth, connects, ep0_submits, input_packets;
static uint8_t control_reply[128];
static unsigned control_length;
static struct usbdev_ep_s endpoints[16];
static struct work_s *work_queue_items[32];
static unsigned work_count;
static void (*input_hook)(struct net_driver_s *);
static irqstate_t enter_critical_section(void) {return critical_depth++;}
static void leave_critical_section(irqstate_t f) {assert(critical_depth == f + 1); critical_depth = f;}
static bool work_available(struct work_s *w) {return !w->worker;}
static int work_queue(int q, struct work_s *w, void (*fn)(void *), void *arg, int delay)
{
	assert(!w->worker && work_count < 32); w->worker = fn; w->arg = arg; work_queue_items[work_count++] = w; return 0;
}
static int work_cancel(int q, struct work_s *w)
{
	for (unsigned i = 0; i < work_count; i++) {
		if (work_queue_items[i] == w) {
			memmove(work_queue_items + i, work_queue_items + i + 1, (--work_count - i)*sizeof(w)); w->worker = NULL; return 0;
		}
	} return -ENOENT;
}
static void run_work(void)
{
	unsigned budget = 100;

	while (work_count) {
		assert(budget--); struct work_s *w = work_queue_items[0];
		memmove(work_queue_items, work_queue_items + 1, --work_count * sizeof(w));
		void (*fn)(void *) = w->worker; w->worker = NULL; fn(w->arg);
	}
}
static int nxsem_init(sem_t *s, int shared, unsigned value) {*s = value; return 0;}
static int nxsem_post(sem_t *s) {++*s; return 0;}
static int nxsem_wait_uninterruptible(sem_t *s) {assert(!critical_depth); run_work(); assert(*s); --*s; return 0;}
static int nxsem_destroy(sem_t *s) {return 0;}
#define kmm_zalloc(n) calloc(1,n)
#define kmm_free free
#define EP_ALLOCREQ(ep) calloc(1,sizeof(struct usbdev_req_s))
#define EP_ALLOCBUFFER(ep,n) calloc(1,n)
#define EP_FREEREQ(ep,r) free(r)
#define EP_FREEBUFFER(ep,p) free(p)
static struct usbdev_ep_s *alloc_ep(unsigned number, bool in)
{
	struct usbdev_ep_s *e = &endpoints[number + (in ? 8 : 0)]; assert(!e->address); e->address = number | (in ? 128 : 0); return e;
}
#define DEV_ALLOCEP(dev,n,in,type) alloc_ep(n,in)
#define DEV_FREEEP(dev,e) do {assert(!(e)->pending); (e)->address=0;} while(0)
#define DEV_CONNECT(dev) (++connects)
static int submit(struct usbdev_ep_s *e, struct usbdev_req_s *r)
{
	assert(e->enabled); assert(!e->pending); e->pending = r; return 0;
}
static void complete(struct usbdev_ep_s *e, int result, unsigned xfrd)
{
	struct usbdev_req_s *r = e->pending; assert(r); e->pending = NULL; r->result = result; r->xfrd = xfrd; r->callback(e, r);
}
static int disable(struct usbdev_ep_s *e)
{
	e->enabled = false;

	if (e->pending) { complete(e, -ESHUTDOWN, 0); } return 0;
}
#define EP_SUBMIT submit
#define EP_DISABLE disable
static int configure(struct usbdev_ep_s *e, struct usb_epdesc_s *d, bool last) {assert(e->address == d->addr); e->enabled = true; return 0;}
#define EP_CONFIGURE configure
static int composite_ep0submit(struct usbdevclass_driver_s *c, struct usbdev_s *d, struct usbdev_req_s *r,
			       const struct usb_ctrlreq_s *ctrl)
{
	++ep0_submits; control_length = r->len; assert(r->len <= sizeof(control_reply)); memcpy(control_reply, r->buf, r->len); return 0;
}
static int netdev_register(struct net_driver_s *d, int type) {return 0;}
static int netdev_unregister(struct net_driver_s *d) {return 0;}
static void net_lock(void) {}
static void net_unlock(void) {}
static void netdev_carrier_on(struct net_driver_s *d) {d->d_flags |= IFF_RUNNING;}
static void netdev_carrier_off(struct net_driver_s *d) {d->d_flags &= ~IFF_RUNNING;}
static void arp_out(struct net_driver_s *d) {}
static void arp_ipin(struct net_driver_s *d) {}
static void ipv4_input(struct net_driver_s *d) {++input_packets; d->d_len = 0; if (input_hook)input_hook(d);}
static void arp_arpin(struct net_driver_s *d) {++input_packets; d->d_len = 0;}
static bool devif_loopback(struct net_driver_s *d) {return false;}
static void devif_poll(struct net_driver_s *d, int (*fn)(struct net_driver_s *)) {}
#endif
