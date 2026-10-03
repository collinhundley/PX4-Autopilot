/* SPDX-License-Identifier: BSD-3-Clause */
/* Exercise the actual DHCP option parser and lifecycle state machine. */
#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>
#include <time.h>
#undef HTONS
#undef HTONL
#ifndef SOCK_CLOEXEC
#define SOCK_CLOEXEC 0
#endif
#define CONFIG_NETUTILS_DHCPD_HOST 1
#define CONFIG_NETUTILS_DHCPD_DAEMON_TIMEOUT 2
#define CONFIG_NETUTILS_DHCPD_STACKSIZE 2048
#define CONFIG_NETUTILS_DHCPD_PRIORITY 50
#define CONFIG_NETUTILS_DHCPD_SIGWAKEUP 22
#define CONFIG_NETUTILS_DHCPD_STARTIP 0xc0a8cb02
#define CONFIG_NETUTILS_DHCPD_MAXLEASES 1
#define CONFIG_NETUTILS_DHCPD_ROUTERIP 0
#define CONFIG_NETUTILS_DHCPD_DNSIP 0
#define CONFIG_NETUTILS_DHCPD_NETMASK 0xfffffffc
#define SEM_INITIALIZER(n) {n}
typedef struct {int value;} sem_t;
static void (*wait_hook)(void);
static int task_result = 123;
static int sem_post(sem_t *s) {++s->value; return 0;}
static int sem_timedwait(sem_t *s, const struct timespec *t)
{
	if (!s->value && wait_hook) { wait_hook(); }

	if (!s->value) {errno = ETIMEDOUT; return -1;}

	--s->value; return 0;
}
static int task_create(const char *name, int priority, int stack, int (*entry)(int, char **), char **argv)
{
	if (task_result < 0) { errno = ENOMEM; }

	return task_result;
}
int dhcpd_run(const char *interface);
#include "dhcpd.c"

static void ready(void) {g_dhcpd_daemon.ds_state = DHCPD_RUNNING; sem_post(&g_dhcpd_daemon.ds_sync);}
static void stopped(void) {g_dhcpd_daemon.ds_state = DHCPD_STOPPED; sem_post(&g_dhcpd_daemon.ds_sync);}
static void reset_daemon(void)
{
	g_dhcpd_daemon.ds_state = DHCPD_STOPPED; g_dhcpd_daemon.ds_lock.value = 1; g_dhcpd_daemon.ds_sync.value = 0; wait_hook = NULL;
	task_result = 123;
}
int main(void)
{
	reset_daemon(); task_result = -1; assert(dhcpd_start("eth0") == -ENOMEM);
	reset_daemon(); assert(dhcpd_start("eth0") == -ETIMEDOUT);
	assert(g_dhcpd_daemon.ds_state == DHCPD_STOP_REQUESTED);
	assert(dhcpd_start("eth0") == -EIO); /* no second daemon during teardown */
	wait_hook = stopped; assert(dhcpd_stop() == 0);
	reset_daemon(); wait_hook = ready; assert(dhcpd_start("eth0") == 0);
	assert(dhcpd_start("eth0") == 0); /* idempotent */
	wait_hook = NULL; assert(dhcpd_stop() == -ETIMEDOUT);
	wait_hook = stopped; assert(dhcpd_stop() == 0);

	size_t base = offsetof(struct dhcpmsg_s, options);
	uint8_t *p = g_state.ds_inpacket.options;
	memset(&g_state, 0, sizeof(g_state)); memcpy(p, g_magiccookie, 4);
	p[4] = 53; p[5] = 1; p[6] = DHCPDISCOVER; p[7] = 255;
	assert(dhcpd_parseoptions(base + 8)); assert(g_state.ds_optmsgtype == DHCPDISCOVER);

	for (size_t len = 4; len < 8; len++) { assert(!dhcpd_parseoptions(base + len)); }
	p[4] = 255; assert(!dhcpd_parseoptions(base + 4)); /* END outside received bytes */

	for (unsigned code = 1; code < 255; code++) {
		p[4] = code; assert(!dhcpd_parseoptions(base + 5));
		p[5] = 255; assert(!dhcpd_parseoptions(base + 8));
	}

	/* Last byte of the entire options array has no length byte. */
	memset(p + 4, 0, sizeof(g_state.ds_inpacket.options) - 4);
	p[sizeof(g_state.ds_inpacket.options) - 1] = 53;
	assert(!dhcpd_parseoptions(sizeof(g_state.ds_inpacket)));
	puts("DHCP: bounded start/stop, idempotence, startup failure and truncated options passed");
}
