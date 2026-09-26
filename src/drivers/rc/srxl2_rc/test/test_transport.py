#!/usr/bin/env python3
"""Exercise the actual STM32H7 timed UART functions with fake UART/DMA registers.

No hardware behavior is simulated beyond explicit register transitions. This
checks admission and ownership invariants; physical timing remains a bench test.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[5]
NUTTX = ROOT / "platforms/nuttx/NuttX/nuttx"
source = (NUTTX / "arch/arm/src/stm32h7/stm32_serial.c").read_text()
header = (NUTTX / "include/nuttx/serial/timed_halfduplex.h").read_text()
state_start = source.index("struct up_timed_s\n{")
state = source[state_start:source.index("\n};", state_start) + 3]
helper_start = source.index("/* A timed port currently supports")
helper_end = source.index("static int up_timed_configure(", helper_start)
helpers = source[helper_start:helper_end]
tx_start = source.index("static int up_timed_transmit(", helper_end)
configure = source[helper_end:tx_start]
transmit = source[tx_start:source.index("static int up_timed_ioctl(", tx_start)]
public = header[header.index("struct serial_thdx_config_s"):header.index("#endif /*")]

preamble = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#define SERIAL_THDX_MAX_PACKET 80
#define ARMV7M_DCACHE_LINESIZE 32
#define aligned_data(x) __attribute__((aligned(x)))
#define SERIAL_HAVE_TXDMA 1
#define CONFIG_SERIAL_IFLOWCONTROL 1
#define CONFIG_SERIAL_OFLOWCONTROL 1
#define CONFIG_ARMV7M_DCACHE 1
#define RXDMA_BUFFER_SIZE 32
#define GPIO_PUPD_MASK 0x30
#define GPIO_OPENDRAIN 0x40
#define GPIO_PULLUP 0x10
#define OK 0
#define STM32_USART_CR1_OFFSET 0
#define STM32_USART_CR2_OFFSET 1
#define STM32_USART_CR3_OFFSET 2
#define STM32_USART_ISR_OFFSET 3
#define STM32_USART_ICR_OFFSET 4
#define STM32_USART_RQR_OFFSET 5
#define STM32_USART_TDR_OFFSET 6
#define USART_CR1_UE (1u << 0)
#define USART_CR1_RE (1u << 2)
#define USART_CR1_TE (1u << 3)
#define USART_CR1_TXEIE (1u << 7)
#define USART_CR1_TCIE (1u << 6)
#define USART_CR2_RXINV (1u << 16)
#define USART_CR2_TXINV (1u << 17)
#define USART_CR2_SWAP (1u << 15)
#define USART_CR3_DMAT (1u << 7)
#define USART_CR3_DMAR (1u << 6)
#define USART_CR3_HDSEL (1u << 3)
#define USART_ISR_PE (1u << 0)
#define USART_ISR_FE (1u << 1)
#define USART_ISR_NE (1u << 2)
#define USART_ISR_ORE (1u << 3)
#define USART_ISR_IDLE (1u << 4)
#define USART_ISR_RXNE (1u << 5)
#define USART_ISR_TC (1u << 6)
#define USART_ISR_TXE (1u << 7)
#define USART_ISR_BUSY (1u << 16)
#define USART_ICR_IDLECF USART_ISR_IDLE
#define USART_ICR_TCCF USART_ISR_TC
#define USART_RQR_TXFRQ (1u << 4)
#define USART_RQR_RXFRQ (1u << 3)
#define DMA_STATUS_ERROR (1u << 3)
#define DMA_STATUS_HTIF (1u << 4)
#define DMA_STATUS_TCIF (1u << 5)
#define SERIAL_TXDMA_CONTROL_WORD 0x1234

typedef void *DMA_HANDLE;
struct uart_dev_s {
  void *priv;
  unsigned open_count;
  bool exclusive;
  struct { unsigned head, tail; } xmit, recv;
  struct { unsigned length, nlength; } dmatx;
};
struct stm32_dma_config_s {
  uint32_t paddr, maddr, ndata, cfg1, cfg2;
};
"""
stubs = r"""
struct up_dev_s {
  struct uart_dev_s dev;
  struct up_timed_s timed;
  void *rxdma, *txdma;
  uint32_t rxdmanext, sr, usartbase, baud, tx_gpio, rxdmaavail;
  uint16_t ie;
  uint8_t bits, parity;
  bool stopbits2, iflow, oflow;
};
static uint32_t registers[7], rx_position, tx_residual, configured_gpio;
static uint64_t now_us;
static unsigned dma_starts, dma_stops;
static bool cache_cleaned;
static struct stm32_dma_config_s last_dma_config;
static uint64_t test_clock(void) { return now_us; }
static uint32_t up_serialin(struct up_dev_s *p, int offset)
{ (void)p; return registers[offset]; }
static void up_serialout(struct up_dev_s *p, int offset, uint32_t value)
{
  (void)p;
  if (offset == STM32_USART_ICR_OFFSET) {
    registers[STM32_USART_ISR_OFFSET] &= ~value;
  } else {
    registers[offset] = value;
  }
}
static void up_serialmod(struct up_dev_s *p, int offset, uint32_t clear, uint32_t set)
{ (void)p; registers[offset] = (registers[offset] & ~clear) | set; }
static void up_setusartint(struct up_dev_s *p, uint16_t ie)
{
  p->ie = ie;
  registers[STM32_USART_CR1_OFFSET] &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);
  registers[STM32_USART_CR1_OFFSET] |= ie;
}
static uint32_t up_dma_nextrx(struct up_dev_s *p) { (void)p; return rx_position; }
static uint32_t stm32_dmaresidual(void *dma) { (void)dma; return tx_residual; }
static void up_clean_dcache(uintptr_t start, uintptr_t end)
{ assert(end > start); cache_cleaned = true; }
static void stm32_dmastop(void *dma) { (void)dma; ++dma_stops; }
static void stm32_configgpio(uint32_t gpio) { configured_gpio = gpio; }
static void stm32_dmasetup(void *dma, struct stm32_dma_config_s *config)
{ (void)dma; last_dma_config = *config; tx_residual = config->ndata; }
static void stm32_dmastart(void *dma, void (*callback)(DMA_HANDLE,uint8_t,void*),
                          void *arg, bool half)
{
  (void)dma; (void)callback; (void)arg;
  assert(!half && cache_cleaned);
  assert((registers[STM32_USART_CR1_OFFSET] & USART_CR1_RE) == 0);
  assert(registers[STM32_USART_CR3_OFFSET] & USART_CR3_DMAT);
  ++dma_starts;
}
"""
tests = r"""
static struct up_dev_s port;
static uint8_t bytes[22];
static struct serial_thdx_tx_s packet;
static void reset(void)
{
  memset(&port, 0, sizeof(port));
  memset(registers, 0, sizeof(registers));
  memset(bytes, 0x5a, sizeof(bytes));
  memset(&packet, 0, sizeof(packet));
  port.dev.priv = &port;
  port.dev.open_count = 1;
  port.dev.exclusive = true;
  port.baud = 115200; port.bits = 8; port.tx_gpio = 0x123400;
  port.rxdma = (void *)1;
  port.txdma = (void *)2;
  port.timed.enabled = true;
  port.timed.clock_us = test_clock;
  port.timed.received_sequence = 14;
  port.timed.delivered_sequence = 14;
  port.timed.idle_end_sequence = 14;
  port.timed.idle_start_lower_us = 97000;
  port.timed.idle_observed_us = 99900;
  port.timed.idle_valid = true;
  port.timed.saved_cr1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
  port.timed.saved_cr3 = USART_CR3_DMAR | USART_CR3_DMAT;
  registers[STM32_USART_CR1_OFFSET] = port.timed.saved_cr1;
  registers[STM32_USART_CR3_OFFSET] = port.timed.saved_cr3 | USART_CR3_HDSEL;
  registers[STM32_USART_ISR_OFFSET] = USART_ISR_TC | USART_ISR_TXE;
  rx_position = tx_residual = dma_starts = dma_stops = configured_gpio = 0;
  cache_cleaned = false;
  now_us = 100000;
  packet.buffer = bytes;
  packet.length = sizeof(bytes);
  packet.request_end_sequence = 14;
  packet.frame_period_us = 11000;
  packet.max_idle_age_us = 2000;
  packet.guard_us = 100;
}
static void rejected(int expected)
{
  assert(up_timed_transmit(&port.dev, &packet) == expected);
  assert(dma_starts == 0 && !port.timed.tx_busy);
  assert(registers[STM32_USART_CR1_OFFSET] & USART_CR1_RE);
}
int main(void)
{
  // A reply owns its DMA buffer and cannot depend on caller memory lifetime.
  reset();
  assert(up_timed_transmit(&port.dev, &packet) == 0);
  assert(dma_starts == 1 && last_dma_config.ndata == sizeof(bytes));
  bytes[0] = 0;
  assert(port.timed.tx_buffer[0] == 0x5a);
  assert(port.timed.tx_busy && !(registers[0] & USART_CR1_RE));

  // DMA completion/TXE does not restore RX before the last UART stop bit.
  tx_residual = 0;
  up_timed_dma_callback(port.txdma, DMA_STATUS_TCIF, &port);
  port.sr = USART_ISR_TXE;
  assert(up_timed_interrupt(&port));
  assert(port.timed.tx_busy && !(registers[0] & USART_CR1_RE));
  registers[3] |= USART_ISR_TC;
  now_us += 1910;
  assert(up_timed_interrupt(&port));
  assert(!port.timed.tx_busy && (registers[0] & USART_CR1_RE));
  assert(registers[3] & USART_ISR_TC); // permits a subsequent packet
  assert(port.timed.tx_complete_us == now_us);

  reset(); now_us = 101901; rejected(-ETIMEDOUT); // response-age cap
  reset(); now_us = 101000; port.timed.idle_observed_us = 100900;
  rejected(-ETIMEDOUT); // delayed IDLE ISR within 11 ms cannot hide true age
  reset(); port.timed.idle_start_lower_us = 98500;
  port.timed.idle_observed_us = 100800; now_us = 100900;
  packet.max_idle_age_us = 1000; rejected(-ETIMEDOUT);
  reset(); port.timed.idle_start_lower_us = 98500;
  packet.max_idle_age_us = 1000;
  assert(up_timed_transmit(&port.dev, &packet) == 0); // fresh handshake

  reset(); packet.max_idle_age_us = 1000; now_us = 100901;
  rejected(-ETIMEDOUT); // shorter handshake cap
  reset(); now_us = 99986; rejected(-EAGAIN); // extra idle character
  reset(); port.timed.idle_start_lower_us = 89000;
  rejected(-ETIMEDOUT); // delayed ISR must not rejuvenate an old packet
  reset(); packet.request_end_sequence = 13; rejected(-ESTALE);
  reset(); port.timed.delivered_sequence = 13; rejected(-ESTALE);
  reset(); port.dev.recv.head = 1; rejected(-ESTALE);
  reset(); port.timed.idle_first_sequence = 1; rejected(-ENODATA);
  reset(); port.timed.idle_valid = false; rejected(-ENODATA);
  reset(); port.timed.burst_active = true; rejected(-EAGAIN);
  reset(); port.dev.open_count = 2; rejected(-EBUSY);
  reset(); port.dev.xmit.head = 1; rejected(-EBUSY);
  reset(); registers[3] |= USART_ISR_BUSY; rejected(-ESTALE);
  reset(); registers[3] |= USART_ISR_RXNE; rejected(-ESTALE);
  reset(); rx_position = 1; rejected(-ESTALE);
  reset(); registers[3] &= ~USART_ISR_TC; rejected(-EBUSY);
  reset(); packet.length = 81; rejected(-EINVAL);

  // DMA faults release the receiver and latch an error; no late retry occurs.
  reset(); assert(up_timed_transmit(&port.dev, &packet) == 0);
  up_timed_dma_callback(port.txdma, DMA_STATUS_ERROR, &port);
  assert(!port.timed.tx_busy && port.timed.tx_fault);
  assert(registers[0] & USART_CR1_RE);
  assert(up_timed_transmit(&port.dev, &packet) == -EIO);
  assert(dma_starts == 1);

  // A lost completion IRQ is recovered from TC without falsely faulting TX.
  reset(); assert(up_timed_transmit(&port.dev, &packet) == 0);
  now_us += 3000; tx_residual = 0; registers[3] |= USART_ISR_TC;
  up_timed_service(&port);
  assert(!port.timed.tx_busy && !port.timed.tx_fault);
  assert(registers[0] & USART_CR1_RE);

  // A missing DMA/error callback cannot leave RX disabled indefinitely.
  reset(); assert(up_timed_transmit(&port.dev, &packet) == 0);
  now_us = port.timed.tx_deadline_us + 1;
  up_timed_observe(&port); // the existing periodic DMA poll drives recovery
  assert(!port.timed.tx_busy && port.timed.tx_fault && dma_stops == 2);
  assert(registers[0] & USART_CR1_RE);

  // Start capture uses an earlier empty observation, not DMA delivery time.
  reset(); memset(&port.timed, 0, sizeof(port.timed));
  port.timed.enabled = true; port.timed.clock_us = test_clock;
  up_timed_observe(&port);
  assert(port.timed.quiet_before_us == now_us - TIMED_CHARACTER_US);
  uint64_t lower = port.timed.quiet_before_us;
  now_us += 600; rx_position = 14;
  up_timed_observe(&port);
  assert(port.timed.burst_active && port.timed.burst_valid);
  assert(port.timed.burst_start_lower_us == lower);
  port.timed.received_sequence = 14; port.rxdmanext = 14;
  now_us += 100; up_timed_idle(&port, now_us);
  struct serial_thdx_status_s status;
  up_timed_snapshot(&port, &status);
  assert(status.timing_valid && status.first_sequence == 0 && status.end_sequence == 14);
  assert(status.start_lower_bound_us == lower && status.idle_observed_us == now_us);

  // A delayed old IDLE interrupt cannot mark a newly busy bus as idle.
  port.timed.burst_active = true; registers[3] |= USART_ISR_BUSY;
  up_timed_idle(&port, now_us);
  assert(port.timed.burst_active && !port.timed.idle_valid);
  // A full DMA wrap can leave exactly the old position. A long observation
  // gap must discard metadata and break application sequence continuity.
  reset(); port.timed.last_observation_us = now_us - TIMED_MAX_RX_GAP_US;
  port.dev.recv.head = 4; port.dev.recv.tail = 1;
  up_timed_observe(&port);
  assert(!port.timed.idle_valid && !port.timed.burst_active);
  assert(port.timed.received_sequence == 14 + RXDMA_BUFFER_SIZE);
  assert(port.timed.delivered_sequence == port.timed.received_sequence);
  assert(port.dev.recv.head == port.dev.recv.tail && port.timed.rx_overruns == 1);
  assert(port.timed.quiet_before_us == 0);

  // Coalesced half/full flags make continuity unknown even if position agrees.
  reset(); up_timed_rx_status(&port, DMA_STATUS_HTIF | DMA_STATUS_TCIF);
  assert(!port.timed.idle_valid && port.timed.rx_overruns == 1);
  assert(port.timed.delivered_sequence == 14 + RXDMA_BUFFER_SIZE);
  reset(); up_timed_rx_status(&port, DMA_STATUS_HTIF);
  assert(port.timed.idle_valid && port.timed.rx_overruns == 0);
  reset(); up_timed_rx_status(&port, DMA_STATUS_ERROR);
  assert(!port.timed.idle_valid && port.timed.rx_overruns == 1);
  reset(); port.timed.rx_gap_pending = true; up_timed_observe(&port);
  assert(!port.timed.rx_gap_pending && port.timed.rx_overruns == 1);

  // Stop during TX releases hardware, restores registers/GPIO and resets state.
  reset(); assert(up_timed_transmit(&port.dev, &packet) == 0);
  struct serial_thdx_config_s config = { .clock_us = test_clock, .enable = false };
  uint32_t saved_cr1 = port.timed.saved_cr1, saved_cr3 = port.timed.saved_cr3;
  assert(up_timed_configure(&port.dev, &config) == 0);
  assert(!port.timed.enabled && !port.timed.tx_busy && !port.timed.tx_fault);
  assert(configured_gpio == port.tx_gpio);
  assert(registers[0] == saved_cr1 && registers[2] == saved_cr3);
  assert((registers[2] & USART_CR3_HDSEL) == 0);
  assert(registers[5] == (USART_RQR_TXFRQ | USART_RQR_RXFRQ));

  // Closing/reopening the UART frees/resets its DMA; a fresh claim restarts.
  tx_residual = 0; config.enable = true;
  port.dev.recv.head = 4; port.dev.recv.tail = 1;
  rx_position = RXDMA_BUFFER_SIZE;
  assert(up_timed_configure(&port.dev, &config) == 0);
  assert(port.timed.enabled && !port.timed.tx_fault && port.rxdmanext == 0);
  assert(port.dev.recv.head == port.dev.recv.tail);
  assert((registers[2] & USART_CR3_HDSEL) != 0);
  assert((configured_gpio & (GPIO_OPENDRAIN | GPIO_PUPD_MASK)) ==
         (GPIO_OPENDRAIN | GPIO_PULLUP));
  assert(up_timed_configure(&port.dev, &config) == -EBUSY); // duplicate enable

  // Refusal paths make no pin/serial configuration changes.
  reset(); port.timed.enabled = false; port.dev.exclusive = false;
  assert(up_timed_configure(&port.dev, &config) == -EBUSY);
  assert(configured_gpio == 0);
  port.dev.exclusive = true; port.dev.open_count = 2;
  assert(up_timed_configure(&port.dev, &config) == -EBUSY);
  port.dev.open_count = 1; port.baud = 57600;
  assert(up_timed_configure(&port.dev, &config) == -ENOTSUP);
  port.baud = 115200; port.bits = 7;
  assert(up_timed_configure(&port.dev, &config) == -ENOTSUP);
  port.bits = 8; port.parity = 1;
  assert(up_timed_configure(&port.dev, &config) == -ENOTSUP);
  port.parity = 0; port.stopbits2 = true;
  assert(up_timed_configure(&port.dev, &config) == -ENOTSUP);
  port.stopbits2 = false; port.iflow = true;
  assert(up_timed_configure(&port.dev, &config) == -EINVAL);
  port.iflow = false; port.oflow = true;
  assert(up_timed_configure(&port.dev, &config) == -EINVAL);
  assert(configured_gpio == 0);
  puts("SRXL2 timed UART: admission, DMA/TC, RX gaps, lost IRQ, stop/restart and configuration checks passed");
  return 0;
}
"""

compiler = shutil.which("cc")
if not compiler:
    raise SystemExit("A native C compiler is required")
with tempfile.TemporaryDirectory(prefix="srxl2-transport-test-") as directory:
    c_file = Path(directory) / "transport_test.c"
    executable = Path(directory) / "transport_test"
    c_file.write_text(preamble + public + state + stubs + helpers + configure + transmit + tests)
    subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-Wno-pointer-to-int-cast", str(c_file), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
