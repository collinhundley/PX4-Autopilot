/* SPDX-License-Identifier: BSD-3-Clause */
/* Host regression/fuzz tests for the actual backported NTB16 codec. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "cdcncm_ntb.h"

static uint8_t block[NCM_NTB_SIZE];
static uint8_t frame[NCM_FRAME_SIZE];
static struct ncm_frame_s frames[NCM_MAX_DATAGRAMS];

static void single_frame(void)
{
	for (size_t size = 14; size <= NCM_FRAME_SIZE; size++) {
		memset(frame, (unsigned char)size, size);
		size_t len = ncm_encode(block, frame, size, 65535);
		assert(len == size + NCM_TX_OFFSET);
		assert(ncm_decode(block, len, frames) == 1);
		assert(frames[0].length == size);
		assert(((frames[0].offset + 14) & 3) == 0);
		assert(memcmp(block + frames[0].offset, frame, size) == 0);
		assert(ncm_get16(block + 6) == 65535);

		for (size_t truncated = 0; truncated < len; truncated++) {
			assert(ncm_decode(block, truncated, frames) == -1);
		}
	}

	assert(ncm_encode(block, frame, 13, 0) == 0);
	assert(ncm_encode(block, frame, NCM_FRAME_SIZE + 1, 0) == 0);
}

static void malformed(void)
{
	const unsigned fields[] = {4, 8, 10, 16, 18, 20, 22, 24, 26};

	for (unsigned f = 0; f < sizeof(fields) / sizeof(fields[0]); f++)
		for (unsigned val = 0; val <= UINT16_MAX; val++) {
			ncm_encode(block, frame, 60, 0);
			ncm_put16(block + fields[f], val);
			int n = ncm_decode(block, 90, frames);

			if (n >= 0)
				for (int i = 0; i < n; i++) {
					assert(frames[i].offset + frames[i].length <= 90);
					assert(frames[i].length >= 14);
				}
		}

	ncm_encode(block, frame, 60, 0);
	assert(ncm_decode(block, NCM_NTB_SIZE + 1, frames) == -1);
	block[0] ^= 1;
	assert(ncm_decode(block, 90, frames) == -1);
	ncm_encode(block, frame, 60, 0);
	ncm_put16(block + 18, 12); /* cycle */
	assert(ncm_decode(block, 90, frames) == -1);
	ncm_encode(block, frame, 60, 0);
	ncm_put16(block + 20, 14); /* frame overlapping NDP */
	assert(ncm_decode(block, 90, frames) == -1);
}

static void multiple_frames(void)
{
	memset(block, 0, sizeof(block));
	ncm_put32(block, 0x484d434e);
	ncm_put16(block + 4, 12);
	ncm_put16(block + 8, 2048);
	ncm_put16(block + 10, 12);
	ncm_put32(block + 12, 0x304d434e);
	ncm_put16(block + 16, 8 + (NCM_MAX_DATAGRAMS + 1) * 4);

	for (unsigned i = 0; i < NCM_MAX_DATAGRAMS; i++) {
		ncm_put16(block + 20 + i * 4, 154 + i * 16);
		ncm_put16(block + 22 + i * 4, 14);
	}

	assert(ncm_decode(block, 2048, frames) == NCM_MAX_DATAGRAMS);
	ncm_put16(block + 24, 154); /* duplicate/overlapping frame */
	assert(ncm_decode(block, 2048, frames) == -1);

	/* Two linked NDPs, plus legal transfer padding after wBlockLength. */
	ncm_encode(block, frame, 14, 0);
	ncm_put16(block + 8, 76);
	ncm_put16(block + 18, 44);
	memset(block + 44, 0, 16);
	ncm_put32(block + 44, 0x304d434e);
	ncm_put16(block + 48, 16);
	ncm_put16(block + 52, 62);
	ncm_put16(block + 54, 14);
	assert(ncm_decode(block, 80, frames) == 2);
	ncm_put16(block + 50, 12);
	assert(ncm_decode(block, 80, frames) == -1);
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
	struct ncm_frame_s output[NCM_MAX_DATAGRAMS];
	int n = ncm_decode(data, size, output);
	assert(n >= -1 && n <= NCM_MAX_DATAGRAMS);

	for (int i = 0; i < n; i++) {
		assert(output[i].length >= 14 && output[i].length <= NCM_FRAME_SIZE);
		assert(output[i].offset + output[i].length <= size);
	}

	return 0;
}

#ifndef NCM_LIBFUZZER
int main(void)
{
	single_frame();
	malformed();
	multiple_frames();
	unsigned state = 1;

	for (unsigned i = 0; i < 100000; i++) {
		for (unsigned j = 0; j < sizeof(block); j++) {
			state = state * 1664525 + 1013904223;
			block[j] = state >> 24;
		}

		LLVMFuzzerTestOneInput(block, i % (NCM_NTB_SIZE + 1));
	}

	puts("NTB16: boundaries, truncation, chains, overlaps and randomized inputs passed");
}
#endif
