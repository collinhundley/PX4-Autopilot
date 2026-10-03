#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Check the Pixhawk 6C application ELF, raw binary and unpacked PX4 image."""
import argparse
import base64
import json
from pathlib import Path
import struct
import zlib

FLASH_ORIGIN = 0x08020000
APPLICATION_LIMIT = 1966080
DESIRED_RESERVE = 16384


def elf_usage(data):
    if data[:7] != b'\x7fELF\x01\x01\x01':
        raise ValueError('expected a little-endian ELF32 image')
    phoff = struct.unpack_from('<I', data, 28)[0]
    phsize, phnum = struct.unpack_from('<HH', data, 42)
    if phsize != 32:
        raise ValueError('unexpected ELF program header size')
    flash_end = FLASH_ORIGIN
    ram = 0
    for i in range(phnum):
        kind, offset, virtual, physical, filesz, memsz, flags, align = struct.unpack_from(
            '<8I', data, phoff + i * phsize)
        if kind != 1:
            continue
        if filesz and 0x08000000 <= physical < 0x10000000:
            if physical < FLASH_ORIGIN:
                raise ValueError('application segment overlaps reserved bootloader flash')
            if offset + filesz > len(data):
                raise ValueError('truncated ELF load segment')
            flash_end = max(flash_end, physical + filesz)
        if 0x20000000 <= virtual < 0x40000000:
            ram += memsz
    if flash_end == FLASH_ORIGIN:
        raise ValueError('no flash load segments found')
    return flash_end - FLASH_ORIGIN, ram


def check(elf, binary, package):
    flash, ram = elf_usage(elf)
    desc = json.loads(package)
    image = zlib.decompress(base64.b64decode(desc['image'], validate=True))
    if image != binary or desc['image_size'] != len(image):
        raise ValueError('PX4 image and raw binary do not match')
    if desc['image_maxsize'] != APPLICATION_LIMIT:
        raise ValueError('package advertises the wrong application flash limit')
    used = max(flash, len(binary), len(image))
    if used > APPLICATION_LIMIT:
        raise ValueError(f'application exceeds flash: {used} > {APPLICATION_LIMIT}')
    return {'elf_flash_bytes': flash, 'binary_bytes': len(binary),
            'px4_image_bytes': len(image), 'static_ram_bytes': ram,
            'limit_bytes': APPLICATION_LIMIT, 'spare_bytes': APPLICATION_LIMIT - used,
            'reserve_16k_met': APPLICATION_LIMIT - used >= DESIRED_RESERVE}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('build', type=Path, help='build/px4_fmu-v6c_osd directory')
    args = parser.parse_args()
    stem = args.build / args.build.name
    try:
        result = check(stem.with_suffix('.elf').read_bytes(),
                       stem.with_suffix('.bin').read_bytes(),
                       stem.with_suffix('.px4').read_text())
    except (ValueError, KeyError, OSError, struct.error, zlib.error) as exc:
        parser.exit(1, f'Firmware size check failed: {exc}\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
