#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
import base64
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib
from check_size import APPLICATION_LIMIT, FLASH_ORIGIN, check


class SizeTests(unittest.TestCase):
    @staticmethod
    def fixture(size, origin=FLASH_ORIGIN):
        image = bytes(size)
        elf = bytearray(84 + size)
        elf[:7] = b'\x7fELF\x01\x01\x01'
        struct.pack_into('<I', elf, 28, 52)
        struct.pack_into('<HH', elf, 42, 32, 1)
        struct.pack_into('<8I', elf, 52, 1, 84, origin, origin, size, size, 5, 4)
        package = json.dumps({'image': base64.b64encode(zlib.compress(image)).decode(),
                              'image_size': size, 'image_maxsize': APPLICATION_LIMIT})
        return elf, image, package

    def test_exact_limit(self):
        self.assertEqual(check(*self.fixture(APPLICATION_LIMIT))['spare_bytes'], 0)

    def test_compressed_oversize_rejected(self):
        fixture = self.fixture(APPLICATION_LIMIT + 1)
        self.assertLess(len(fixture[2]), 4000)
        with self.assertRaisesRegex(ValueError, 'exceeds flash'):
            check(*fixture)

    def test_package_mismatch(self):
        elf, image, package = self.fixture(100)
        with self.assertRaisesRegex(ValueError, 'do not match'):
            check(elf, image + b'x', package)

    def test_bootloader_overlap(self):
        with self.assertRaisesRegex(ValueError, 'bootloader'):
            check(*self.fixture(100, FLASH_ORIGIN - 1))

    def test_packager_enforces_prototype_limit(self):
        tool = Path(__file__).resolve().parents[1] / 'px_mkfw.py'
        with tempfile.TemporaryDirectory() as tmp:
            proto, binary = Path(tmp) / 'proto', Path(tmp) / 'fw.bin'
            proto.write_text(json.dumps({'image_maxsize': 128}))
            for size, success in ((128, True), (129, False)):
                binary.write_bytes(bytes(size))
                result = subprocess.run([sys.executable, str(tool), '--prototype', str(proto),
                                         '--image', str(binary)], capture_output=True)
                self.assertEqual(result.returncode == 0, success)
                if not success:
                    self.assertIn(b'exceeds application flash', result.stderr)


if __name__ == '__main__':
    unittest.main()
