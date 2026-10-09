#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Exercise production stream methods with controlled concurrent deletion.

Extract the methods verbatim to avoid linking the flight stack. Only the stream
and receiver dependencies are mocked; List, LockGuard and the method bodies are
the production code. --source-root can select an older checkout for reproduction.
"""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

here = Path(__file__).resolve().parent
repo = here.parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source-root', type=Path, default=repo)
args = parser.parse_args()


def method(path, signature):
    source = path.read_text()
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


module = args.source_root / 'src/modules/mavlink'
bodies = [
    'int ' + method(module / 'mavlink_main.cpp', 'Mavlink::configure_stream('),
    'void ' + method(module / 'mavlink_main.cpp', 'Mavlink::display_status_streams()'),
    method(module / 'mavlink_receiver.cpp',
           'uint8_t MavlinkReceiver::handle_request_message_command('),
    'void ' + method(module / 'mavlink_receiver.cpp', 'MavlinkReceiver::get_message_interval('),
]

with tempfile.TemporaryDirectory(prefix='px4-stream-lifetime-') as temp:
    temp = Path(temp)
    (temp / 'stream_methods.inc').write_text('\n\n'.join(bodies))
    exe = temp / 'stream_lifetime'
    subprocess.run([
        os.environ.get('CXX', 'clang++'), '-std=c++17', '-g', '-pthread',
        '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
        '-fsanitize=address,undefined', f'-I{repo / "src/include"}', f'-I{temp}',
        str(here / 'stream_lifetime.cpp'), '-o', str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True, timeout=20)
