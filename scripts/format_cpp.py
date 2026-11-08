#!/usr/bin/env python3
"""Format project C++/CUDA sources with clang-format 14 (Google style)."""
import argparse
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
# Keep bundled libraries and NVIDIA helpers in their upstream style.
EXCLUDED = ('/third_party/', '/include/ode/', '/lbfgs.hpp', '/helper_math.h')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    files = sorted(p for p in (ROOT / 'src').rglob('*')
                   if p.suffix in ('.h', '.hpp', '.c', '.cpp', '.cu', '.cuh')
                   and not any(s in str(p) for s in EXCLUDED))
    flags = ['--dry-run', '--Werror'] if args.check else ['-i']
    subprocess.run(['clang-format', '--style=file', *flags,
                    *(str(p) for p in files)], check=True)
    print(f'{"Checked" if args.check else "Formatted"} {len(files)} C++/CUDA files')


if __name__ == '__main__':
    main()
