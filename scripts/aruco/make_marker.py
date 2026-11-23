#!/usr/bin/env python3
"""Write a printable ArUco tag at the servo size (6.5 in outer black border, 300 dpi).

Usage:
  python3 scripts/aruco/make_marker.py                    # ID 6, DICT_7X7_50 -> tag6_7x7_6p5in.png
  python3 scripts/aruco/make_marker.py --dict DICT_7X7_250 --id 6 --out my.png

Print at 100% (actual size, no "fit to page"), then measure the black square with a ruler:
it should read 6.5 in (165 mm). The servo treats that outer edge as the marker size.
"""
import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'autonomy'))
import tag  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--id', type=int, default=6)
    ap.add_argument('--dict', default='DICT_7X7_50', choices=tag.DICT_NAMES)  # 7x7 printable tag
    ap.add_argument('--dpi', type=int, default=300)
    ap.add_argument('--out', default=os.path.join(HERE, 'tag6_7x7_6p5in.png'))
    args = ap.parse_args()
    img = tag.render_marker(args.id, args.dict, args.dpi)
    if not cv2.imwrite(args.out, img):
        sys.exit(f'could not write {args.out}')
    side_mm = tag.TAG_SIZE_IN * 25.4
    print(f'wrote {args.out}: ID {args.id} {args.dict}, black square {tag.TAG_SIZE_IN} in ({side_mm:.1f} mm), '
          f'{img.shape[1]}x{img.shape[0]} px at {args.dpi} dpi')


if __name__ == '__main__':
    main()
