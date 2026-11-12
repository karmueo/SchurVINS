#!/usr/bin/env python3
"""Pair image indexes by exactly equal timestamps; preserve image paths and order."""
import argparse
from decimal import Decimal
from pathlib import Path


def combine(left, right, output):
    def rows(path):
        return [line.split() for line in Path(path).read_text().splitlines()
                if line.strip() and not line.lstrip().startswith('#')]
    left_rows, right_rows = rows(left), rows(right)
    by_stamp = {}
    for row in right_rows:
        stamp = Decimal(row[1])
        if stamp in by_stamp:
            raise ValueError(f'Duplicate right timestamp: {stamp}')
        by_stamp[stamp] = row[2]
    count = 0
    with Path(output).open('w') as stream:
        stream.write('# id timestamp image_0 image_1\n')
        for row in left_rows:
            right_image = by_stamp.pop(Decimal(row[1]), None)
            if right_image is not None:
                stream.write(f'{count} {row[1]} {row[2]} {right_image}\n')
                count += 1
    if count != len(left_rows) or count != len(right_rows):
        raise ValueError(f'Unpaired images: left={len(left_rows)}, right={len(right_rows)}, paired={count}')
    return count


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left', default='images_0.txt')
    parser.add_argument('--right', default='images_1.txt')
    parser.add_argument('--output', default='images_combined.txt')
    args = parser.parse_args()
    print(f'Paired {combine(args.left, args.right, args.output)} stereo images')
