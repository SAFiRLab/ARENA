#!/usr/bin/env python3
"""Labels on a screenshot of the trajectories (style of the real flight figure): rounded boxes with a bold white text,
optionally an arrow from the box to a point of the image (e.g. the energy and safety paths).

Spec (JSON, pixels of the screenshot, x right, y down):
    {"font_size": 48,
     "labels": [{"text": "Start", "center": [x, y], "color": "dark"},
                {"text": "Safety", "center": [x, y], "color": "safety", "arrow_from": [x, y], "arrow_to": [x, y]}]}
    color: dark (start, goal), energy (magenta), safety (green), or [r, g, b, a]. arrow_from defaults to the box center
    (the arrow starts at the box border).

Usage: annotate_figure.py <screenshot.png> <spec.json> <output.png> [--font <ttf>]
    Default font: Poppins Bold if found (fonts/Poppins-Bold.ttf next to this script), else Liberation Sans Bold.
"""

import argparse
import json
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.realpath(__file__))
FONTS = [os.path.join(HERE, 'fonts', 'Poppins-Bold.ttf'), '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf']
# Box fill and arrow colors (RGBA), sampled from the original figure: the boxes let the voxels show through
COLORS = {'dark': ((58, 58, 58, 215), None),
          'energy': ((198, 88, 198, 225), (214, 86, 213, 255)),
          'safety': ((108, 230, 98, 240), (120, 236, 110, 255))}
SUPERSAMPLING = 4


def rounded_box(draw, center, size, radius, fill):
    x, y = center
    w, h = size
    draw.rounded_rectangle((x - w / 2, y - h / 2, x + w / 2, y + h / 2), radius=radius, fill=fill)


def border_point(center, size, toward):
    """Point of the border of the box (center, size) on the segment from its center toward a point."""
    d = np.asarray(toward, float) - np.asarray(center, float)
    scale = min((size[0] / 2) / abs(d[0]) if d[0] else np.inf, (size[1] / 2) / abs(d[1]) if d[1] else np.inf)
    return np.asarray(center, float) + d * min(scale, 1.0)


def arrow(draw, start, end, width, head, fill):
    start, end = np.asarray(start, float), np.asarray(end, float)
    direction = (end - start) / np.linalg.norm(end - start)
    normal = np.array([-direction[1], direction[0]])
    base = end - direction * head
    draw.line([tuple(start), tuple(base + direction * 1.0)], fill=fill, width=int(round(width)))
    draw.polygon([tuple(end), tuple(base + normal * head * 0.55), tuple(base - normal * head * 0.55)], fill=fill)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('screenshot')
    parser.add_argument('spec')
    parser.add_argument('output')
    parser.add_argument('--font')
    args = parser.parse_args()
    with open(args.spec) as f:
        spec = json.load(f)
    font_file = args.font or next(f for f in FONTS if os.path.exists(f))

    image = Image.open(args.screenshot).convert('RGBA')
    k = SUPERSAMPLING
    overlay = Image.new('RGBA', (image.size[0] * k, image.size[1] * k), (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    font = ImageFont.truetype(font_file, int(spec.get('font_size', 48) * k))
    pad_x, pad_y = spec.get('padding', [26, 12])

    for label in spec['labels']:
        fill, arrow_fill = (COLORS[label['color']] if isinstance(label['color'], str)
                            else (tuple(label['color']), tuple(label['color'][:3]) + (255,)))
        left, _, right, _ = draw.textbbox((0, 0), label['text'], font=font, anchor='mm')
        _, top, _, bottom = draw.textbbox((0, 0), 'Sgy', font=font, anchor='mm')  # Same height for every label
        size = ((right - left) / k + 2 * pad_x, (bottom - top) / k + 2 * pad_y)
        center = np.asarray(label['center'], float)
        if 'arrow_to' in label:
            start = border_point(center, size, label.get('arrow_from', label['arrow_to']))
            if 'arrow_from' in label:
                start = np.asarray(label['arrow_from'], float)
            arrow(draw, start * k, np.asarray(label['arrow_to'], float) * k, spec.get('arrow_width', 7) * k,
                  spec.get('arrow_head', 26) * k, arrow_fill)
        rounded_box(draw, center * k, (size[0] * k, size[1] * k), spec.get('radius', 12) * k, fill)
        draw.text(tuple(center * k), label['text'], font=font, fill=(255, 255, 255, 255), anchor='mm')

    overlay = overlay.resize(image.size, Image.LANCZOS)
    Image.alpha_composite(image, overlay).convert('RGB').save(args.output)
    print(args.output)


if __name__ == '__main__':
    main()
