"""Trajectory figure of the risk scenarios: the screenshots of both maps side by side (one column of the paper), with
one shared legend on top, the start and goal labels and arrows pointing at the extreme scenarios.

Inputs, per map (baselines/<map>/risk_scenarios/):
    screenshot.png      view of the trajectories published by publish_risk_scenarios.py (RViz/Foxglove)
    annotations.json    name of the environment (written at the bottom of the screenshot) and positions in the pixels
                        of the screenshot (x right, y down):
        {"name": "Cluttered environment",
         "start": {"xy": [x, y], "ha": "right", "va": "top"},           text placed at xy
         "goal":  {"xy": [x, y], "text": "Goal", "arrow_to": [x, y]},   optional text and arrow
         "labels": [{"scenario": "high_winds", "xy": [x, y], "arrow_to": [x, y], "text": "High\\nwinds"}, ...]}
    The label of a scenario is its legend label unless "text" is given; "ha"/"va" align the text on xy (default center).

Usage:
    python3 make_risk_scenarios_figure.py [--maps CL_map_3 seed_42] [--width 3.5] [--output risk_scenarios_trajectories]
"""

import argparse
import json
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.patheffects as path_effects  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
from PIL import Image  # noqa: E402

HERE = os.path.dirname(os.path.realpath(__file__))
# Same order, labels and colors as publish_risk_scenarios.py (the colors of the trajectories in the screenshots)
SCENARIOS = [('high_winds', 'High winds', (0.0, 0.0, 1.0)),
             ('low_gps', 'Low GPS signal', (0.0, 1.0, 0.0)),
             ('low_battery', 'Low battery', (1.0, 0.0, 0.0)),
             ('battery_winds', 'Low battery + High winds', (1.0, 0.0, 1.0)),
             ('battery_gps', 'Low battery + Low GPS signal', (1.0, 1.0, 0.0)),
             ('winds_gps', 'High winds + Low GPS signal', (0.0, 1.0, 1.0))]
STYLE = {'font.family': 'sans-serif',
         'font.sans-serif': ['Liberation Sans', 'Arial', 'Helvetica', 'DejaVu Sans'],
         'pdf.fonttype': 42,  # Embedded TrueType fonts, required by IEEE PDF eXpress
         'ps.fonttype': 42}
LABEL_SIZE = 7.0  # pt, bold labels of the extremes, start and goal
LEGEND_SIZE = 6.0
NAME_SIZE = 7.0  # pt, name of the environment at the bottom of each screenshot
LEGEND_HEIGHT = 0.36  # in, strip of the legend on top
PANEL_GAP = 0.04  # in, between the screenshots
ARROW_WIDTH = 0.9
# White outline of the labels: readable over the grey voxels and the trajectories
HALO = [path_effects.withStroke(linewidth=1.6, foreground='white')]


def annotate(ax, spec, text, color):
    arrow = spec.get('arrow_to')
    ax.annotate(text, xy=arrow if arrow else spec['xy'], xytext=spec['xy'], textcoords='data',
                ha=spec.get('ha', 'center'), va=spec.get('va', 'center'), fontsize=LABEL_SIZE, fontweight='bold',
                color=color, linespacing=0.95, path_effects=HALO, annotation_clip=False,
                arrowprops=dict(arrowstyle='-|>', color=color, lw=ARROW_WIDTH, mutation_scale=6, shrinkA=1, shrinkB=0,
                                path_effects=HALO) if arrow else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--maps', nargs='+', default=['CL_map_3', 'seed_42'], help='Panels, from left to right')
    parser.add_argument('--width', type=float, default=3.5, help='Width of the figure (in), one column of RA-L')
    parser.add_argument('--output', default=os.path.join(HERE, 'risk_scenarios_trajectories'),
                        help='Output path without extension (.pdf and .png)')
    args = parser.parse_args()

    panels = []
    for name in args.maps:
        folder = os.path.join(HERE, name, 'risk_scenarios')
        with open(os.path.join(folder, 'annotations.json')) as f:
            panels.append((Image.open(os.path.join(folder, 'screenshot.png')).convert('RGB'), json.load(f)))

    # Same height for every screenshot, the widths fill the figure
    aspects = [image.width / image.height for image, _ in panels]
    height = (args.width - PANEL_GAP * (len(panels) - 1)) / sum(aspects)
    fig_height = height + LEGEND_HEIGHT
    plt.rcParams.update(STYLE)
    fig = plt.figure(figsize=(args.width, fig_height))
    left = 0.0
    colors = {folder: rgb for folder, _, rgb in SCENARIOS}
    labels = {folder: label for folder, label, _ in SCENARIOS}
    for (image, notes), aspect in zip(panels, aspects):
        width = height * aspect
        ax = fig.add_axes([left / args.width, 0.0, width / args.width, height / fig_height])
        ax.imshow(image, interpolation='lanczos')
        ax.set_axis_off()
        for key in ('start', 'goal'):
            spec = notes[key]
            annotate(ax, spec, spec.get('text', key.capitalize()), 'black')
        for spec in notes['labels']:
            annotate(ax, spec, spec.get('text', labels[spec['scenario']]), colors[spec['scenario']])
        ax.text(0.5, 0.02, notes['name'], transform=ax.transAxes, ha='center', va='bottom', fontsize=NAME_SIZE,
                bbox=dict(boxstyle='square,pad=0.25', facecolor='white', edgecolor='black', linewidth=0.6))
        left += width + PANEL_GAP

    handles = [Patch(facecolor=rgb, edgecolor='none', label=label) for _, label, rgb in SCENARIOS]
    fig.legend(handles=handles, title='Risk scenarios', loc='upper center', bbox_to_anchor=(0.5, 1.0), ncol=3,
               fontsize=LEGEND_SIZE, title_fontsize=LEGEND_SIZE, frameon=True, fancybox=False, edgecolor='black',
               borderpad=0.3, handlelength=1.6, handleheight=0.6, handletextpad=0.4, columnspacing=0.8,
               labelspacing=0.25, borderaxespad=0.0).get_frame().set_linewidth(0.6)

    for extension in ('pdf', 'png'):
        fig.savefig('{}.{}'.format(args.output, extension), dpi=600)
    print('{}.pdf/.png: {:.2f} x {:.2f} in'.format(args.output, args.width, fig_height))


if __name__ == '__main__':
    main()
