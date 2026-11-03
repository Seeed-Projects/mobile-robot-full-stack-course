"""Standalone plot of a recorded AlgorithmView snapshot; no ROS runtime required."""
import argparse
import colorsys
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np


def color(index):
    return colorsys.hsv_to_rgb((.06+index*.61803398875) % 1., .8, 1.)


def render(report, output):
    grid = report['map']
    pixels = np.array(grid['data']).reshape(grid['height'], grid['width'])
    pixels = np.where(pixels < 0, 205, 254*(1-pixels/100)).astype(np.uint8)
    x, y, heading = grid['origin']
    resolution = grid['resolution']
    reference, trajectory = np.array(report['reference_path']), np.array(report['estimated_trajectory'])
    snapshot = report['detection_snapshot']
    selected = next(b for b in snapshot['boxes'] if b['id'] == snapshot['selected_cluster'])
    center = np.array(selected['center'])
    points = np.array(snapshot['obstacle_points']).reshape(-1, 3)
    robot = np.array(snapshot['robot_pose'])
    fig = plt.figure(figsize=(17, 6.8))
    axes = [fig.add_subplot(1, 3, 1), fig.add_subplot(1, 3, 2)]
    from matplotlib.transforms import Affine2D
    for ax in axes:
        image = ax.imshow(pixels, origin='lower', cmap='gray', vmin=0, vmax=255,
            extent=[0, grid['width']*resolution, 0, grid['height']*resolution],
            transform=Affine2D().rotate(heading).translate(x, y)+ax.transData)
        ax.plot(reference[:, 0], reference[:, 1], '--', color='#299342', linewidth=2, label='Initial global reference')
        ax.plot(trajectory[:, 0], trajectory[:, 1], color='#2478c8', linewidth=2, label='Localized trajectory (full run)')
        if len(points):
            ax.scatter(points[:, 0], points[:, 1], s=2, color='#ef8d35', alpha=.6, label='Detected obstacle points')
        ax.scatter(*reference[0], c='green', s=35, zorder=7, label='Start')
        ax.scatter(*reference[-1], c='red', marker='x', s=65, zorder=7, label='Goal')
        ax.scatter(*robot[:2], c='#14558a', marker='^', s=70, zorder=8, label='Robot at detection snapshot')
        for box in snapshot['boxes']:
            if not box['compact']:
                continue
            corners = np.array(box['corners'])
            polygon = corners[[0, 4, 6, 2], :2]
            ax.add_patch(Polygon(polygon, fill=False, color=color(box['id']), linewidth=2, zorder=8))
            ax.text(box['center'][0]+.05, box['center'][1]+.10, f"C{box['id']}",
                fontsize=8, color=color(box['id']), weight='bold', zorder=9, clip_on=True)
        ax.set(xlabel='SLAM map X (m)', ylabel='SLAM map Y (m)', aspect='equal')
    axes[0].set(xlim=(x-.25, x+grid['width']*resolution+.25),
                ylim=(y-.25, y+grid['height']*resolution+.25),
                title=f"{report.get('planner', 'A*')} + {report.get('controller', 'MPPI')}: route and LiDAR detections")
    axes[0].legend(fontsize=7, loc='lower left', framealpha=.95)
    axes[1].set(xlim=(center[0]-1.6, center[0]+1.6), ylim=(center[1]-1.7, center[1]+1.7),
                title='Local detour and a detected cluster')
    # An explicit orthographic 3D projection also works on hosts with mixed
    # Matplotlib/mpl_toolkits installations, without the optional Axes3D plugin.
    ax = fig.add_subplot(1, 3, 3)
    azimuth, elevation = np.radians(-45), np.radians(25)
    def project(cloud):
        relative = np.asarray(cloud)-center
        horizontal = np.cos(azimuth)*relative[:, 0]-np.sin(azimuth)*relative[:, 1]
        depth = np.sin(azimuth)*relative[:, 0]+np.cos(azimuth)*relative[:, 1]
        vertical = np.cos(elevation)*relative[:, 2]-np.sin(elevation)*depth
        return np.column_stack([horizontal, vertical])
    crop = points[np.linalg.norm(points[:, :2]-center[:2], axis=1) < .5] if len(points) else points
    if len(crop):
        projected = project(crop)
        ax.scatter(projected[:, 0], projected[:, 1], c=crop[:, 2], cmap='autumn', s=6, alpha=.7)
    corners = np.array(selected['corners'])
    for a in range(8):
        for b in range(a+1, 8):
            if bin(a ^ b).count('1') == 1:
                edge = corners[[a, b]]
                projected = project(edge)
                ax.plot(projected[:, 0], projected[:, 1], color=color(selected['id']), linewidth=2)
    ax.text(.03, .97, f"C{selected['id']} | {selected['points']} points\n"
        f"Box: {selected['size'][0]:.2f} x {selected['size'][1]:.2f} x {selected['size'][2]:.2f} m",
        transform=ax.transAxes, va='top', fontsize=9)
    ax.set(xlim=(-.7, .7), ylim=(-.6, .75), aspect='equal', xlabel='Projected horizontal distance (m)',
           ylabel='Projected vertical distance (m)', title='LiDAR points / cluster box (3D projection)')
    ax.grid(alpha=.15)
    fig.suptitle(f"Detection frame at simulation t={snapshot['stamp_sec']:.1f} s | "
                 f"Ground fit={snapshot['ground_fitted']} | Processing {snapshot['processing_ms']:.0f} ms", fontsize=12)
    fig.text(.5, .02, 'Blue: localization estimate, not ground truth. Points/boxes: one recorded frame; cluster IDs are not tracking IDs.',
             ha='center', fontsize=9, color='#555555')
    fig.tight_layout(rect=(0, .07, 1, .94))
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    render(json.loads(args.record.read_text()), args.output or args.record.with_suffix('.png'))


if __name__ == '__main__':
    main()
