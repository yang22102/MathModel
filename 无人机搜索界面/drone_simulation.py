"""
无人机区域划分与搜索路径可视化脚本
Drone Area Division & Search Path Visualization

依赖: numpy, matplotlib, scipy
安装: pip install numpy matplotlib scipy
用法: python drone_simulation.py
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import ListedColormap
from scipy.spatial import Voronoi, voronoi_plot_2d
import random
import argparse
import os

# ─────────────────────────────────────────────────────────
#  Configuration
# ─────────────────────────────────────────────────────────
DRONE_COLORS = [
    '#e94560', '#63b3ed', '#68d391', '#f6ad55',
    '#b794f4', '#f687b3', '#4fd1c5', '#fbd38d'
]

# ─────────────────────────────────────────────────────────
#  Area Partition Algorithms
# ─────────────────────────────────────────────────────────

def partition_grid(W, H, n_drones):
    """均匀网格划分: divide area into n_drones rectangular blocks."""
    zone = np.full((H, W), -1, dtype=int)
    cols = int(np.ceil(np.sqrt(n_drones)))
    rows = int(np.ceil(n_drones / cols))
    bw = int(np.ceil(W / cols))
    bh = int(np.ceil(H / rows))
    for y in range(H):
        for x in range(W):
            c = x // bw
            r = y // bh
            idx = r * cols + c
            zone[y, x] = min(idx, n_drones - 1)
    return zone


def partition_strip(W, H, n_drones):
    """条带划分: vertical strips."""
    zone = np.full((H, W), -1, dtype=int)
    bw = int(np.ceil(W / n_drones))
    for y in range(H):
        for x in range(W):
            zone[y, x] = min(x // bw, n_drones - 1)
    return zone


def partition_quadrant(W, H, n_drones):
    """象限/扇形划分: angular sectors from center."""
    zone = np.full((H, W), -1, dtype=int)
    cx, cy = W / 2, H / 2
    for y in range(H):
        for x in range(W):
            angle = np.arctan2(y - cy, x - cx)
            if angle < 0:
                angle += 2 * np.pi
            idx = int(angle / (2 * np.pi / n_drones))
            zone[y, x] = min(idx, n_drones - 1)
    return zone


def partition_voronoi(W, H, n_drones, seed=42):
    """Voronoi划分: nearest-seed assignment with stratified random seeds."""
    rng = random.Random(seed)
    cols = int(np.ceil(np.sqrt(n_drones)))
    rows = int(np.ceil(n_drones / cols))
    seeds = []
    for i in range(n_drones):
        c, r = i % cols, i // cols
        cell_w = W / cols
        cell_h = H / rows
        jitter_x = rng.uniform(-cell_w * 0.3, cell_w * 0.3)
        jitter_y = rng.uniform(-cell_h * 0.3, cell_h * 0.3)
        sx = (c + 0.5) * cell_w + jitter_x
        sy = (r + 0.5) * cell_h + jitter_y
        seeds.append((max(0, min(W-1, sx)), max(0, min(H-1, sy))))

    zone = np.full((H, W), -1, dtype=int)
    seed_arr = np.array(seeds)
    for y in range(H):
        for x in range(W):
            dists = (seed_arr[:, 0] - x)**2 + (seed_arr[:, 1] - y)**2
            zone[y, x] = int(np.argmin(dists))
    return zone, seeds


# ─────────────────────────────────────────────────────────
#  Search Path Algorithms
# ─────────────────────────────────────────────────────────

def path_zigzag(zone_cells, zone_bbox):
    """S形覆盖搜索: row-by-row zigzag."""
    minX, maxX, minY, maxY = zone_bbox
    cell_set = set(map(tuple, zone_cells))
    path = []
    for y in range(minY, maxY + 1):
        row = [(x, y) for x in range(minX, maxX + 1) if (x, y) in cell_set]
        if y % 2 == 1:
            row = row[::-1]
        path.extend(row)
    return path


def path_boustrophedon(zone_cells, zone_bbox):
    """往复扫描搜索: column-by-column."""
    minX, maxX, minY, maxY = zone_bbox
    cell_set = set(map(tuple, zone_cells))
    path = []
    for x in range(minX, maxX + 1):
        col = [(x, y) for y in range(minY, maxY + 1) if (x, y) in cell_set]
        if x % 2 == 1:
            col = col[::-1]
        path.extend(col)
    return path


def path_spiral(zone_cells):
    """螺旋式搜索: sorted by distance from centroid (center-out)."""
    if not zone_cells:
        return []
    arr = np.array(zone_cells)
    cx, cy = arr[:, 0].mean(), arr[:, 1].mean()
    dists = (arr[:, 0] - cx)**2 + (arr[:, 1] - cy)**2
    order = np.argsort(dists)
    return [tuple(zone_cells[i]) for i in order]


def path_random(zone_cells, seed=None):
    """随机游走搜索: shuffled zone cells."""
    cells = list(map(tuple, zone_cells))
    rng = random.Random(seed)
    rng.shuffle(cells)
    return cells


def get_zone_cells(zone_map, drone_idx, obstacles):
    """Return list of (x,y) cells belonging to drone_idx, excluding obstacles."""
    H, W = zone_map.shape
    cells = []
    for y in range(H):
        for x in range(W):
            if zone_map[y, x] == drone_idx and (x, y) not in obstacles:
                cells.append((x, y))
    return cells


def zone_bbox(cells):
    if not cells:
        return 0, 0, 0, 0
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    return min(xs), max(xs), min(ys), max(ys)


# ─────────────────────────────────────────────────────────
#  Visualization
# ─────────────────────────────────────────────────────────

def visualize(W=30, H=30, n_drones=3,
              partition='grid',
              search='zigzag',
              n_obstacles=20,
              output_dir='output',
              show=True,
              seed=0):
    """Generate and save visualizations."""
    os.makedirs(output_dir, exist_ok=True)
    rng = random.Random(seed)

    # --- Generate obstacles ---
    obstacles = set()
    while len(obstacles) < n_obstacles:
        x = rng.randint(0, W - 1)
        y = rng.randint(0, H - 1)
        obstacles.add((x, y))

    # --- Partition ---
    voronoi_seeds = None
    if partition == 'grid':
        zone_map = partition_grid(W, H, n_drones)
    elif partition == 'strip':
        zone_map = partition_strip(W, H, n_drones)
    elif partition == 'quadrant':
        zone_map = partition_quadrant(W, H, n_drones)
    elif partition == 'voronoi':
        zone_map, voronoi_seeds = partition_voronoi(W, H, n_drones)
    else:
        raise ValueError(f"Unknown partition algorithm: {partition}")

    # --- Build drone zone cells & paths ---
    all_paths = []
    drone_starts = []
    for i in range(n_drones):
        cells = get_zone_cells(zone_map, i, obstacles)
        bbox = zone_bbox(cells)
        if search == 'zigzag':
            path = path_zigzag(cells, bbox)
        elif search == 'boustrophedon':
            path = path_boustrophedon(cells, bbox)
        elif search == 'spiral':
            path = path_spiral(cells)
        elif search == 'random':
            path = path_random(cells, seed=i)
        else:
            path = path_zigzag(cells, bbox)
        all_paths.append(path)
        drone_starts.append(path[0] if path else (0, 0))

    # ─── Figure 1: Area Partition ───────────────────────────
    fig1, ax1 = plt.subplots(figsize=(8, 8))
    fig1.patch.set_facecolor('#0d1117')
    ax1.set_facecolor('#111827')

    # Zone coloring (RGBA array)
    zone_rgba = np.zeros((H, W, 4))
    for y in range(H):
        for x in range(W):
            di = zone_map[y, x]
            if di >= 0:
                c = plt.cm.colors.to_rgba(DRONE_COLORS[di], alpha=0.45)
                zone_rgba[y, x] = c

    ax1.imshow(zone_rgba, origin='upper', extent=[0, W, H, 0], aspect='auto', interpolation='nearest')

    # Grid lines
    for x in range(W + 1):
        ax1.axvline(x, color='#374151', linewidth=0.3, alpha=0.5)
    for y in range(H + 1):
        ax1.axhline(y, color='#374151', linewidth=0.3, alpha=0.5)

    # Voronoi seeds
    if voronoi_seeds:
        for i, (sx, sy) in enumerate(voronoi_seeds):
            ax1.plot(sx + 0.5, sy + 0.5, '*', color=DRONE_COLORS[i], markersize=12,
                     markeredgecolor='white', markeredgewidth=0.8, zorder=5)

    # Obstacles
    for (ox, oy) in obstacles:
        rect = mpatches.FancyBboxPatch((ox + 0.05, oy + 0.05), 0.9, 0.9,
                                       boxstyle='round,pad=0.05',
                                       facecolor='#374151', edgecolor='#6b7280',
                                       linewidth=0.8)
        ax1.add_patch(rect)

    # Drone markers
    for i, (sx, sy) in enumerate(drone_starts):
        ax1.plot(sx + 0.5, sy + 0.5, 'o', color=DRONE_COLORS[i],
                 markersize=14, markeredgecolor='white', markeredgewidth=1.5, zorder=6)
        ax1.text(sx + 0.5, sy + 0.5, str(i + 1),
                 color='white', fontsize=8, ha='center', va='center',
                 fontweight='bold', zorder=7)

    # Legend
    legend_handles = [
        mpatches.Patch(facecolor=DRONE_COLORS[i], edgecolor='white',
                       linewidth=0.5, label=f'Drone {i+1}')
        for i in range(n_drones)
    ]
    legend_handles.append(mpatches.Patch(facecolor='#374151', edgecolor='#6b7280',
                                         linewidth=0.5, label='Obstacle'))
    ax1.legend(handles=legend_handles, loc='upper right',
               facecolor='#16213e', edgecolor='#e94560',
               labelcolor='white', fontsize=8)

    ALGO_NAMES = {
        'grid': 'Uniform Grid', 'strip': 'Strip',
        'quadrant': 'Quadrant/Sector', 'voronoi': 'Voronoi'
    }
    ax1.set_title(f'Drone Area Division — {ALGO_NAMES.get(partition, partition)}\n'
                  f'{n_drones} Drones | {W}x{H} Grid | {n_obstacles} Obstacles',
                  color='#e94560', fontsize=11, pad=10)
    ax1.set_xlim(0, W); ax1.set_ylim(H, 0)
    ax1.tick_params(colors='#a0aec0', labelsize=7)
    for spine in ax1.spines.values():
        spine.set_edgecolor('#374151')

    fname1 = os.path.join(output_dir, f'partition_{partition}_{n_drones}drones.png')
    fig1.tight_layout()
    fig1.savefig(fname1, dpi=120, bbox_inches='tight', facecolor='#0d1117')
    print(f'[saved] {fname1}')

    # ─── Figure 2: Search Paths ──────────────────────────────
    fig2, ax2 = plt.subplots(figsize=(8, 8))
    fig2.patch.set_facecolor('#0d1117')
    ax2.set_facecolor('#111827')

    ax2.imshow(zone_rgba, origin='upper', extent=[0, W, H, 0], aspect='auto', interpolation='nearest')

    for x in range(W + 1):
        ax2.axvline(x, color='#374151', linewidth=0.3, alpha=0.4)
    for y in range(H + 1):
        ax2.axhline(y, color='#374151', linewidth=0.3, alpha=0.4)

    # Draw search paths
    for i, path in enumerate(all_paths):
        if len(path) < 2:
            continue
        xs = [p[0] + 0.5 for p in path]
        ys = [p[1] + 0.5 for p in path]
        ax2.plot(xs, ys, '-', color=DRONE_COLORS[i], linewidth=0.8, alpha=0.7, zorder=3)
        # Start marker
        ax2.plot(xs[0], ys[0], 'o', color=DRONE_COLORS[i], markersize=10,
                 markeredgecolor='white', markeredgewidth=1, zorder=5)
        ax2.text(xs[0], ys[0], str(i + 1),
                 color='white', fontsize=7, ha='center', va='center',
                 fontweight='bold', zorder=6)
        # End marker
        ax2.plot(xs[-1], ys[-1], 's', color=DRONE_COLORS[i], markersize=8,
                 markeredgecolor='white', markeredgewidth=0.8, alpha=0.8, zorder=5)

    # Obstacles
    for (ox, oy) in obstacles:
        rect = mpatches.FancyBboxPatch((ox + 0.05, oy + 0.05), 0.9, 0.9,
                                       boxstyle='round,pad=0.05',
                                       facecolor='#374151', edgecolor='#6b7280',
                                       linewidth=0.8)
        ax2.add_patch(rect)

    SEARCH_NAMES = {
        'zigzag': 'S-shape Coverage', 'boustrophedon': 'Boustrophedon Scan',
        'spiral': 'Spiral Search', 'random': 'Random Walk'
    }
    legend_handles2 = [
        mpatches.Patch(facecolor=DRONE_COLORS[i], edgecolor='white',
                       linewidth=0.5, label=f'Drone {i+1}')
        for i in range(n_drones)
    ]
    ax2.legend(handles=legend_handles2, loc='upper right',
               facecolor='#16213e', edgecolor='#e94560',
               labelcolor='white', fontsize=8)

    ax2.set_title(f'Drone Search Paths — {SEARCH_NAMES.get(search, search)}\n'
                  f'Partition: {ALGO_NAMES.get(partition, partition)} | circle=Start  square=End',
                  color='#63b3ed', fontsize=11, pad=10)
    ax2.set_xlim(0, W); ax2.set_ylim(H, 0)
    ax2.tick_params(colors='#a0aec0', labelsize=7)
    for spine in ax2.spines.values():
        spine.set_edgecolor('#374151')

    fname2 = os.path.join(output_dir, f'search_{search}_{partition}_{n_drones}drones.png')
    fig2.tight_layout()
    fig2.savefig(fname2, dpi=120, bbox_inches='tight', facecolor='#0d1117')
    print(f'[saved] {fname2}')

    if show:
        plt.show()

    plt.close('all')
    return fname1, fname2


# ─────────────────────────────────────────────────────────
#  CLI Entry Point
# ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='无人机区域划分与搜索路径可视化'
    )
    parser.add_argument('--width',      type=int,   default=30,       help='网格宽度 (默认 30)')
    parser.add_argument('--height',     type=int,   default=30,       help='网格高度 (默认 30)')
    parser.add_argument('--drones',     type=int,   default=3,        help='无人机数量 (默认 3)')
    parser.add_argument('--partition',  type=str,   default='grid',
                        choices=['grid', 'strip', 'quadrant', 'voronoi'],
                        help='区域划分算法 (默认 grid)')
    parser.add_argument('--search',     type=str,   default='zigzag',
                        choices=['zigzag', 'boustrophedon', 'spiral', 'random'],
                        help='搜索路径算法 (默认 zigzag)')
    parser.add_argument('--obstacles',  type=int,   default=20,       help='障碍物数量 (默认 20)')
    parser.add_argument('--output',     type=str,   default='output', help='输出目录 (默认 output)')
    parser.add_argument('--seed',       type=int,   default=0,        help='随机种子 (默认 0，-1 为随机)')
    parser.add_argument('--no-show',    action='store_true',          help='不弹出窗口，只保存文件')
    args = parser.parse_args()

    seed = None if args.seed == -1 else args.seed

    print(f"\nDrone Area Division & Search Path Visualization")
    print(f"{'='*40}")
    print(f"  Grid size  : {args.width} x {args.height}")
    print(f"  Drones     : {args.drones}")
    print(f"  Partition  : {args.partition}")
    print(f"  Search     : {args.search}")
    print(f"  Obstacles  : {args.obstacles}")
    print(f"  Seed       : {seed}")
    print(f"{'='*40}\n")

    visualize(
        W=args.width, H=args.height,
        n_drones=args.drones,
        partition=args.partition,
        search=args.search,
        n_obstacles=args.obstacles,
        output_dir=args.output,
        show=not args.no_show,
        seed=seed,
    )


if __name__ == '__main__':
    main()
