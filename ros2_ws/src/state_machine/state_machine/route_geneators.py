# Map-aware waypoint generation:
#  - Reads map YAML (image, resolution, origin, thresholds)
#  - Scans rows at 'lane_spacing_m' to find long free segments
#  - Drops endpoints as waypoints (alternating direction, lawn-mower)
#  - Converts image pixels -> world xy using origin + resolution
#
# Also includes a simple loader for a YAML LIST of waypoints.

import yaml
from pathlib import Path
from typing import List, Dict, Any, Tuple

try:
    from PIL import Image
except Exception:
    Image = None

Waypoint = Dict[str, float]

def load_map_info(map_yaml_path: str) -> Dict[str, Any]:
    with open(map_yaml_path, 'r') as f:
        info = yaml.safe_load(f)
    for k in ['image', 'resolution', 'origin', 'occupied_thresh', 'free_thresh']:
        if k not in info:
            raise RuntimeError(f"Map YAML missing '{k}': {map_yaml_path}")
    info['image'] = str(Path(map_yaml_path).parent / info['image'])
    return info

def pixel_to_world(ix: int, iy: int, info: Dict[str, Any], img_height: int) -> Tuple[float, float]:
    # ROS map coords: origin is bottom-left of the image; image (0,0) is top-left.
    res = float(info['resolution'])
    ox, oy, _ = [float(v) for v in info['origin']]
    wx = ox + (ix + 0.5) * res
    wy = oy + (img_height - iy - 0.5) * res
    return wx, wy

def _is_free(pixel_val: int, free_thresh: float) -> bool:
    # PGM/PNG grayscale: 0..255, where 255≈free, 0≈occupied
    return (pixel_val / 255.0) >= free_thresh

def boustrophedon_sweep(map_yaml_path: str,
                        lane_spacing_m: float = 1.0,
                        sample_stride_px: int = 4,
                        heading_deg: float = 0.0) -> List[Waypoint]:
    if Image is None:
        raise RuntimeError("Install Pillow (python3-pil) to use map-aware generators.")
    info = load_map_info(map_yaml_path)
    img = Image.open(info['image']).convert('L')
    w, h = img.size
    res = float(info['resolution'])
    free_thresh = float(info.get('free_thresh', 0.196))
    px_per_lane = max(1, int(lane_spacing_m / res))

    waypoints: List[Waypoint] = []
    direction_lr = True  # alternate left->right, then right->left

    for iy in range(0, h, px_per_lane):
        row = img.crop((0, iy, w, iy+1)).getdata()
        run_start = None
        for ix in range(0, w, sample_stride_px):
            free = _is_free(row[ix], free_thresh)
            at_end = (ix >= w - sample_stride_px)
            if free and run_start is None:
                run_start = ix
            if (not free or at_end) and run_start is not None:
                run_end = ix if not free else min(ix + sample_stride_px, w - 1)
                # keep runs >= ~2m
                if (run_end - run_start) * res >= 2.0:
                    ends = [run_start, run_end] if direction_lr else [run_end, run_start]
                    for ex in ends:
                        wx, wy = pixel_to_world(ex, iy, info, h)
                        waypoints.append({'x': wx, 'y': wy, 'theta_deg': heading_deg})
                run_start = None
        direction_lr = not direction_lr

    # prune near-duplicates (<0.5m)
    pruned: List[Waypoint] = []
    last = None
    for wp in waypoints:
        if last is None:
            pruned.append(wp); last = wp; continue
        dx = wp['x'] - last['x']; dy = wp['y'] - last['y']
        if (dx*dx + dy*dy) >= 0.25:
            pruned.append(wp); last = wp
    return pruned

def load_waypoint_list_from_yaml(yaml_path: str) -> List[Waypoint]:
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, list):
        raise RuntimeError(f"Expected a YAML LIST of waypoints in: {yaml_path}")
    # minimal validation
    for i, wp in enumerate(data):
        for k in ['x', 'y']:
            if k not in wp:
                raise RuntimeError(f"Waypoint {i} missing '{k}' in: {yaml_path}")
        wp.setdefault('theta_deg', 0.0)
    return data
