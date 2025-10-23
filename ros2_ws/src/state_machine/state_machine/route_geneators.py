import math
import yaml
from pathlib import Path
from typing import List, Tuple, Dict, Any
try:
    from PIL import Image
except Exception:
    Image = None

# Types: list of dictionaries: {'x': float, 'y': float, 'theta_deg': float}
Waypoint = Dict[str, float]

def load_map_info(map_yaml_path: str) -> Dict[str, Any]:
    with open(map_yaml_path, 'r') as f:
        info = yaml.safe_load(f)
    required = ['image', 'resolution', 'origin', 'occupied_thresh', 'free_thresh']
    for k in required:
        if k not in info:
            raise RuntimeError(f"Map YAML missing '{k}'")
    image_path = str(Path(map_yaml_path).parent / info['image'])
    info['image'] = image_path
    return info

def image_to_world(ix: int, iy: int, info: Dict[str, Any]) -> Tuple[float, float]:
    # ROS map: pixel (0,0) is top-left of image; origin is world coords of bottom-left of image
    res = float(info['resolution'])
    ox, oy, _ = list(map(float, info['origin']))
    img = Image.open(info['image']) if Image else None
    if img:
        height = img.size[1]
    else:
        # Fallback assume top-left reference with given height?
        raise RuntimeError("Pillow (PIL) is required for map-based generation. Install python3-pil.")
    wx = ox + (ix + 0.5) * res
    wy = oy + (height - iy - 0.5) * res
    return wx, wy

def is_free(pixel_val: int, mode: str, occ_thresh: float, free_thresh: float) -> bool:
    # YAML uses normalized [0..1] thresholds. PGM is 0..255 (0=occupied, 255=free)
    norm = pixel_val / 255.0
    # Consider free if greater than free_thresh
    return norm >= free_thresh

def boustrophedon_sweep(map_yaml_path: str,
                        lane_spacing_m: float = 1.0,
                        sample_stride_px: int = 4,
                        heading_deg: float = 0.0) -> List[Waypoint]:
    """
    Very simple row-based sweep:
    - Walk horizontal scanlines every ~lane_spacing_m
    - Within each scanline, sample pixels; when we find long free segments,
      place waypoints at segment ends and alternate direction (lawnmower).
    """
    info = load_map_info(map_yaml_path)
    if Image is None:
        raise RuntimeError("Install Pillow (python3-pil) to use map-aware generators.")
    img = Image.open(info['image']).convert('L')
    w, h = img.size
    res = float(info['resolution'])
    px_per_lane = max(1, int(lane_spacing_m / res))
    occ_thresh = float(info.get('occupied_thresh', 0.65))
    free_thresh = float(info.get('free_thresh', 0.196))

    waypoints: List[Waypoint] = []
    # Scan from top to bottom (image coords)
    direction_left_to_right = True
    for iy in range(0, h, px_per_lane):
        # find free run segments along this row
        run_start = None
        row = img.crop((0, iy, w, iy+1)).getdata()
        for ix in range(0, w, sample_stride_px):
            free = is_free(row[ix], img.mode, occ_thresh, free_thresh)
            if free and run_start is None:
                run_start = ix
            if (not free or ix >= w - sample_stride_px) and run_start is not None:
                run_end = ix if free is False else min(ix + sample_stride_px, w-1)
                # If run is long enough (at least ~2m), keep it
                if (run_end - run_start) * res >= 2.0:
                    # endpoints in alternating direction
                    segment = list(range(run_start, run_end, sample_stride_px))
                    if direction_left_to_right:
                        endpoints = [run_start, run_end]
                    else:
                        endpoints = [run_end, run_start]
                    for ex in endpoints:
                        wx, wy = image_to_world(ex, iy, info)
                        waypoints.append({'x': wx, 'y': wy, 'theta_deg': heading_deg})
                run_start = None
        direction_left_to_right = not direction_left_to_right

    # Deduplicate nearby points (optional)
    pruned: List[Waypoint] = []
    last = None
    for wp in waypoints:
        if last is None:
            pruned.append(wp); last = wp; continue
        dx = wp['x'] - last['x']; dy = wp['y'] - last['y']
        if (dx*dx + dy*dy) >= (0.5*0.5):  # keep if >= 0.5m apart
            pruned.append(wp); last = wp
    return pruned

