"""Deterministic 2-D A*: fixed-world AABBs, inflated obstacles, checked smoothing."""
from __future__ import annotations

import heapq
import math
import random
import numpy as np


def segment_box(a, b, lo, hi, margin=0.):
    a, b, lo, hi = [np.asarray(v, dtype=float) for v in (a, b, lo, hi)]
    if any(v.shape != (2,) or not np.isfinite(v).all() for v in (a, b, lo, hi)):
        raise ValueError("Expected finite 2-D vectors")
    if np.any(lo > hi) or not math.isfinite(margin) or margin < 0:
        raise ValueError("Invalid obstacle or margin")
    lo, hi = lo - margin, hi + margin
    t0, t1 = 0., 1.
    for k, d in enumerate(b - a):
        if abs(d) < 1e-12:
            if not lo[k] <= a[k] <= hi[k]:
                return False
        else:
            x, y = (lo[k] - a[k]) / d, (hi[k] - a[k]) / d
            t0, t1 = max(t0, min(x, y)), min(t1, max(x, y))
            if t0 > t1:
                return False
    return True


def line_free(a, b, obstacles, margin):
    return not any(segment_box(a, b, item["min"][:2], item["max"][:2], margin) for item in obstacles)


def clearance(point, obstacles):
    """Unsigned point-to-box distance; zero inside. Not full robot mesh clearance."""
    p = np.asarray(point)
    return min((float(np.linalg.norm(np.maximum(np.maximum(np.array(o["min"][:2]) - p,
                    p - np.array(o["max"][:2])), 0))) for o in obstacles), default=float("inf"))


def random_obstacle(config, seed):
    # Draw once; no resampling until success, and no test-dependent seed selection.
    rng = random.Random(seed)
    start, goal = np.array(config["start_xy"]), np.array(config["goal_xy"])
    forward = (goal - start) / np.linalg.norm(goal - start)
    lateral = np.array([-forward[1], forward[0]])
    position = start + rng.uniform(*config["random_along_fraction"]) * (goal - start)
    position += rng.uniform(*config["random_lateral_m"]) * lateral
    size = np.array(config["obstacle_size_m"])
    center = np.r_[position, config["floor_z"] + size[2] / 2]
    return {"path": "/World/KetiNav/Obstacle", "center": center.tolist(), "size": size.tolist(),
            "min": (center - size / 2).tolist(), "max": (center + size / 2).tolist(), "seed": int(seed)}


def astar(start, goal, obstacles, bounds, resolution, margin):
    start, goal = np.array(start, dtype=float), np.array(goal, dtype=float)
    low, high = np.array(bounds[:2], dtype=float), np.array(bounds[2:], dtype=float)
    if resolution <= 0 or margin < 0 or not np.isfinite([*bounds, resolution, margin]).all():
        raise ValueError("Invalid grid configuration")
    if np.any(low >= high):
        raise ValueError("Invalid grid bounds")
    # Objects outside the planning region cannot intersect a path in this region.
    # Keep the inflation in this pruning test; never shift obstacles with goals.
    obstacles = [o for o in obstacles if np.all(np.array(o["max"][:2]) + margin >= low)
                 and np.all(np.array(o["min"][:2]) - margin <= high)]
    if any(np.any(p < low) or np.any(p > high) for p in (start, goal)):
        raise ValueError("Start or goal outside the fixed grid")
    for point in (start, goal):
        if not line_free(point, point, obstacles, margin):
            raise RuntimeError("Start/goal occupied; do not move target automatically")
    shape = np.floor((high - low) / resolution + 1e-6).astype(int) + 1
    to_cell = lambda p: tuple(np.rint((p - low) / resolution).astype(int))
    to_world = lambda c: low + np.array(c) * resolution
    source, target = to_cell(start), to_cell(goal)
    for p, c in ((start, source), (goal, target)):
        if not line_free(p, to_world(c), obstacles, margin):
            raise RuntimeError("Endpoint cannot connect to grid")
    occupied = {}

    def free(c):
        if not (0 <= c[0] < shape[0] and 0 <= c[1] < shape[1]):
            return False
        if c not in occupied:
            occupied[c] = line_free(to_world(c), to_world(c), obstacles, margin)
        return occupied[c]

    queue = [(float(np.linalg.norm(start - goal)), 0., source)]
    cost, previous = {source: 0.}, {}
    expanded = 0
    while queue:
        _, g, current = heapq.heappop(queue)
        if g > cost.get(current, float("inf")) + 1e-10:
            continue
        expanded += 1
        if current == target:
            break
        for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1)):
            nxt = (current[0] + dx, current[1] + dy)
            if not free(nxt):
                continue
            if dx and dy and (not free((current[0] + dx, current[1])) or not free((current[0], current[1] + dy))):
                continue
            if not line_free(to_world(current), to_world(nxt), obstacles, margin):
                continue
            tentative = g + resolution * math.hypot(dx, dy)
            if tentative < cost.get(nxt, float("inf")) - 1e-10:
                cost[nxt], previous[nxt] = tentative, current
                heapq.heappush(queue, (tentative + float(np.linalg.norm(to_world(nxt) - goal)), tentative, nxt))
    if target not in cost:
        raise RuntimeError("NO_PATH: retain target and stop")
    cells, current = [target], target
    while current != source:
        current = previous[current]
        cells.append(current)
    points = [start] + [to_world(c) for c in reversed(cells)] + [goal]
    raw = [points[0]]
    for p in points[1:]:
        if np.linalg.norm(raw[-1] - p) > 1e-8:
            raw.append(p)
    smooth, i = [raw[0]], 0
    while i < len(raw) - 1:
        j = len(raw) - 1
        while j > i + 1 and not line_free(raw[i], raw[j], obstacles, margin):
            j -= 1
        smooth.append(raw[j])
        i = j
    if any(not line_free(a, b, obstacles, margin) for a, b in zip(smooth, smooth[1:])):
        raise RuntimeError("Smoothed path failed collision check")
    return {"path_xy": [p.tolist() for p in smooth], "raw_path_xy": [p.tolist() for p in raw],
            "length_m": sum(float(np.linalg.norm(b - a)) for a, b in zip(smooth, smooth[1:])),
            "expanded_nodes": expanded, "direct_path_blocked": not line_free(start, goal, obstacles, margin)}


def lookahead_target(point, path, progress, distance):
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    cumulative = np.r_[0., np.cumsum(lengths)]
    best = (float("inf"), progress)
    for i, (a, b) in enumerate(zip(path, path[1:])):
        fraction = float(np.clip(np.dot(point - a, b - a) / max(lengths[i] ** 2, 1e-12), 0, 1))
        s = cumulative[i] + fraction * lengths[i]
        if s + 1e-8 < progress:
            continue
        distance_to_segment = float(np.linalg.norm(point - (a + fraction * (b - a))))
        if distance_to_segment < best[0]:
            best = distance_to_segment, s
    progress = max(progress, best[1])
    target_s = min(progress + distance, cumulative[-1])
    i = min(len(lengths) - 1, max(0, int(np.searchsorted(cumulative, target_s, side="right")) - 1))
    fraction = (target_s - cumulative[i]) / max(lengths[i], 1e-12)
    return path[i] + fraction * (path[i + 1] - path[i]), progress
