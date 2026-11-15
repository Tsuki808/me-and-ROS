# -*- coding: utf-8 -*-
"""Coordinate and completion rules, runnable without ROS."""
from __future__ import division
import math
import os


def scene_anchor(scene_pose, map_pose):
    yaw = map_pose[2] - scene_pose[2]
    c, s = math.cos(yaw), math.sin(yaw)
    return [map_pose[0] - c * scene_pose[0] + s * scene_pose[1],
            map_pose[1] - s * scene_pose[0] - c * scene_pose[1], yaw]


def transform_scene(point, anchor, pose=False):
    c, s = math.cos(anchor[2]), math.sin(anchor[2])
    result = [anchor[0] + c * point[0] - s * point[1],
              anchor[1] + s * point[0] + c * point[1], point[2]]
    if pose:
        result[2] += anchor[2]
    return result


def sample_map(grid, anchor, bounds, step):
    """Sample fixed scene cell centers, independent of SLAM map resizing."""
    width, height, resolution = grid['width'], grid['height'], grid['resolution']
    ox, oy, yaw = grid['origin']
    c, s = math.cos(yaw), math.sin(yaw)
    result = []
    nx = int(round((bounds[2] - bounds[0]) / step))
    ny = int(round((bounds[3] - bounds[1]) / step))
    for row in range(ny):
        for col in range(nx):
            point = transform_scene([bounds[0] + (col + .5) * step,
                                     bounds[1] + (row + .5) * step, 0.], anchor)
            dx, dy = point[0] - ox, point[1] - oy
            x = int(math.floor((c * dx + s * dy) / resolution))
            y = int(math.floor((-s * dx + c * dy) / resolution))
            value = grid['data'][y * width + x] if 0 <= x < width and 0 <= y < height else -1
            result.append(0 if 0 <= value <= 25 else (1 if value >= 65 else -1))
    return result


def residual_regions(grid, points, anchor, bounds, component_limit, total_limit):
    """Measure frontier-connected unknown components; never fill unknown cells."""
    result = dict(accepted=False, component_areas_m2=[], total_area_m2=0.)
    if not points or component_limit <= 0 or total_limit <= 0:
        result['reason'] = 'Missing frontier evidence or invalid area limits'
        return result
    width, height, resolution = grid['width'], grid['height'], grid['resolution']
    ox, oy, yaw = grid['origin']
    c, s = math.cos(yaw), math.sin(yaw)
    ac, ass = math.cos(anchor[2]), math.sin(anchor[2])
    visited = set()
    for point in points:
        dx, dy = point[0] - ox, point[1] - oy
        x = int(math.floor((c * dx + s * dy) / resolution))
        y = int(math.floor((-s * dx + c * dy) / resolution))
        if not (0 <= x < width and 0 <= y < height):
            result['reason'] = 'Frontier outside available map'
            return result
        seed = y * width + x
        if seed in visited or grid['data'][seed] >= 0:
            continue
        queue, visited_seed = [seed], 0
        visited.add(seed)
        while visited_seed < len(queue):
            cell = queue[visited_seed]
            visited_seed += 1
            if visited_seed * resolution ** 2 > component_limit + 1e-9:
                result['reason'] = 'Unknown component exceeds per-component area limit'
                return result
            x, y = cell % width, cell // width
            wx = ox + c * (x + .5) * resolution - s * (y + .5) * resolution
            wy = oy + s * (x + .5) * resolution + c * (y + .5) * resolution
            sx = ac * (wx - anchor[0]) + ass * (wy - anchor[1])
            sy = -ass * (wx - anchor[0]) + ac * (wy - anchor[1])
            if not (bounds[0] <= sx <= bounds[2] and bounds[1] <= sy <= bounds[3]):
                result['reason'] = 'Unknown component extends beyond search bounds'
                return result
            for nx, ny in [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]:
                if not (0 <= nx < width and 0 <= ny < height):
                    result['reason'] = 'Unknown component reaches map edge'
                    return result
                neighbor = ny * width + nx
                if neighbor not in visited and grid['data'][neighbor] < 0:
                    visited.add(neighbor)
                    queue.append(neighbor)
        area = len(queue) * resolution ** 2
        result['component_areas_m2'].append(area)
        result['total_area_m2'] += area
        if result['total_area_m2'] > total_limit + 1e-9:
            result['reason'] = 'Residual components exceed total area limit'
            return result
    result['accepted'] = True
    result['reason'] = 'Only bounded small residual unknown regions remain'
    return result


class Completion(object):
    def __init__(self, duration=10., change_limit=.001):
        self.duration, self.limit = duration, change_limit
        self.since, self.reference = None, None
        self.changed = set()

    def update(self, now, state, failures, fresh, cells, cancelled=False, residuals=None):
        terminal = ((state == 'EXHAUSTED' and not failures) or
                    (state in ['ACTIVE', 'RETRYING', 'BLOCKED', 'PAUSED'] and
                     residuals is not None and residuals.get('accepted') is True))
        if cancelled or not fresh or not terminal:
            self.since, self.reference = None, None
            self.changed.clear()
            return False
        if not cells:
            return False
        if self.reference is not None and len(cells) == len(self.reference):
            self.changed.update(i for i, (a, b) in enumerate(zip(cells, self.reference)) if a != b)
        if (self.reference is None or len(cells) != len(self.reference) or
                len(self.changed) / len(cells) > self.limit or now < self.since):
            self.since, self.reference = now, list(cells)
            self.changed.clear()
        return now - self.since >= self.duration


def validate_saved_map(yaml_path, metadata, grid):
    """Directly compare map_saver's pixels/metadata with the frozen grid."""
    if abs(metadata['resolution'] - grid['resolution']) > 1e-6:
        raise ValueError('Saved map resolution mismatch')
    if len(metadata['origin']) != 3 or any(abs(a - b) > 1e-6 for a, b in zip(metadata['origin'], grid['origin'])):
        raise ValueError('Saved map origin mismatch')
    if (metadata.get('negate') != 0 or abs(metadata['occupied_thresh'] - .65) > 1e-6 or
            abs(metadata['free_thresh'] - .196) > 1e-6):
        raise ValueError('Unexpected saved map interpretation')
    image = os.path.join(os.path.dirname(yaml_path), metadata['image'])
    with open(image, 'rb') as stream:
        raw = bytearray(stream.read())
    position = [0]
    def token():
        while position[0] < len(raw):
            value = raw[position[0]]
            if value in [9, 10, 13, 32]:
                position[0] += 1
            elif value == 35:
                while position[0] < len(raw) and raw[position[0]] != 10:
                    position[0] += 1
            else:
                break
        start = position[0]
        while position[0] < len(raw) and raw[position[0]] not in [9, 10, 13, 32]:
            position[0] += 1
        return bytes(raw[start:position[0]]).decode('ascii')
    magic, width, height, maximum = token(), int(token()), int(token()), int(token())
    if magic != 'P5' or maximum != 255 or (width, height) != (grid['width'], grid['height']):
        raise ValueError('Saved map dimensions/format mismatch')
    if position[0] >= len(raw) or raw[position[0]] not in [9, 10, 13, 32]:
        raise ValueError('Missing PGM header delimiter')
    position[0] += 2 if raw[position[0]:position[0] + 2] == bytearray(b'\r\n') else 1
    pixels = raw[position[0]:]
    if len(pixels) != width * height:
        raise ValueError('Saved map pixel count mismatch')
    for row in range(height):
        for col in range(width):
            value = grid['data'][(height - row - 1) * width + col]
            expected = 254 if 0 <= value <= 25 else (0 if value >= 65 else 205)
            if pixels[row * width + col] != expected:
                raise ValueError('Saved map differs from frozen snapshot')
