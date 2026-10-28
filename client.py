# -*- coding: utf-8 -*-
"""
GenPark Physical AI 3D Spatial Occupancy Voxel Grid Solver
Standard Library only: math, collections, typing.
Implements 3D Bresenham raycasting, log-odds Bayesian occupancy updating,
and radial/AABB obstacle query engine.
"""

import math
from typing import Dict, List, Tuple, Any

class GenParkEmbodiedSpatial3DOccupancyGridSolver:
    def __init__(self, resolution: float = 0.1, l_occ: float = 0.85, l_free: float = -0.4,
                 l_min: float = -5.0, l_max: float = 5.0, occ_threshold: float = 0.65):
        self.resolution = resolution
        self.l_occ = l_occ
        self.l_free = l_free
        self.l_min = l_min
        self.l_max = l_max
        self.occ_threshold = occ_threshold
        # Map: (ix, iy, iz) -> log_odds float
        self.grid: Dict[Tuple[int, int, int], float] = {}

    def _to_voxel(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        return (
            int(math.floor(x / self.resolution)),
            int(math.floor(y / self.resolution)),
            int(math.floor(z / self.resolution))
        )

    def _to_coords(self, ix: int, iy: int, iz: int) -> Tuple[float, float, float]:
        return (
            (ix + 0.5) * self.resolution,
            (iy + 0.5) * self.resolution,
            (iz + 0.5) * self.resolution
        )

    def _log_odds_to_prob(self, l: float) -> float:
        return 1.0 - (1.0 / (1.0 + math.exp(l)))

    def _raycast_3d(self, start: Tuple[int, int, int], end: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
        """3D Bresenham line algorithm generating all traversed voxels."""
        x0, y0, z0 = start
        x1, y1, z1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        dz = abs(z1 - z0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        sz = 1 if z1 > z0 else -1

        voxels = []
        if dx >= dy and dx >= dz:
            p1 = 2 * dy - dx
            p2 = 2 * dz - dx
            while x0 != x1:
                voxels.append((x0, y0, z0))
                x0 += sx
                if p1 >= 0:
                    y0 += sy
                    p1 -= 2 * dx
                if p2 >= 0:
                    z0 += sz
                    p2 -= 2 * dx
                p1 += 2 * dy
                p2 += 2 * dz
        elif dy >= dx and dy >= dz:
            p1 = 2 * dx - dy
            p2 = 2 * dz - dy
            while y0 != y1:
                voxels.append((x0, y0, z0))
                y0 += sy
                if p1 >= 0:
                    x0 += sx
                    p1 -= 2 * dy
                if p2 >= 0:
                    z0 += sz
                    p2 -= 2 * dy
                p1 += 2 * dx
                p2 += 2 * dz
        else:
            p1 = 2 * dy - dz
            p2 = 2 * dx - dz
            while z0 != z1:
                voxels.append((x0, y0, z0))
                z0 += sz
                if p1 >= 0:
                    y0 += sy
                    p1 -= 2 * dz
                if p2 >= 0:
                    x0 += sx
                    p2 -= 2 * dz
                p1 += 2 * dy
                p2 += 2 * dx
        return voxels

    def update_raycasts(self, sensor_origin: Tuple[float, float, float],
                         endpoints: List[Tuple[float, float, float]]) -> Dict[str, Any]:
        v_start = self._to_voxel(*sensor_origin)
        updated_free = 0
        updated_occ = 0

        for ep in endpoints:
            v_end = self._to_voxel(*ep)
            traversed = self._raycast_3d(v_start, v_end)
            for v in traversed:
                old_l = self.grid.get(v, 0.0)
                new_l = max(self.l_min, min(self.l_max, old_l + self.l_free))
                self.grid[v] = new_l
                updated_free += 1

            old_l = self.grid.get(v_end, 0.0)
            new_l = max(self.l_min, min(self.l_max, old_l + self.l_occ))
            self.grid[v_end] = new_l
            updated_occ += 1

        occupied_count = sum(1 for v, l in self.grid.items() if self._log_odds_to_prob(l) >= self.occ_threshold)
        return {
            "total_voxels_tracked": len(self.grid),
            "free_updates": updated_free,
            "occupied_updates": updated_occ,
            "occupied_voxel_count": occupied_count,
            "resolution_m": self.resolution
        }

    def check_sphere_collision(self, center: Tuple[float, float, float], radius: float) -> Dict[str, Any]:
        cx, cy, cz = center
        r_vox = int(math.ceil(radius / self.resolution))
        c_vox = self._to_voxel(cx, cy, cz)

        colliding_voxels = []
        max_prob = 0.0

        for dx in range(-r_vox, r_vox + 1):
            for dy in range(-r_vox, r_vox + 1):
                for dz in range(-r_vox, r_vox + 1):
                    dist = math.sqrt(dx*dx + dy*dy + dz*dz) * self.resolution
                    if dist <= radius:
                        vox = (c_vox[0] + dx, c_vox[1] + dy, c_vox[2] + dz)
                        if vox in self.grid:
                            p = self._log_odds_to_prob(self.grid[vox])
                            if p > max_prob:
                                max_prob = p
                            if p >= self.occ_threshold:
                                colliding_voxels.append({
                                    "voxel": vox,
                                    "coords": self._to_coords(*vox),
                                    "occupancy_prob": round(p, 4),
                                    "dist_to_center": round(dist, 4)
                                })

        return {
            "in_collision": len(colliding_voxels) > 0,
            "max_occupancy_prob": round(max_prob, 4),
            "collision_voxel_count": len(colliding_voxels),
            "query_radius": radius,
            "query_center": center,
            "samples": colliding_voxels[:5]
        }
