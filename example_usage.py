# -*- coding: utf-8 -*-
from client import GenParkEmbodiedSpatial3DOccupancyGridSolver

def main():
    solver = GenParkEmbodiedSpatial3DOccupancyGridSolver(resolution=0.05)
    print("=== GenPark Physical AI 3D Spatial Occupancy Grid ===")

    origin = (0.0, 0.0, 1.0)
    simulated_depth_hits = [
        (1.2, 0.0, 1.0),
        (1.2, 0.1, 1.0),
        (1.2, -0.1, 1.0),
        (0.8, 0.8, 0.5)
    ]
    update_res = solver.update_raycasts(origin, simulated_depth_hits)
    print(f"Updated Voxel Grid: {update_res}")

    # Query collision in front of robot
    col_check = solver.check_sphere_collision((1.2, 0.0, 1.0), radius=0.1)
    print(f"Collision Check (Obstacle Hit): in_collision={col_check['in_collision']}, max_prob={col_check['max_occupancy_prob']}")

    # Query free space
    free_check = solver.check_sphere_collision((0.5, 0.0, 1.0), radius=0.1)
    print(f"Collision Check (Free Corridor): in_collision={free_check['in_collision']}, max_prob={free_check['max_occupancy_prob']}")

if __name__ == "__main__":
    main()
