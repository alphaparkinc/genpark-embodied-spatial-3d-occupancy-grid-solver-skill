# -*- coding: utf-8 -*-
import sys, json
from client import GenParkEmbodiedSpatial3DOccupancyGridSolver

solver = GenParkEmbodiedSpatial3DOccupancyGridSolver()

def run_tool(name, args):
    if name == "update_raycasts":
        origin = tuple(args.get("sensor_origin", [0.0, 0.0, 0.0]))
        endpoints = [tuple(ep) for ep in args.get("endpoints", [])]
        return solver.update_raycasts(origin, endpoints)
    elif name == "check_sphere_collision":
        center = tuple(args.get("center", [0.0, 0.0, 0.0]))
        radius = float(args.get("radius", 0.5))
        return solver.check_sphere_collision(center, radius)
    else:
        raise ValueError(f"Unknown tool: {name}")

def main():
    if "--test" in sys.argv:
        print("[TEST] Running GenPark 3D Occupancy Grid Solver Self-Test...")
        res1 = solver.update_raycasts((0,0,0), [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)])
        assert res1["occupied_voxel_count"] >= 3, "Occupied voxels mismatch"
        col = solver.check_sphere_collision((1.0, 0.0, 0.0), 0.15)
        assert col["in_collision"] is True, "Collision should be detected"
        print("[TEST] All tests passed successfully!")
        return

    for line in sys.stdin:
        if not line.strip(): continue
        try:
            req = json.loads(line)
            res = run_tool(req.get("name"), req.get("arguments", {}))
            print(json.dumps({"jsonrpc": "2.0", "id": req.get("id"), "result": res}))
        except Exception as e:
            print(json.dumps({"jsonrpc": "2.0", "id": None, "error": {"code": -32603, "message": str(e)}}))
        sys.stdout.flush()

if __name__ == "__main__":
    main()
