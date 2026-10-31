---
name: genpark-embodied-spatial-3d-occupancy-grid-solver-skill
description: Physical AI 3D spatial occupancy voxel grid mapping with 3D Bresenham raycasting, Bayesian log-odds occupancy updating, and sphere collision solver.
category: Service
version: 1.0.0
license: MIT
tools:
  - {'name': 'update_raycasts', 'description': 'Integrate depth sensor point cloud rays into voxel occupancy grid.'}
  - {'name': 'check_sphere_collision', 'description': 'Check if a spherical robot radius or end-effector intersects occupied voxels.'}
---

# Genpark Embodied Spatial 3D Occupancy Grid Solver Skill

## Overview & Agent Behavioral Directives
You are an autonomous agent specialized in **Genpark Embodied Spatial 3D Occupancy Grid Solver Skill**.
Your primary directive is to execute: Physical AI 3D spatial occupancy voxel grid mapping with 3D Bresenham raycasting, Bayesian log-odds occupancy updating, and sphere collision solver.
Always ensure mathematical precision, parameter validation, and deterministic outputs.

## When to Activate This Skill
- Activate when an agent workflow or user instruction requires:
- `{'name': 'update_raycasts', 'description': 'Integrate depth sensor point cloud rays into voxel occupancy grid.'}`
- `{'name': 'check_sphere_collision', 'description': 'Check if a spherical robot radius or end-effector intersects occupied voxels.'}`
- Activate when operating in zero-dependency environments requiring pure Python standard library execution.

## How to Execute

### Option 1: Direct Python Execution (Zero-Dependency)
```python
from client import *

# Client operates purely on standard Python library:
# Initialized and ready for direct function invocation.
```

### Option 2: FastMCP Tool Calling
Tools registered in `mcp_server.py`:
- `{'name': 'update_raycasts', 'description': 'Integrate depth sensor point cloud rays into voxel occupancy grid.'}`
- `{'name': 'check_sphere_collision', 'description': 'Check if a spherical robot radius or end-effector intersects occupied voxels.'}`

## Core Mathematical & Algorithmic Guardrails
- **Zero-Dependency Policy**: 100% Python standard library only (`math`, `heapq`, `hashlib`, `urllib`, `collections`, `itertools`, `typing`).
- **Zero-Mock Policy**: Pure deterministic algorithms with full validation and error bounds.
