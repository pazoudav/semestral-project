# octomap_planner_utils

## Purpose

This package is the shared geometry/octomap helper library for the energy-aware UAV coverage
path planning system. Several independent nodelets in this workspace — `mrs_octomap_planner`
(the `Explorer`) and `frontier_detection` — need the same low-level building blocks: axis-aligned
bounding box (AABB) math, octomap voxel-neighbor lookups, free-space checks against a live
`octomap::OcTree`, random sampling within a region, and a helper to read the UAV's current
position (transformed into the octree frame). Rather than duplicating that logic in each nodelet,
it is factored out here into a single plain (non-nodelet) library that the consuming packages link
against and call with the `octomap_planner_utils::` namespace prefix.

## ROS interface

This package exposes **no ROS interface of its own** — it publishes no topics, subscribes to no
topics, and advertises no services or actions. It is a plain linked library (no nodelet, no node
executable); its `getPosition()`/`getFullStatePrediction()` helpers take already-constructed
`mrs_lib::SubscribeHandler` objects owned by the *calling* nodelet as arguments, they do not create
their own subscriptions. Consuming packages (`mrs_octomap_planner`, `frontier_detection`) own all
actual ROS communication.

## Important functions/types

Types:
- `color_t` — RGB color in [0,1] range, used for visualization markers.
- `AABB` — axis-aligned bounding box defined by its min and max corner points.
- `path_info_t` — cost breakdown for a path segment between two points (distance, velocity/heading
  change, height), produced by a `distance_funcion_t`.
- `distance_funcion_t` — signature of a pairwise cost/distance function between two points,
  returning a `path_info_t` breakdown (e.g. used as an edge-cost metric for TSP ordering).
- `INVALID_DISTANCE` / `BIG_DISTANCE` — sentinel distance values for "no path" / "effectively
  infinite cost".
- `NeighbourOffset` — POD struct `{int dx, dy, dz;}`, a single integer offset to a neighbouring
  voxel.
- `NEIGHBOUR_OFFSETS` — `const std::array<NeighbourOffset, 26>` of the offsets to a voxel's
  neighbours in a 3x3x3 block, excluding itself.

Functions:
- `getColor(int i)` — returns a color from a small fixed palette, cycling every 15 indices.
- `getNeighbourKey(key, const NeighbourOffset&)` / `getNeighboursKeys` — octomap key(s) of a
  voxel's neighbour(s); `getNeighbourKey` takes its offset by const reference (avoids a per-call
  copy — it is called millions of times during a zone flood-fill).
- `sampleSpherePoints(int n)` — n quasi-uniform points on the unit sphere (Fibonacci sphere);
  currently unused.
- `getConrners(AABB a)` — the 8 corner points of an AABB.
- `aabbFromCenter` / `aabbSmartFromCenter` — build an AABB from a center point and side lengths
  (the latter additionally clamps to a floor level; currently unused).
- `localZoneFromPosition(octomap::point3d, ...)` / `localZoneFromPosition(geometry_msgs::Point, ...)`
  — AABB centered on a position, sized `(width, width, height)`, clamped to a flight zone; the
  `geometry_msgs::Point` overload delegates to the `octomap::point3d` one via `pointToOctomap`.
- `intersect(AABB, point3d)` / `intersect(AABB, AABB)` — point-in-box / box-overlap tests.
- `isSmallerEq` / `isBiggerEq` — component-wise vector comparisons.
- `makeUnion` / `makeIntersection` / `isSubset` / `volume` — AABB set operations and volume.
- `isFreeSpace(AABB, tree)` / `isFreeSpace(center, diameter, tree)` — check that a region (box or
  approximate sphere) is entirely known-and-free in an `octomap::OcTree`.
- `getRand()` / `getRand(a,b)` / `getSampleFromAABB` — uniform random sampling helpers.
- `getPosition(...)` — reads the UAV's current commanded position (from `mrs_msgs::TrackerCommand`
  via a caller-owned `SubscribeHandler`) and transforms it into the octree frame; assumes the
  octree mutex is already held by the caller when reading `octree_frame`.
- `getFullStatePrediction(...)` — reads the tracker's full-state MPC prediction
  (`mrs_msgs::TrackerCommand::full_state_prediction`, via the same two caller-owned
  `SubscribeHandler`s as `getPosition`) and returns it once both the control-manager diagnostics
  and tracker command are fresh (<2s old) and a transform from the prediction's frame into
  `octree_frame` exists; assumes the octree mutex is already held by the caller. Note it only
  checks that `transformer.getTransform(...)` succeeds — it does not apply that transform to the
  returned prediction, so the returned message is still expressed in its original `header.frame_id`
  rather than `octree_frame`.

## File structure

```
octomap_planner_utils/
├── CMakeLists.txt                          # catkin build: builds the OctomapPlannerUtils library from src/utils.cpp
├── package.xml                             # catkin package manifest / dependency declarations
├── include/
│   └── octomap_planner_utils/
│       └── utils.hpp                       # public API: AABB/color/path_info types, distance_funcion_t, and all free-function declarations
└── src/
    └── utils.cpp                           # implementation of every function declared in utils.hpp
```

## Dependencies

From `package.xml` / `CMakeLists.txt`:
- `cmake_modules` — CMake helper modules used by the catkin build.
- `roscpp` — core ROS C++ client library (logging macros, `ros::Time`, etc. used by `getPosition`
  and `getFullStatePrediction`).
- `octomap_msgs`, `octomap_ros` — ROS message/conversion glue for `octomap`, pulled in alongside
  the core `octomap` library.
- `mrs_lib` — provides `mrs_lib::SubscribeHandler` and `mrs_lib::Transformer`, used by
  `getPosition()`/`getFullStatePrediction()` to read and transform the UAV's tracker command.
- `mrs_msgs` — message types (`ReferenceStamped`, `TrackerCommand`, `ControlManagerDiagnostics`,
  `MpcPredictionFullState`) consumed/produced by `getPosition()`/`getFullStatePrediction()`.
- `octomap` (via `find_package(octomap REQUIRED)`) — the actual octree data structure
  (`octomap::OcTree`, `octomap::point3d`, `octomap::OcTreeKey`) that most of this library's
  geometry/free-space functions operate on.
- `geometry_msgs` (header-only use of `geometry_msgs::Point`, pulled in transitively) — used by
  `localZoneFromPosition`.
