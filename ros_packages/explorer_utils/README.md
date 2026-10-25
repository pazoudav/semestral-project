# explorer_utils

## Purpose

This package is the shared geometry/octomap helper library for the energy-aware UAV coverage
path planning system. Several independent nodelets in this workspace — `explorer`
(the `Explorer`) and `frontier_detector` — need the same low-level building blocks: axis-aligned
bounding box (AABB) math, octomap voxel-neighbor lookups, free-space checks against a live
`octomap::OcTree`, random sampling within a region, and a helper to read the UAV's current
position (transformed into the octree frame). Rather than duplicating that logic in each nodelet,
it is factored out here into a single plain (non-nodelet) library that the consuming packages link
against and call with the `explorer_utils::` namespace prefix.

## ROS interface

This package has **no node, nodelet, or fixed topics/services of its own**. It is a plain linked
library; its `getPosition()`/`getFullStatePrediction()` helpers take already-constructed
`mrs_lib::SubscribeHandler` objects owned by the *calling* nodelet as arguments, they do not create
their own subscriptions. The one optional ROS interface is the header-only `TimingPublisher`
helper (`timing.hpp`), which publishes in the *caller's* namespace:

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `<nodehandle ns>/timing/<name>` | `std_msgs/Float64` | published | timing measurement in ms; one topic per `<name>`, advertised lazily on first use (queue size 10) |

Consuming packages (`explorer`, `frontier_detector`) own all other ROS communication.

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

Classes (`timing.hpp`, header-only):
- `TimingPublisher` — constructed from a `ros::NodeHandle` (topics go under its `timing/` sub-namespace;
  default-constructed instance is also allowed). `publish(name, ms)` publishes a `std_msgs/Float64`
  on `timing/<name>`, lazily advertising one publisher per name; `publishSince(name, ros::WallTime start)`
  publishes and returns the wall-clock ms elapsed since `start`. Mutex-protected, so safe to call
  from multiple threads/callbacks.

Functions:
- `getColor(int i)` — returns a color from a small fixed palette, cycling every 15 indices.
- `getNeighbourKey(key, const NeighbourOffset&)` — octomap key of a single voxel's neighbour,
  given a `(dx,dy,dz)` offset; takes its offset by const reference (avoids a per-call copy — it is
  called millions of times during a zone flood-fill).
- `sampleSpherePoints(int n)` — n quasi-uniform points on the unit sphere (Fibonacci sphere);
  currently unused.
- `getConrners(AABB a)` — the 8 corner points of an AABB, returned as a `std::array` (no heap
  allocation).
- `aabbFromCenter` / `aabbSmartFromCenter` — build an AABB from a center point and side lengths
  (the latter additionally clamps to a floor level; currently unused).
- `localZoneFromPosition(octomap::point3d, ...)` / `localZoneFromPosition(geometry_msgs::Point, ...)`
  — AABB centered on a position, sized `(width, width, height)`, clamped to a flight zone; the
  `geometry_msgs::Point` overload delegates to the `octomap::point3d` one via `pointToOctomap`.
- `intersect(AABB, point3d)` / `intersect(AABB, AABB)` — point-in-box / box-overlap tests; the
  box-overlap test is an allocation-free separating-axis test built on `isSmallerEq`/`isBiggerEq`
  (also correctly handles the "cross" case where neither box's corners lie inside the other).
- `isSmallerEq` / `isBiggerEq` — component-wise vector comparisons.
- `makeUnion` / `makeIntersection` / `isSubset` / `volume` — AABB set operations and volume.
- `isFreeSpace(AABB, const octomap::OcTree&)` / `isFreeSpace(center, diameter, const octomap::OcTree&)`
  — check that a region (box or approximate sphere) is entirely known-and-free in an
  `octomap::OcTree`, both taking the tree by const reference. The box variant walks only the leaves
  overlapping the zone via octomap's native `begin_leafs_bbx`/`end_leafs_bbx` iterator and compares
  the known-free leaves' clipped volume against the zone's volume, rather than descending the tree
  per fine voxel. The sphere variant still does a per-voxel spherical scan (unchanged) since it sits
  on `explorer`'s 20Hz collision-check hot path.
- `getRand()` / `getRand(a,b)` / `getSampleFromAABB` — uniform random sampling helpers; `getRand()`
  draws from a thread-local, `std::random_device`-seeded `std::mt19937` (rather than libc `rand()`).
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
explorer_utils/
├── CMakeLists.txt                          # catkin build: builds the ExplorerUtils library from src/utils.cpp
├── package.xml                             # catkin package manifest / dependency declarations
├── include/
│   └── explorer_utils/
│       ├── utils.hpp                       # public API: AABB/color/path_info types, distance_funcion_t, and all free-function declarations
│       └── timing.hpp                      # header-only TimingPublisher (named timing measurements as Float64 topics)
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
- `std_msgs` — `Float64`, published by `TimingPublisher`.
- `octomap` (via `find_package(octomap REQUIRED)`) — the actual octree data structure
  (`octomap::OcTree`, `octomap::point3d`, `octomap::OcTreeKey`) that most of this library's
  geometry/free-space functions operate on.
- `geometry_msgs` (header-only use of `geometry_msgs::Point`, pulled in transitively) — used by
  `localZoneFromPosition`.
