#pragma once

#include <array>
#include <cmath>
#include <vector>

namespace clustering
{

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, double k) { return {a.x * k, a.y * k, a.z * k}; }
inline Vec3 operator/(Vec3 a, double k) { return {a.x / k, a.y / k, a.z / k}; }
inline double norm_sq(Vec3 v) { return v.x * v.x + v.y * v.y + v.z * v.z; }
inline double norm(Vec3 v) { return std::sqrt(norm_sq(v)); }

/// Radii of the tree, loaded from config/clustering.yaml. Every radius is scaled by
/// growth_factor**scale of the node it is read from.
struct Params {
    /// growth[scale] = growth_factor**scale for every reachable scale
    static constexpr int MAX_SCALE = 32;

    double growth_factor  = 1.0;
    double expand_radius  = 1.0;    // a node grows its radii while the robot is farther than this
    double deflate_radius = 1.0;    // and shrinks them while the robot is closer than this
    double break_radius   = 1.0;    // children farther than this are handed back to the parent
    double build_radius   = 1.0;    // siblings closer than this are merged
    double root_radius    = 1.0;    // additions and removals happen closer than 2*root_radius

    std::array<double, MAX_SCALE + 1> growth = {};

    /// Recompute the growth table; call after changing growth_factor.
    void update_growth() {
        double f = 1.0;
        for (double& slot : growth) {
            slot = f;
            f *= growth_factor;
        }
    }
};

/// A point in the tree. Edges are the links between a node and its children.
/// Nodes do not own each other; the owning Tree keeps them alive.
class Node {
public:
    Node(Vec3 pos, const Params& params, int id = -1);

    Vec3 pos;
    const Params* params;   // owned by the Tree, outlives the node
    int id;                 // identifies the point across updates; -1 for the root
    Node* parent = nullptr;
    std::vector<Node*> children;
    int scale = 0;          // radius = base radius * growth_factor**scale; 0 gives the base radii
    int weight = 1;         // subtree size
    bool is_root = false;

    void add_child(Node* node);
    void remove_child(Node* node);

    /// Leave the parent, which adopts this node's children. Must not be the root.
    void detach();

    /// The child of the root this node hangs under (itself if it is one).
    Node* branch();

    // the four radii always scale together, so they are derived from the integer step count
    double expand_radius() const;
    double deflate_radius() const;
    double break_radius() const;
    double build_radius() const;

    /// Grow the radii until they fit the distance to the robot.
    void double_radii(Vec3 parent_pos);
    /// Shrink the radii until they fit the distance to the robot (never below the base radii).
    void halve(Vec3 parent_pos);

    /// Hand children that are beyond break_radius back to this node's parent.
    void release_far_children();
    /// Merge the siblings within build_radius (and this node) under the best-placed one.
    void absorb_neighbours();
    /// Make the node closest to the group's weighted mean the parent of the others in group.
    /// Drops the nodes the head would release again right away from group.
    void merge(std::vector<Node*>& group);

private:
    /// weight is the subtree size, so a change here applies to every ancestor too.
    void add_weight(int delta);
};

}  // namespace clustering
