#pragma once

#include <memory>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <clustering/node.hpp>

namespace clustering
{

/// Tree rooted at the robot (set root.pos to the robot position before each update); starts empty and follows the (position, id) points it is given.
/// Owns every node in it.
class Tree {
public:
    Tree(Vec3 root_pos, const Params& params);

    // nodes point at each other, so the tree must stay where it is
    Tree(const Tree&) = delete;
    Tree& operator=(const Tree&) = delete;

    const Params params;    // declared before root: nodes point at it
    Node root;      // root.pos is the robot position

    /// Add the points whose id is not in the tree yet, remove the tree nodes that are missing
    /// from points, then adapt radii, release far children and merge close neighbours.
    ///
    /// Additions and removals must happen closer than 2*root_radius to the root. Because of
    /// that only the tree nodes within that distance are checked for removal, so points
    /// may list either everything known or just what is within 2*root_radius.
    void update(const std::vector<std::tuple<Vec3, int>>& points);

    /// Whether a point with this id is currently in the tree.
    bool contains(int id) const;

private:
    struct Entry {
        std::unique_ptr<Node> node;
        unsigned long long seen = 0;    // epoch_ of the last update that listed this id
    };

    std::unordered_map<int, Entry> nodes_;
    unsigned long long epoch_ = 0;
};

}  // namespace clustering
