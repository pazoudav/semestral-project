#include <clustering/tree.hpp>

namespace clustering
{

Tree::Tree(Vec3 root_pos, const Params& params) : params(params), root(root_pos, this->params) {
    root.is_root = true;
}

bool Tree::contains(int id) const {
    return nodes_.count(id) > 0;
}

void Tree::update(const std::vector<std::tuple<Vec3, int>>& points) {
    ++epoch_;
    for (const auto& [pos, id] : points) {
        auto [it, inserted] = nodes_.try_emplace(id);
        Entry& entry = it->second;
        if (inserted) {
            entry.node = std::make_unique<Node>(pos, params, id);
            root.add_child(entry.node.get());
        }
        entry.seen = epoch_;
    }

    // additions and removals happen closer than 2*root_radius
    const double change_radius = 2 * params.root_radius;
    const double change_sq     = change_radius * change_radius;
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        Node* node = it->second.node.get();
        if (it->second.seen != epoch_ && norm_sq(node->pos - root.pos) < change_sq) {
            node->detach();
            it = nodes_.erase(it);
        } else {
            ++it;
        }
    }

    // radii first, for every branch, so merging and releasing see up-to-date radii
    for (Node* child : root.children) {
        child->double_radii(root.pos);
        child->halve(root.pos);
    }

    std::vector<Node*> snapshot = root.children;    // copy: merging re-parents root children
    for (Node* child : snapshot) {
        if (child->parent != &root) {               // was merged under another node this pass
            continue;
        }
        child->release_far_children();
        child->absorb_neighbours();
    }
}

}  // namespace clustering
