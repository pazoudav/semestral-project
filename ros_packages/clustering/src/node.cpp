#include <clustering/node.hpp>

#include <algorithm>
#include <cmath>

namespace clustering
{

namespace {

double square(double x) { return x * x; }

/// Reused between calls so grouping neighbours does not allocate every time.
thread_local std::vector<Node*> group_scratch;

}  // namespace

Node::Node(Vec3 pos, const Params& params, int id) : pos(pos), params(&params), id(id) {}

void Node::add_child(Node* node) {
    node->parent = this;
    children.push_back(node);

    add_weight(node->weight);
}

void Node::remove_child(Node* node) {
    children.erase(std::find(children.begin(), children.end(), node));
    node->parent = nullptr;
    add_weight(-node->weight);
}

void Node::detach() {
    // The children stay in the same subtree of the parent, so the ancestors' weights
    // only change by this node itself.
    Node* old_parent = parent;
    for (Node* child : children) {
        child->parent = old_parent;
        old_parent->children.push_back(child);
    }
    children.clear();
    weight = 1;
    old_parent->remove_child(this);
}

void Node::add_weight(int delta) {
    for (Node* ancestor = this; ancestor != nullptr; ancestor = ancestor->parent) {
        ancestor->weight += delta;
    }
}

Node* Node::branch() {
    Node* node = this;
    while (node->parent != nullptr && !node->parent->is_root) {
        node = node->parent;
    }
    return node;
}

double Node::expand_radius() const { return params->expand_radius * params->growth[scale]; }
double Node::deflate_radius() const { return params->deflate_radius * params->growth[scale]; }
double Node::break_radius() const { return params->break_radius * params->growth[scale]; }
double Node::build_radius() const { return params->build_radius * params->growth[scale]; }

void Node::double_radii(Vec3 parent_pos) {
    double d2 = norm_sq(parent_pos - pos);
    while (scale < Params::MAX_SCALE && d2 > square(expand_radius())) {
        scale += 1;
    }
}

void Node::halve(Vec3 parent_pos) {
    double d2 = norm_sq(parent_pos - pos);
    while (scale > 0 && d2 < square(deflate_radius())) {
        scale -= 1;
    }
}

void Node::release_far_children() {
    // Same as remove_child + parent->add_child per child, but done in one pass over
    // children: the weight only changes for this node, since parent and above lose
    // and regain the same amount.
    const double break_sq = square(break_radius());
    size_t kept = 0;
    for (Node* child : children) {
        if (norm_sq(child->pos - pos) > break_sq) {
            child->parent = parent;
            parent->children.push_back(child);
            weight -= child->weight;
        } else {
            children[kept++] = child;
        }
    }
    children.erase(children.begin() + kept, children.end());
}

void Node::absorb_neighbours() {
    const double build_sq = square(build_radius());
    std::vector<Node*>& group = group_scratch;
    group.clear();
    group.push_back(this);
    for (Node* sibling : parent->children) {
        if (sibling != this && norm_sq(pos - sibling->pos) < build_sq) {
            group.push_back(sibling);
        }
    }
    if (group.size() > 1) {
        merge(group);
    }
}

void Node::merge(std::vector<Node*>& group) {
    int total = 0;
    Vec3 weighted_sum;
    for (Node* n : group) {
        total += n->weight;
        weighted_sum = weighted_sum + n->pos * n->weight;
    }
    Vec3 weight_mean = weighted_sum / total;

    Node* head = group.front();
    double best = norm_sq(head->pos - weight_mean);
    for (Node* n : group) {
        double d2 = norm_sq(n->pos - weight_mean);
        if (d2 < best) {
            head = n;
            best = d2;
        }
    }

    // only take nodes the head would not release again right away
    const double break_sq = square(head->break_radius());
    group.erase(std::remove_if(group.begin(), group.end(),
                               [&](Node* n) { return n != head && norm_sq(n->pos - head->pos) > break_sq; }),
                group.end());
    if (group.size() == 1) {
        return;
    }
    Node* parent = head->parent;

    for (Node* node : group) {
        parent->remove_child(node);
    }
    for (Node* node : group) {
        if (node != head) {
            head->add_child(node);
        }
    }
    parent->add_child(head);    // last, so the head's weight already includes the others
}

}  // namespace clustering
