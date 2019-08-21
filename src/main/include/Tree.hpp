// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

namespace frc3512 {

/**
 * Implementation of a tree abstract data type.
 */
template <typename T>
class Tree {
public:
    T data;
    std::vector<Tree<T>> children;

    template <typename... Args>
    constexpr Tree(Args&&... args) : data(std::forward<Args>(args)...) {}

    /**
     * Visit children in a depth-first search fashion.
     *
     * @param visitor A function taking the current node and returning true if
     *                the search should continue.
     */
    void DFS(std::function<bool(T&)> visitor) {
        if (visitor(data)) {
            for (auto& child : children) {
                child.DFS(visitor);
            }
        }
    }

    /**
     * Returns the number of nodes in the tree.
     */
    size_t Size() const {
        size_t count = 1;

        // Recursively add the count below each node that is attached to the
        // current one
        for (const auto& child : children) {
            count += child->size();
        }

        return count;
    }
};

}  // namespace frc3512
