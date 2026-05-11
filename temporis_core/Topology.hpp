#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <random>
#include <algorithm>
#include <stdexcept>
#include <set>

/**
 * Topology — defines which agents communicate with which.
 *
 * Used by consensus_demo to determine the send loop: each agent sends
 * messages only to its neighbors() instead of to all others.
 *
 * Four implementations:
 *   AllToAllTopology  — every agent talks to every other (N×(N-1) messages)
 *   RingTopology      — each agent talks to left and right neighbor (2N messages)
 *   GridTopology      — agents on a 2D grid, 4-connected (up/down/left/right)
 *   RandomKTopology   — each agent has K random neighbors (symmetric: if A→B then B→A)
 *
 * All topologies are **symmetric**: if A is a neighbor of B, then B is a
 * neighbor of A. This is important for consensus convergence — asymmetric
 * communication can prevent convergence entirely.
 */
class Topology {
public:
    virtual ~Topology() = default;

    /// List of neighbor agent ids for the given agent.
    virtual const std::vector<int>& neighbors(int agent_id) const = 0;

    /// Number of neighbors (degree) for the given agent.
    virtual int degree(int agent_id) const = 0;

    /// Human-readable name for logging.
    virtual std::string name() const = 0;

    /// Total number of directed messages per step (sum of all degrees).
    virtual int total_messages_per_step() const = 0;
};


/**
 * AllToAllTopology — every agent communicates with every other.
 * Degree = N-1 for all agents. Total messages = N×(N-1).
 */
class AllToAllTopology : public Topology {
public:
    explicit AllToAllTopology(int num_agents)
        : adj_(num_agents)
    {
        for (int i = 0; i < num_agents; ++i) {
            adj_[i].reserve(num_agents - 1);
            for (int j = 0; j < num_agents; ++j) {
                if (i != j) adj_[i].push_back(j);
            }
        }
    }

    const std::vector<int>& neighbors(int id) const override {
        return adj_[id];
    }
    int degree(int id) const override {
        return static_cast<int>(adj_[id].size());
    }
    std::string name() const override { return "all_to_all"; }
    int total_messages_per_step() const override {
        int n = static_cast<int>(adj_.size());
        return n * (n - 1);
    }

private:
    std::vector<std::vector<int>> adj_;
};


/**
 * RingTopology — each agent talks to its left and right neighbor.
 * Agent i's neighbors: (i-1+N)%N and (i+1)%N.
 * Degree = 2 for all agents. Total messages = 2N.
 *
 * Information propagates sequentially around the ring: the farthest
 * agent is N/2 hops away, so consensus is slower than all-to-all but
 * network load is O(N) instead of O(N²).
 */
class RingTopology : public Topology {
public:
    explicit RingTopology(int num_agents)
        : adj_(num_agents)
    {
        if (num_agents < 3)
            throw std::invalid_argument(
                "RingTopology requires at least 3 agents");

        for (int i = 0; i < num_agents; ++i) {
            int left  = (i - 1 + num_agents) % num_agents;
            int right = (i + 1) % num_agents;
            adj_[i] = {left, right};
        }
    }

    const std::vector<int>& neighbors(int id) const override {
        return adj_[id];
    }
    int degree(int id) const override {
        return static_cast<int>(adj_[id].size());
    }
    std::string name() const override { return "ring"; }
    int total_messages_per_step() const override {
        return 2 * static_cast<int>(adj_.size());
    }

private:
    std::vector<std::vector<int>> adj_;
};


/**
 * GridTopology — agents on a 2D grid with 4-connectivity.
 *
 * Grid dimensions: rows = ceil(sqrt(N)), cols = ceil(N/rows).
 * Agent i sits at row = i/cols, col = i%cols.
 * Neighbors: up, down, left, right (if they exist and id < N).
 *
 * Boundary agents have 2-3 neighbors, interior agents have 4.
 * Average degree ≈ 4 - 4/sqrt(N).
 *
 * Information propagates in O(sqrt(N)) hops diagonally.
 */
class GridTopology : public Topology {
public:
    explicit GridTopology(int num_agents)
        : adj_(num_agents), rows_(0), cols_(0)
    {
        if (num_agents < 4)
            throw std::invalid_argument(
                "GridTopology requires at least 4 agents");

        rows_ = static_cast<int>(std::ceil(std::sqrt(
                    static_cast<double>(num_agents))));
        cols_ = (num_agents + rows_ - 1) / rows_;

        for (int i = 0; i < num_agents; ++i) {
            int r = i / cols_;
            int c = i % cols_;

            auto try_add = [&](int nr, int nc) {
                if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                    int nid = nr * cols_ + nc;
                    if (nid < num_agents && nid != i) {
                        adj_[i].push_back(nid);
                    }
                }
            };

            try_add(r - 1, c);  // up
            try_add(r + 1, c);  // down
            try_add(r, c - 1);  // left
            try_add(r, c + 1);  // right
        }
    }

    const std::vector<int>& neighbors(int id) const override {
        return adj_[id];
    }
    int degree(int id) const override {
        return static_cast<int>(adj_[id].size());
    }
    std::string name() const override {
        return "grid_" + std::to_string(rows_) + "x" + std::to_string(cols_);
    }
    int total_messages_per_step() const override {
        int total = 0;
        for (const auto& v : adj_) total += static_cast<int>(v.size());
        return total;
    }

private:
    std::vector<std::vector<int>> adj_;
    int rows_, cols_;
};


/**
 * RandomKTopology — each agent has K random neighbors.
 *
 * The graph is **symmetric**: if A is chosen as B's neighbor, then B
 * is also added to A's neighbor list. Final degree per agent is
 * approximately K but may vary slightly due to symmetrization.
 *
 * The graph is **connected**: after random selection, connectivity is
 * verified and additional edges are added if needed to ensure a single
 * connected component (via BFS). This guarantees consensus can converge.
 *
 * Total messages ≈ K×N (slightly more due to symmetrization).
 */
class RandomKTopology : public Topology {
public:
    RandomKTopology(int num_agents, int k, uint64_t seed = 42)
        : adj_(num_agents)
    {
        if (num_agents < 3)
            throw std::invalid_argument(
                "RandomKTopology requires at least 3 agents");
        if (k < 1)
            throw std::invalid_argument(
                "RandomKTopology requires k >= 1");
        if (k >= num_agents)
            throw std::invalid_argument(
                "RandomKTopology requires k < num_agents "
                "(use AllToAllTopology for k = num_agents - 1)");

        int N = num_agents;
        std::mt19937_64 rng(seed);

        // Step 1: for each agent, pick K random neighbors
        std::vector<std::set<int>> neighbor_sets(N);

        for (int i = 0; i < N; ++i) {
            // Build candidate list (all agents except self and existing neighbors)
            while (static_cast<int>(neighbor_sets[i].size()) < k) {
                int j = std::uniform_int_distribution<int>(0, N - 1)(rng);
                if (j == i) continue;
                if (neighbor_sets[i].count(j)) continue;
                // Symmetric: add both directions
                neighbor_sets[i].insert(j);
                neighbor_sets[j].insert(i);
            }
        }

        // Step 2: ensure connectivity via BFS
        ensure_connected(neighbor_sets, N, rng);

        // Step 3: convert sets to vectors
        for (int i = 0; i < N; ++i) {
            adj_[i].assign(neighbor_sets[i].begin(),
                           neighbor_sets[i].end());
            std::sort(adj_[i].begin(), adj_[i].end());
        }
    }

    const std::vector<int>& neighbors(int id) const override {
        return adj_[id];
    }
    int degree(int id) const override {
        return static_cast<int>(adj_[id].size());
    }
    std::string name() const override { return "random_k"; }
    int total_messages_per_step() const override {
        int total = 0;
        for (const auto& v : adj_) total += static_cast<int>(v.size());
        return total;
    }

private:
    std::vector<std::vector<int>> adj_;

    static void ensure_connected(std::vector<std::set<int>>& sets,
                                  int N, std::mt19937_64& /*rng*/) {
        // BFS to find connected components
        std::vector<int> component(N, -1);
        int num_components = 0;

        for (int start = 0; start < N; ++start) {
            if (component[start] >= 0) continue;

            // BFS from start
            std::vector<int> queue = {start};
            component[start] = num_components;
            int head = 0;

            while (head < static_cast<int>(queue.size())) {
                int cur = queue[head++];
                for (int nb : sets[cur]) {
                    if (component[nb] < 0) {
                        component[nb] = num_components;
                        queue.push_back(nb);
                    }
                }
            }
            num_components++;
        }

        // Connect components by adding edges between them
        if (num_components <= 1) return;

        // For each component pair, add one bridge edge
        for (int c = 1; c < num_components; ++c) {
            // Find one node in component 0 and one in component c
            int from = -1, to = -1;
            for (int i = 0; i < N && (from < 0 || to < 0); ++i) {
                if (component[i] == 0 && from < 0) from = i;
                if (component[i] == c && to < 0) to = i;
            }
            if (from >= 0 && to >= 0) {
                sets[from].insert(to);
                sets[to].insert(from);
                // Merge: relabel component c as 0
                for (int i = 0; i < N; ++i) {
                    if (component[i] == c) component[i] = 0;
                }
            }
        }
    }
};


// ---- Factory ----

#include <memory>

/**
 * Create a Topology from config parameters.
 *
 * @param name        "all_to_all", "ring", "grid", or "random_k"
 * @param num_agents  total number of agents
 * @param k           number of neighbors for random_k (ignored for others)
 * @param seed        RNG seed for random_k (ignored for others)
 */
inline std::unique_ptr<Topology>
make_topology(const std::string& name, int num_agents,
              int k = 5, uint64_t seed = 42)
{
    if (name == "all_to_all")
        return std::make_unique<AllToAllTopology>(num_agents);
    if (name == "ring")
        return std::make_unique<RingTopology>(num_agents);
    if (name == "grid")
        return std::make_unique<GridTopology>(num_agents);
    if (name == "random_k")
        return std::make_unique<RandomKTopology>(num_agents, k, seed);

    throw std::invalid_argument("Unknown topology: " + name);
}
