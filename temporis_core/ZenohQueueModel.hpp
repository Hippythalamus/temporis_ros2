#pragma once

#include <vector>
#include <random>
#include <LatencyModel.hpp>

/**
 * ZenohQueueModel — two-stage queue model for Zenoh client+router topology.
 *
 * Models the message path: client egress → router forwarding → subscriber.
 * See docs/zenoh_queue_model.md for full specification.
 *
 * Stage 1 (client egress): shared sender-side queue, one per client.
 *   Same physics as QueueLatencyModel — all outgoing messages from one
 *   client share a single upstream channel to the router.
 *
 * Stage 2 (router forwarding): single shared queue for ALL clients.
 *   Service time per message = router_base_cost + router_per_sub_cost * (N-1).
 *   This captures the sequential per-subscriber forwarding loop in zenoh-router.
 *   The router is the main bottleneck at scale (work grows as O(N^2)).
 *
 * Backward compatible: when router costs are zero, degenerates to
 * QueueLatencyModel behavior (plus extra propagation hop).
 */
class ZenohQueueModel : public LatencyModel {
public:
    struct Config {
        // Client egress parameters
        double client_bandwidth;            // bytes/sec, client upstream to router
        double packet_size;                 // bytes per message

        // Propagation delays (two hops)
        double propagation_client_router;   // seconds
        double propagation_router_subscriber; // seconds

        // Router parameters
        double router_base_cost;            // seconds per message (fixed overhead)
        double router_per_sub_cost;         // seconds per subscriber per message

        int num_agents;

        // Client bandwidth AR(1) noise (optional, default 0 = deterministic)
        double client_bandwidth_logstd = 0.0;
        double client_bandwidth_rho    = 0.0;

        uint64_t seed = 42;
    };

    explicit ZenohQueueModel(const Config& config);

    double sample(int sender, int receiver, double t,
                  int network_load, int queue_size) override;

    double compute_delay(int sender, int receiver, double t_arrival);

    void reset();

private:
    static constexpr double TIE_BREAK_EPS = 1e-9;

    // --- Client egress state (one per sender) ---
    struct ClientState {
        double t_complete = 0.0;
        double bw_log_state = 0.0;
        double last_step_time = -1.0;
        double current_bandwidth = 0.0;
    };
    std::vector<ClientState> clients_;

    // --- Router state (single shared queue) ---
    double router_t_complete_ = 0.0;

    // --- Config ---
    double client_bandwidth_mean_;
    double packet_size_;
    double prop_client_router_;
    double prop_router_sub_;
    double router_base_cost_;
    double router_per_sub_cost_;
    double router_service_time_;  // precomputed: base + per_sub * (N-1)
    int    num_agents_;

    // Client bandwidth noise
    double client_bw_logstd_;
    double client_bw_rho_;
    bool   client_stochastic_;
    double client_min_bandwidth_;

    std::mt19937_64 rng_;
    int floor_trigger_count_ = 0;

    void maybe_update_client_bandwidth(ClientState& c, double t_arrival);
};
