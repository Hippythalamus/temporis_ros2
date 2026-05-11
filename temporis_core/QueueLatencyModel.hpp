#pragma once

#include <vector>
#include <random>
#include <LatencyModel.hpp>

/**
 * QueueLatencyModel — mechanistic latency model based on shared sender-side
 * queues with optional bandwidth AR(1) noise.
 *
 * Each sender has ONE outgoing queue shared across all destinations.
 * Messages are served FIFO with service_time = packet_size / bandwidth(t).
 *
 * Step 1 (bandwidth_logstd = 0): deterministic M/D/1, validated against
 * batch-arrival analytical formula (0.00% error).
 *
 * Step 2 (bandwidth_logstd > 0): bandwidth fluctuates as a log-normal
 * AR(1) process per sender, creating stochastic service times and
 * temporal correlation in the latency trace. See docs/queue_model.md §5.
 *
 * Bandwidth sampling discipline: bandwidth is sampled ONCE per message
 * at the moment service begins (not integrated over service duration).
 * See docs/queue_model.md §5 "Sampling discipline".
 *
 * Bandwidth AR(1) evolves per SIMULATION STEP (not per message).
 * Within one step, all messages from a sender see the same bandwidth.
 * See docs/queue_model.md §5 "Note on time scale".
 *
 * Tie-breaking: t_real = t_arrival + sender * TIE_BREAK_EPS (1e-9 sec).
 * Purely for reproducible queue ordering, not physical time.
 * See docs/queue_model.md §4.
 */
class QueueLatencyModel : public LatencyModel {
public:
    struct Config {
        // Physical parameters
        double bandwidth;          // mean bandwidth, bytes/sec
        double propagation_delay;  // seconds
        double packet_size;        // bytes per message
        int    num_agents;

        // Bandwidth AR(1) noise (Step 2). Both default to 0 = deterministic.
        double bandwidth_logstd = 0.0;  // log-space std of bandwidth fluctuation
        double bandwidth_rho    = 0.0;  // lag-1 autocorrelation of bandwidth AR(1)

        // Seed for RNG (only used when bandwidth_logstd > 0)
        uint64_t seed = 42;
    };

    explicit QueueLatencyModel(const Config& config);

    double sample(int sender, int receiver, double t,
                  int network_load, int queue_size) override;

    double compute_delay(int sender, int receiver, double t_arrival);

    void reset();

private:
    static constexpr double TIE_BREAK_EPS = 1e-9;

    struct SenderState {
        double t_complete = 0.0;       // when queue is next free
        double bw_log_state = 0.0;     // AR(1) state in log-space
        double last_step_time = -1.0;  // tracks which step we're in
        double current_bandwidth = 0.0; // sampled bandwidth for this step
    };

    std::vector<SenderState> senders_;

    // Config values
    double bandwidth_mean_;
    double propagation_delay_;
    double packet_size_;
    double bandwidth_logstd_;
    double bandwidth_rho_;
    bool   stochastic_;  // true if bandwidth_logstd > 0

    // Floor to prevent bandwidth from collapsing to near-zero
    // (see docs/queue_model.md §8)
    double min_bandwidth_;

    // RNG (shared across all senders for simplicity)
    std::mt19937_64 rng_;

    // Diagnostics: how many times min_bandwidth floor was triggered
    int floor_trigger_count_ = 0;

    // Advance AR(1) state for a sender if we've entered a new step
    void maybe_update_bandwidth(SenderState& s, double t_arrival);
};