#include "QueueLatencyModel.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>
#include <iostream>

QueueLatencyModel::QueueLatencyModel(const Config& config)
    : senders_(),
      bandwidth_mean_(config.bandwidth),
      propagation_delay_(config.propagation_delay),
      packet_size_(config.packet_size),
      bandwidth_logstd_(config.bandwidth_logstd),
      bandwidth_rho_(config.bandwidth_rho),
      stochastic_(config.bandwidth_logstd > 1e-15),
      min_bandwidth_(config.bandwidth / 100.0),
      rng_(config.seed),
      floor_trigger_count_(0)
{
    if (config.bandwidth <= 0.0)
        throw std::invalid_argument(
            "QueueLatencyModel: bandwidth must be > 0 (got " +
            std::to_string(config.bandwidth) + ")");
    if (config.packet_size <= 0.0)
        throw std::invalid_argument(
            "QueueLatencyModel: packet_size must be > 0 (got " +
            std::to_string(config.packet_size) + ")");
    if (config.propagation_delay < 0.0)
        throw std::invalid_argument(
            "QueueLatencyModel: propagation_delay must be >= 0 (got " +
            std::to_string(config.propagation_delay) + ")");
    if (config.num_agents <= 0)
        throw std::invalid_argument(
            "QueueLatencyModel: num_agents must be > 0 (got " +
            std::to_string(config.num_agents) + ")");
    if (config.bandwidth_rho < -1.0 || config.bandwidth_rho > 1.0)
        throw std::invalid_argument(
            "QueueLatencyModel: bandwidth_rho must be in [-1, 1] (got " +
            std::to_string(config.bandwidth_rho) + ")");

    // Initialize sender states
    SenderState init{};
    init.t_complete = 0.0;
    init.bw_log_state = 0.0;  // stationary mean in log-space = 0
    init.last_step_time = -1.0;
    init.current_bandwidth = config.bandwidth;
    senders_.assign(config.num_agents, init);
}

void QueueLatencyModel::maybe_update_bandwidth(SenderState& s,
                                                double t_arrival)
{
    if (!stochastic_) return;

    // Bandwidth AR(1) evolves once per simulation step, not per message.
    // We detect "new step" by checking if t_arrival moved to a new integer
    // second (since dt >= 1.0 in our setup). This avoids coupling the model
    // to a specific dt value — any t_arrival that differs from last_step_time
    // by more than 0.5 seconds triggers an update.
    //
    // Within one step, all messages from this sender see the same bandwidth.
    if (std::abs(t_arrival - s.last_step_time) < 0.5) return;

    s.last_step_time = t_arrival;

    // AR(1) in log-space:
    //   epsilon_t = rho * epsilon_{t-1} + innovation
    //   bandwidth_t = bandwidth_mean * exp(epsilon_t)
    //
    // Stationary distribution: epsilon ~ N(0, logstd^2 / (1 - rho^2))
    // We store epsilon as bw_log_state.
    std::normal_distribution<double> normal(0.0, 1.0);
    double innovation = bandwidth_logstd_ * std::sqrt(1.0 - bandwidth_rho_ * bandwidth_rho_);
    s.bw_log_state = bandwidth_rho_ * s.bw_log_state + innovation * normal(rng_);

    double bw = bandwidth_mean_ * std::exp(s.bw_log_state);

    // Floor to prevent near-zero bandwidth (see queue_model.md §8)
    if (bw < min_bandwidth_) {
        bw = min_bandwidth_;
        floor_trigger_count_++;
    }

    s.current_bandwidth = bw;
}

double QueueLatencyModel::compute_delay(int sender, int receiver,
                                        double t_arrival)
{
    assert(sender   >= 0 && sender   < static_cast<int>(senders_.size()));
    assert(receiver >= 0 && receiver < static_cast<int>(senders_.size()));
    assert(sender != receiver);

    const double t_real = t_arrival + sender * TIE_BREAK_EPS;
    //assert(sender == 0 || t_real != t_arrival);

    SenderState& sq = senders_[sender];

    // Update bandwidth if we've entered a new simulation step
    maybe_update_bandwidth(sq, t_arrival);

    // Service time uses THIS step's sampled bandwidth
    double service_time = packet_size_ / sq.current_bandwidth;

    // M/D/1 (or M/G/1 with stochastic service) recursion
    const double t_start    = std::max(t_real, sq.t_complete);
    const double t_complete = t_start + service_time;
    sq.t_complete = t_complete;

    return (t_complete - t_real) + propagation_delay_;
}

double QueueLatencyModel::sample(int sender, int receiver, double t,
                                  int /*network_load*/, int /*queue_size*/)
{
    return compute_delay(sender, receiver, t);
}

void QueueLatencyModel::reset()
{
    for (auto& sq : senders_) {
        sq.t_complete = 0.0;
        sq.bw_log_state = 0.0;
        sq.last_step_time = -1.0;
        sq.current_bandwidth = bandwidth_mean_;
    }
    floor_trigger_count_ = 0;
}