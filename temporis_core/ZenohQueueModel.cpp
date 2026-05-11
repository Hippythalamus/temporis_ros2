#include "ZenohQueueModel.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>

ZenohQueueModel::ZenohQueueModel(const Config& config)
    : clients_(),
      router_t_complete_(0.0),
      client_bandwidth_mean_(config.client_bandwidth),
      packet_size_(config.packet_size),
      prop_client_router_(config.propagation_client_router),
      prop_router_sub_(config.propagation_router_subscriber),
      router_base_cost_(config.router_base_cost),
      router_per_sub_cost_(config.router_per_sub_cost),
      router_service_time_(0.0),
      num_agents_(config.num_agents),
      client_bw_logstd_(config.client_bandwidth_logstd),
      client_bw_rho_(config.client_bandwidth_rho),
      client_stochastic_(config.client_bandwidth_logstd > 1e-15),
      client_min_bandwidth_(config.client_bandwidth / 100.0),
      rng_(config.seed),
      floor_trigger_count_(0)
{
    // Validate
    if (config.client_bandwidth <= 0.0)
        throw std::invalid_argument(
            "ZenohQueueModel: client_bandwidth must be > 0 (got " +
            std::to_string(config.client_bandwidth) + ")");
    if (config.packet_size <= 0.0)
        throw std::invalid_argument(
            "ZenohQueueModel: packet_size must be > 0 (got " +
            std::to_string(config.packet_size) + ")");
    if (config.propagation_client_router < 0.0)
        throw std::invalid_argument(
            "ZenohQueueModel: propagation_client_router must be >= 0");
    if (config.propagation_router_subscriber < 0.0)
        throw std::invalid_argument(
            "ZenohQueueModel: propagation_router_subscriber must be >= 0");
    if (config.router_base_cost < 0.0)
        throw std::invalid_argument(
            "ZenohQueueModel: router_base_cost must be >= 0");
    if (config.router_per_sub_cost < 0.0)
        throw std::invalid_argument(
            "ZenohQueueModel: router_per_sub_cost must be >= 0");
    if (config.num_agents <= 0)
        throw std::invalid_argument(
            "ZenohQueueModel: num_agents must be > 0");

    // Precompute router service time for all-to-all consensus:
    // each message goes to N-1 subscribers
    int fan_out = config.num_agents - 1;
    router_service_time_ = config.router_base_cost
                         + config.router_per_sub_cost * fan_out;

    // Initialize client states
    ClientState init{};
    init.current_bandwidth = config.client_bandwidth;
    clients_.assign(config.num_agents, init);
}

void ZenohQueueModel::maybe_update_client_bandwidth(ClientState& c,
                                                     double t_arrival)
{
    if (!client_stochastic_) return;
    if (std::abs(t_arrival - c.last_step_time) < 0.5) return;

    c.last_step_time = t_arrival;

    std::normal_distribution<double> normal(0.0, 1.0);
    double innovation = client_bw_logstd_
                      * std::sqrt(1.0 - client_bw_rho_ * client_bw_rho_);
    c.bw_log_state = client_bw_rho_ * c.bw_log_state
                   + innovation * normal(rng_);

    double bw = client_bandwidth_mean_ * std::exp(c.bw_log_state);

    if (bw < client_min_bandwidth_) {
        bw = client_min_bandwidth_;
        floor_trigger_count_++;
    }

    c.current_bandwidth = bw;
}

double ZenohQueueModel::compute_delay(int sender, int receiver,
                                       double t_arrival)
{
    assert(sender   >= 0 && sender   < num_agents_);
    assert(receiver >= 0 && receiver < num_agents_);
    assert(sender != receiver);

    // ---- Stage 1: client egress queue ----
    const double t_real = t_arrival + sender * TIE_BREAK_EPS;
    assert(sender == 0 || t_real != t_arrival);

    ClientState& client = clients_[sender];
    maybe_update_client_bandwidth(client, t_arrival);

    double client_service_time = packet_size_ / client.current_bandwidth;
    double t_start_client = std::max(t_real, client.t_complete);
    double t_done_client  = t_start_client + client_service_time;
    client.t_complete = t_done_client;

    // ---- Propagation: client → router ----
    double t_arrive_router = t_done_client + prop_client_router_;

    // ---- Stage 2: router forwarding queue ----
    // Secondary tie-break at router by sender id, for the rare case
    // where two clients finish egress at exactly the same time.
    double t_arrive_router_real = t_arrive_router + sender * TIE_BREAK_EPS;

    double t_start_router = std::max(t_arrive_router_real, router_t_complete_);
    double t_done_router  = t_start_router + router_service_time_;
    router_t_complete_ = t_done_router;

    // ---- Propagation: router → subscriber ----
    double t_arrive_subscriber = t_done_router + prop_router_sub_;

    // ---- Total delay from send time ----
    return t_arrive_subscriber - t_real;
}

double ZenohQueueModel::sample(int sender, int receiver, double t,
                                int /*network_load*/, int /*queue_size*/)
{
    return compute_delay(sender, receiver, t);
}

void ZenohQueueModel::reset()
{
    for (auto& c : clients_) {
        c.t_complete = 0.0;
        c.bw_log_state = 0.0;
        c.last_step_time = -1.0;
        c.current_bandwidth = client_bandwidth_mean_;
    }
    router_t_complete_ = 0.0;
    floor_trigger_count_ = 0;
}
