/// @file temporis_per_agent_shaper_node.cpp
/// @brief Step 7: Per-agent shaper.
///
/// Architecture (Step 7):
///   Single process. N independent per-agent client queues sharing one
///   router latency model. Each agent's messages flow through its own
///   ClientQueue (M/D/1 with bandwidth AR(1) + propagation_client_router)
///   and are then served by the shared router (router_base_cost +
///   router_per_sub_cost * subscribers), followed by propagation_router_subscriber.
///
///   This matches the Zenoh client-router-clients architecture that was
///   calibrated in ZenohQueueModel (108µs/2.7µs at N=10/20/50).
///
/// Why per-agent (vs Step 4-6 single shaper):
///   - Eliminates startup transient: a slow agent doesn't block others
///   - Late-joining agents don't see "queue catch-up" effect
///   - Each agent's queue load = its own publish rate (not aggregated NxN)
///
/// Topology and addressing unchanged from Step 5:
///   Discovery via GID → namespace ("/robot_3" → agent_id=3)
///   Topology determines fan_out for ZenohQueueModel

#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Temporis core
#include "QueueLatencyModel.hpp"
#include "ZenohQueueModel.hpp"
#include "Topology.hpp"

namespace temporis_ros2 {

using MsgType = std_msgs::msg::ByteMultiArray;

// ====================================================================
// DelayedMessage — same shape as Step 4-6, tagged with agent_id
// ====================================================================
struct DelayedMessage {
    MsgType::SharedPtr msg;
    rclcpp::Time publish_at;
    rclcpp::Time received_at;
    int sender_id;
    double delay_sec;
};

// ====================================================================
// PerAgentShaperNode
//
// One Node instance per logical role:
//   - Subscribes to /temporis/raw/states (one shared input topic)
//   - Has N independent client queues internally
//   - Publishes to /fleet/states (one shared output topic)
//
// The "per-agent" property is realized by:
//   1. Routing each incoming message to a queue indexed by sender_id
//   2. Each queue draining independently at 2kHz
//   3. Latency model called with (sender_id, receiver_id) — its internal
//      state already tracks per-client AR(1) bandwidth and per-client M/D/1
//
// Topic structure unchanged — only internal queue organization differs.
// ====================================================================
class PerAgentShaper : public rclcpp::Node {
public:
    PerAgentShaper()
        : Node("temporis_per_agent_shaper")
    {
        // ---- Parameters (same surface as Step 4-6) ----
        declare_parameter("input_topic", "/temporis/raw/states");
        declare_parameter("output_topic", "/fleet/states");
        declare_parameter("model", "ZENOH_QUEUE");
        declare_parameter("topology", "all_to_all");
        declare_parameter("topology_k", 5);
        declare_parameter("num_agents", 10);
        declare_parameter("enabled", true);
        declare_parameter("publish_diagnostics", true);
        declare_parameter("namespace_prefix", "robot_");

        // ZENOH_QUEUE parameters (calibrated 108µs/2.7µs)
        declare_parameter("client_bandwidth", 1000.0);
        declare_parameter("packet_size", 1.0);
        declare_parameter("propagation_client_router", 0.000150);
        declare_parameter("propagation_router_subscriber", 0.000150);
        declare_parameter("router_base_cost", 0.000108);
        declare_parameter("router_per_sub_cost", 0.0000027);

        // QUEUE parameters
        declare_parameter("bandwidth", 70.0);
        declare_parameter("propagation_delay", 0.05);

        // Noise
        declare_parameter("bandwidth_logstd", 0.0);
        declare_parameter("bandwidth_rho", 0.0);
        declare_parameter("seed", 42);

        // ---- Read core ----
        input_topic_ = get_parameter("input_topic").as_string();
        output_topic_ = get_parameter("output_topic").as_string();
        model_name_ = get_parameter("model").as_string();
        enabled_ = get_parameter("enabled").as_bool();
        publish_diag_ = get_parameter("publish_diagnostics").as_bool();
        num_agents_ = get_parameter("num_agents").as_int();
        ns_prefix_ = get_parameter("namespace_prefix").as_string();

        // Sanity
        if (num_agents_ <= 0 || num_agents_ > 1024) {
            throw std::runtime_error(
                "num_agents out of plausible range [1, 1024]");
        }

        // Parameter change callback (toggling 'enabled' at runtime)
        param_callback_handle_ =
            this->add_on_set_parameters_callback(
                std::bind(&PerAgentShaper::on_param_change,
                          this, std::placeholders::_1));

        // ---- Build latency model (shared router) ----
        build_latency_model();

        // ---- Build topology ----
        auto topo_name = get_parameter("topology").as_string();
        auto topo_k = get_parameter("topology_k").as_int();
        auto seed = static_cast<uint64_t>(get_parameter("seed").as_int());
        topology_ = make_topology(topo_name, num_agents_, topo_k, seed);

        // ---- Per-agent queues ----
        per_agent_queues_.resize(num_agents_);

        RCLCPP_INFO(get_logger(),
            "Per-agent shaper (Step 7): model=%s topology=%s agents=%d "
            "enabled=%s",
            model_name_.c_str(), topology_->name().c_str(),
            num_agents_, enabled_ ? "true" : "false");
        RCLCPP_INFO(get_logger(),
            "  %d independent client queues sharing one router model",
            num_agents_);
        RCLCPP_INFO(get_logger(),
            "  input: %s -> output: %s",
            input_topic_.c_str(), output_topic_.c_str());

        auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();

        publisher_ = create_publisher<MsgType>(output_topic_, qos);

        subscription_ = create_subscription<MsgType>(
            input_topic_, qos,
            [this](const MsgType::ConstSharedPtr msg,
                   const rclcpp::MessageInfo & info) {
                on_message(msg, info);
            });

        if (publish_diag_) {
            diag_pub_ = create_publisher<std_msgs::msg::String>(
                "/temporis/diagnostics", 10);
        }

        // Drain timer: 2kHz over all N queues
        drain_timer_ = create_wall_timer(
            std::chrono::microseconds(500),
            std::bind(&PerAgentShaper::drain_all_queues, this));

        // Discovery (1Hz)
        discovery_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PerAgentShaper::refresh_publisher_map, this));

        // Stats timer: every 5s, log queue depths
        stats_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&PerAgentShaper::log_queue_stats, this));
    }

private:
    // ================================================================
    // Latency model construction (shared router)
    // ================================================================
    void build_latency_model() {
        if (model_name_ == "ZENOH_QUEUE") {
            ZenohQueueModel::Config cfg{};
            cfg.client_bandwidth = get_parameter("client_bandwidth").as_double();
            cfg.packet_size = get_parameter("packet_size").as_double();
            cfg.propagation_client_router =
                get_parameter("propagation_client_router").as_double();
            cfg.propagation_router_subscriber =
                get_parameter("propagation_router_subscriber").as_double();
            cfg.router_base_cost = get_parameter("router_base_cost").as_double();
            cfg.router_per_sub_cost =
                get_parameter("router_per_sub_cost").as_double();
            cfg.num_agents = num_agents_;
            cfg.client_bandwidth_logstd =
                get_parameter("bandwidth_logstd").as_double();
            cfg.client_bandwidth_rho =
                get_parameter("bandwidth_rho").as_double();
            cfg.seed = static_cast<uint64_t>(get_parameter("seed").as_int());
            latency_model_ = std::make_unique<ZenohQueueModel>(cfg);
        } else if (model_name_ == "QUEUE") {
            QueueLatencyModel::Config cfg{};
            cfg.bandwidth = get_parameter("bandwidth").as_double();
            cfg.propagation_delay =
                get_parameter("propagation_delay").as_double();
            cfg.packet_size = get_parameter("packet_size").as_double();
            cfg.num_agents = num_agents_;
            cfg.bandwidth_logstd =
                get_parameter("bandwidth_logstd").as_double();
            cfg.bandwidth_rho =
                get_parameter("bandwidth_rho").as_double();
            cfg.seed = static_cast<uint64_t>(get_parameter("seed").as_int());
            latency_model_ = std::make_unique<QueueLatencyModel>(cfg);
        } else {
            throw std::runtime_error("Unknown Temporis model: " + model_name_);
        }
    }

    rcl_interfaces::msg::SetParametersResult
    on_param_change(const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & param : params) {
            if (param.get_name() == "enabled") {
                enabled_ = param.as_bool();
                RCLCPP_INFO(get_logger(),
                    "Per-agent shaper enabled=%s",
                    enabled_ ? "true" : "false");
            }
        }
        return result;
    }

    // ================================================================
    // Namespace-based discovery (same as Step 3-6)
    // ================================================================
    int parse_agent_id(const std::string & node_namespace) const {
        std::string ns = node_namespace;
        while (!ns.empty() && ns.front() == '/') ns.erase(ns.begin());

        auto pos = ns.rfind(ns_prefix_);
        if (pos == std::string::npos) return -1;

        std::string digits;
        for (size_t i = pos + ns_prefix_.size(); i < ns.size(); ++i) {
            if (std::isdigit(ns[i])) digits += ns[i];
            else break;
        }
        if (digits.empty()) return -1;

        int id = std::stoi(digits);
        if (id < 0 || id >= num_agents_) {
            RCLCPP_WARN_ONCE(get_logger(),
                "Agent id %d from '%s' out of range [0, %d), wrapping",
                id, node_namespace.c_str(), num_agents_);
            return id % num_agents_;
        }
        return id;
    }

    static uint64_t make_gid_key(const uint8_t * data) {
        uint64_t a = 0, b = 0;
        std::memcpy(&a, data, sizeof(a));
        std::memcpy(&b, data + 8, sizeof(b));
        return a ^ (b * 0x9e3779b97f4a7c15ULL);
    }

    void refresh_publisher_map() {
        auto pubs_info = get_publishers_info_by_topic(input_topic_);
        for (const auto & info : pubs_info) {
            uint64_t gid_key = make_gid_key(info.endpoint_gid().data());
            if (gid_to_agent_.count(gid_key)) continue;

            int agent_id = parse_agent_id(info.node_namespace());
            if (agent_id < 0) continue;

            gid_to_agent_[gid_key] = agent_id;
            RCLCPP_INFO(get_logger(),
                "Discovered: ns='%s' -> agent_id=%d (total: %zu/%d)",
                info.node_namespace().c_str(),
                agent_id, gid_to_agent_.size(), num_agents_);
        }
    }

    int lookup_sender(const rclcpp::MessageInfo & info) const {
        uint64_t gid_key = make_gid_key(
            info.get_rmw_message_info().publisher_gid.data);
        auto it = gid_to_agent_.find(gid_key);
        return (it != gid_to_agent_.end()) ? it->second : -1;
    }

    // ================================================================
    // Message intake — route into per-agent queue
    // ================================================================
    void on_message(const MsgType::ConstSharedPtr msg,
                    const rclcpp::MessageInfo & info) {
        if (!enabled_) {
            publisher_->publish(*msg);
            return;
        }

        int sender_id = lookup_sender(info);

        if (sender_id < 0) {
            // Pre-discovery fallback. We do NOT round-robin into a
            // per-agent queue here because that would corrupt that
            // agent's queue accounting. Instead: publish immediately
            // (best-effort) and warn.
            //
            // Discovery transient is short (~1s); the cost of a few
            // unshaped messages at startup is much less than
            // mis-attributing them to the wrong agent's queue.
            undiscovered_count_++;
            if (undiscovered_count_ <= 5) {
                RCLCPP_WARN(get_logger(),
                    "Sender GID not yet discovered, publishing unshaped "
                    "(discovery 1Hz; %lu so far)",
                    static_cast<unsigned long>(undiscovered_count_));
            }
            publisher_->publish(*msg);
            return;
        }

        // Per-agent fan_out: how many subscribers does THIS sender reach?
        // For all_to_all this is N-1; for ring it's 2; etc.
        int fan_out = topology_->degree(sender_id);

        // Receiver: any neighbour works for the model's internal accounting.
        // Using sender's first neighbour for determinism.
        int receiver_id = (sender_id + 1) % num_agents_;

        double now_sec = this->now().seconds();
        double delay = latency_model_->sample(sender_id, receiver_id,
                                              now_sec, fan_out, 0);

        auto received_at = this->now();
        auto publish_time =
            received_at + rclcpp::Duration::from_seconds(delay);

        auto msg_copy = std::make_shared<MsgType>(*msg);

        per_agent_queues_[sender_id].push_back(DelayedMessage{
            msg_copy, publish_time, received_at, sender_id, delay
        });

        msg_count_++;
    }

    // ================================================================
    // Drain: iterate all N queues, publish messages whose time has come
    // ================================================================
    void drain_all_queues() {
        auto now = this->now();

        for (int aid = 0; aid < num_agents_; ++aid) {
            auto & q = per_agent_queues_[aid];

            while (!q.empty() && q.front().publish_at <= now) {
                auto & dm = q.front();
                double actual_delay = (now - dm.received_at).seconds();

                if (publish_diag_ && diag_pub_) {
                    auto diag = std_msgs::msg::String();
                    char buf[512];
                    std::snprintf(buf, sizeof(buf),
                        "model=%s topo=%s sender=%d "
                        "target_delay_ms=%.3f actual_delay_ms=%.3f "
                        "queue_local=%zu discovered=%zu",
                        model_name_.c_str(),
                        topology_->name().c_str(),
                        dm.sender_id,
                        dm.delay_sec * 1000.0,
                        actual_delay * 1000.0,
                        q.size(),
                        gid_to_agent_.size());
                    diag.data = buf;
                    diag_pub_->publish(diag);
                }

                publisher_->publish(*dm.msg);
                q.pop_front();
            }
        }
    }

    /// Periodic log: queue depth distribution. Useful to spot one agent
    /// falling behind.
    void log_queue_stats() {
        size_t total = 0;
        size_t max_depth = 0;
        int max_agent = -1;
        size_t nonempty = 0;
        for (int aid = 0; aid < num_agents_; ++aid) {
            size_t d = per_agent_queues_[aid].size();
            total += d;
            if (d > 0) nonempty++;
            if (d > max_depth) {
                max_depth = d;
                max_agent = aid;
            }
        }
        RCLCPP_INFO(get_logger(),
            "stats: msgs=%lu discovered=%zu/%d "
            "queues_nonempty=%zu/%d total_depth=%zu max=%zu(agent_%d)",
            static_cast<unsigned long>(msg_count_),
            gid_to_agent_.size(), num_agents_,
            nonempty, num_agents_,
            total, max_depth, max_agent);
    }

    // ================================================================
    // Members
    // ================================================================
    std::string input_topic_;
    std::string output_topic_;
    std::string model_name_;
    std::string ns_prefix_;
    bool enabled_;
    bool publish_diag_;
    int num_agents_;

    std::unique_ptr<LatencyModel> latency_model_;
    std::unique_ptr<Topology> topology_;

    // Per-agent state
    std::vector<std::deque<DelayedMessage>> per_agent_queues_;

    // Discovery
    std::map<uint64_t, int> gid_to_agent_;
    uint64_t undiscovered_count_ = 0;
    uint64_t msg_count_ = 0;

    // ROS2 interfaces
    rclcpp::Publisher<MsgType>::SharedPtr publisher_;
    rclcpp::Subscription<MsgType>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;
    rclcpp::TimerBase::SharedPtr drain_timer_;
    rclcpp::TimerBase::SharedPtr discovery_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        param_callback_handle_;
};

}  // namespace temporis_ros2

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // MultiThreadedExecutor: lets the drain timer, discovery timer,
    // and subscription callback run concurrently on different threads.
    // Important here because drain runs at 2kHz across N queues —
    // we don't want a slow discovery tick to skip drain ticks.
    rclcpp::executors::MultiThreadedExecutor exec(
        rclcpp::ExecutorOptions(), 0 /* auto-thread-count */);

    auto node = std::make_shared<temporis_ros2::PerAgentShaper>();
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}