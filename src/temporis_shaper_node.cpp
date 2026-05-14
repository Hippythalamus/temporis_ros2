/// @file temporis_shaper_node.cpp
/// @brief ROS2 node that intercepts messages and applies Temporis
///        latency model delays before re-publishing.
///
/// Architecture:
///   Agents publish to their normal topic. In sim, a launch file remaps
///   their output to an internal topic. This shaper reads from that
///   internal topic, applies a delay computed by the Temporis latency
///   model, and re-publishes on the original topic name.
///
///   In production the launch file has no remap and no shaper —
///   agents talk directly. Zero code changes between sim and production.
///
/// Agent identification (Step 3):
///   Typed subscription provides MessageInfo with publisher GID.
///   Discovery timer (1Hz) maps GID → node namespace → agent_id.
///   Namespace "/robot_3" with prefix "robot_" → agent_id=3.
///   All publishers from the same namespace share one agent_id.
///
///   If GID is not yet discovered (first messages before discovery
///   tick), falls back to round-robin with a warning.
///
/// Message type:
///   Uses ByteMultiArray as the transport type. For a different
///   msg type, replace MsgType alias and rebuild.
///
/// Latency models supported:
///   QUEUE        — shared sender-side M/D/1 with bandwidth AR(1)
///   ZENOH_QUEUE  — two-stage client+router, calibrated from real benchmark
///
/// Topology support:
///   all_to_all, ring, grid, random_k — determines which agent pairs
///   communicate, affecting queue load and convergence behavior.

#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Temporis core
#include "QueueLatencyModel.hpp"
#include "ZenohQueueModel.hpp"
#include "Topology.hpp"

namespace temporis_ros2 {

using MsgType = std_msgs::msg::ByteMultiArray;

/// A message waiting to be published after its delay expires.
struct DelayedMessage {
    MsgType::SharedPtr msg;
    rclcpp::Time publish_at;
    rclcpp::Time received_at;
    int sender_id;
    double delay_sec;
};

class TemporisShaper : public rclcpp::Node {
public:
    TemporisShaper()
        : Node("temporis_shaper")
    {
        // ---- Parameters ----
        declare_parameter("input_topic", "/temporis/raw/states");
        declare_parameter("output_topic", "/fleet/states");
        declare_parameter("model", "ZENOH_QUEUE");
        declare_parameter("topology", "all_to_all");
        declare_parameter("topology_k", 5);
        declare_parameter("num_agents", 10);
        declare_parameter("enabled", true);
        declare_parameter("publish_diagnostics", true);
        declare_parameter("namespace_prefix", "robot_");

        // ZENOH_QUEUE parameters
        declare_parameter("client_bandwidth", 1000.0);
        declare_parameter("packet_size", 1.0);
        declare_parameter("propagation_client_router", 0.000150);
        declare_parameter("propagation_router_subscriber", 0.000150);
        declare_parameter("router_base_cost", 0.000108);
        declare_parameter("router_per_sub_cost", 0.0000027);

        // QUEUE parameters
        declare_parameter("bandwidth", 70.0);
        declare_parameter("propagation_delay", 0.05);

        // Shared noise parameters
        declare_parameter("bandwidth_logstd", 0.0);
        declare_parameter("bandwidth_rho", 0.0);
        declare_parameter("seed", 42);

        // ---- Read ----
        input_topic_ = get_parameter("input_topic").as_string();
        output_topic_ = get_parameter("output_topic").as_string();
        model_name_ = get_parameter("model").as_string();
        enabled_ = get_parameter("enabled").as_bool();
        publish_diag_ = get_parameter("publish_diagnostics").as_bool();
        num_agents_ = get_parameter("num_agents").as_int();
        ns_prefix_ = get_parameter("namespace_prefix").as_string();

        // Parameter change callback (allows `ros2 param set enabled false`)
        param_callback_handle_ =
            this->add_on_set_parameters_callback(
                std::bind(&TemporisShaper::on_param_change,
                          this, std::placeholders::_1));

        // ---- Build latency model ----
        build_latency_model();

        // ---- Build topology ----
        auto topo_name = get_parameter("topology").as_string();
        auto topo_k = get_parameter("topology_k").as_int();
        auto seed = static_cast<uint64_t>(get_parameter("seed").as_int());
        topology_ = make_topology(topo_name, num_agents_, topo_k, seed);

        RCLCPP_INFO(get_logger(),
            "Temporis shaper: model=%s topology=%s agents=%d enabled=%s",
            model_name_.c_str(), topology_->name().c_str(),
            num_agents_, enabled_ ? "true" : "false");
        RCLCPP_INFO(get_logger(),
            "  input: %s → output: %s",
            input_topic_.c_str(), output_topic_.c_str());
        RCLCPP_INFO(get_logger(),
            "  namespace_prefix: '%s' (e.g. /robot_3 → agent_id=3)",
            ns_prefix_.c_str());

        auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();

        // ---- Typed publisher ----
        publisher_ = create_publisher<MsgType>(output_topic_, qos);

        // ---- Typed subscription WITH MessageInfo ----
        // Key difference from generic subscription: callback receives
        // MessageInfo, giving us the publisher GID for sender lookup.
        subscription_ = create_subscription<MsgType>(
            input_topic_, qos,
            [this](const MsgType::ConstSharedPtr msg,
                   const rclcpp::MessageInfo & info) {
                on_message(msg, info);
            });

        // ---- Diagnostics ----
        if (publish_diag_) {
            diag_pub_ = create_publisher<std_msgs::msg::String>(
                "/temporis/diagnostics", 10);
        }

        // ---- Drain timer (2kHz) ----
        drain_timer_ = create_wall_timer(
            std::chrono::microseconds(500),
            std::bind(&TemporisShaper::drain_queue, this));

        // ---- Discovery timer (1Hz) — refresh GID→agent mapping ----
        discovery_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TemporisShaper::refresh_publisher_map, this));
    }

private:
    // ================================================================
    // Latency model construction
    // ================================================================
    void build_latency_model() {
        if (model_name_ == "ZENOH_QUEUE") {
            ZenohQueueModel::Config cfg{};
            cfg.client_bandwidth = get_parameter("client_bandwidth").as_double();
            cfg.packet_size = get_parameter("packet_size").as_double();
            cfg.propagation_client_router = get_parameter("propagation_client_router").as_double();
            cfg.propagation_router_subscriber = get_parameter("propagation_router_subscriber").as_double();
            cfg.router_base_cost = get_parameter("router_base_cost").as_double();
            cfg.router_per_sub_cost = get_parameter("router_per_sub_cost").as_double();
            cfg.num_agents = num_agents_;
            cfg.client_bandwidth_logstd = get_parameter("bandwidth_logstd").as_double();
            cfg.client_bandwidth_rho = get_parameter("bandwidth_rho").as_double();
            cfg.seed = static_cast<uint64_t>(get_parameter("seed").as_int());
            latency_model_ = std::make_unique<ZenohQueueModel>(cfg);
        } else if (model_name_ == "QUEUE") {
            QueueLatencyModel::Config cfg{};
            cfg.bandwidth = get_parameter("bandwidth").as_double();
            cfg.propagation_delay = get_parameter("propagation_delay").as_double();
            cfg.packet_size = get_parameter("packet_size").as_double();
            cfg.num_agents = num_agents_;
            cfg.bandwidth_logstd = get_parameter("bandwidth_logstd").as_double();
            cfg.bandwidth_rho = get_parameter("bandwidth_rho").as_double();
            cfg.seed = static_cast<uint64_t>(get_parameter("seed").as_int());
            latency_model_ = std::make_unique<QueueLatencyModel>(cfg);
        } else {
            throw std::runtime_error("Unknown Temporis model: " + model_name_);
        }
    }

    // ================================================================
    // Parameter change callback
    // ================================================================
    rcl_interfaces::msg::SetParametersResult
    on_param_change(const std::vector<rclcpp::Parameter> & params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & param : params) {
            if (param.get_name() == "enabled") {
                enabled_ = param.as_bool();
                RCLCPP_INFO(get_logger(),
                    "Temporis shaper enabled=%s",
                    enabled_ ? "true" : "false");
            }
        }
        return result;
    }

    // ================================================================
    // Namespace-based agent discovery
    // ================================================================

    /// Parse agent_id from a ROS2 node namespace.
    /// "/robot_3" with prefix "robot_" → 3
    /// Returns -1 if namespace doesn't match the expected pattern.
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

    /// Build GID key from raw GID bytes (hash combine of both halves).
    static uint64_t make_gid_key(const uint8_t * data) {
        uint64_t a = 0, b = 0;
        std::memcpy(&a, data, sizeof(a));
        std::memcpy(&b, data + 8, sizeof(b));
        return a ^ (b * 0x9e3779b97f4a7c15ULL);
    }

    /// Refresh GID → agent_id mapping from ROS2 discovery.
    void refresh_publisher_map() {
        auto pubs_info = get_publishers_info_by_topic(input_topic_);
        for (const auto & info : pubs_info) {
            uint64_t gid_key = make_gid_key(info.endpoint_gid().data());

            if (gid_to_agent_.count(gid_key)) continue;

            int agent_id = parse_agent_id(info.node_namespace());
            if (agent_id < 0) continue;

            gid_to_agent_[gid_key] = agent_id;
            RCLCPP_INFO(get_logger(),
                "Discovered: ns='%s' node='%s' → agent_id=%d (total: %zu)",
                info.node_namespace().c_str(),
                info.node_name().c_str(),
                agent_id, gid_to_agent_.size());
        }
    }

    /// Look up agent_id by GID from MessageInfo.
    /// Returns discovered agent_id, or -1 if not yet known.
    int lookup_sender(const rclcpp::MessageInfo & info) const {
        uint64_t gid_key = make_gid_key(
            info.get_rmw_message_info().publisher_gid.data);
        auto it = gid_to_agent_.find(gid_key);
        return (it != gid_to_agent_.end()) ? it->second : -1;
    }

    // ================================================================
    // Message handling
    // ================================================================

    void on_message(const MsgType::ConstSharedPtr msg,
                    const rclcpp::MessageInfo & info) {
        if (!enabled_) {
            publisher_->publish(*msg);
            return;
        }

        // Look up sender via GID → namespace → agent_id
        int sender_id = lookup_sender(info);

        if (sender_id < 0) {
            // Not yet discovered — fallback to round-robin
            sender_id = next_sender_id_;
            next_sender_id_ = (next_sender_id_ + 1) % num_agents_;
            undiscovered_count_++;
            if (undiscovered_count_ <= 5) {
                RCLCPP_WARN(get_logger(),
                    "Sender GID not yet discovered, using fallback id=%d "
                    "(discovery refreshes at 1Hz)", sender_id);
            }
        }

        // Compute delay
        int receiver_id = (sender_id + 1) % num_agents_;
        double now_sec = this->now().seconds();
        double delay = latency_model_->sample(sender_id, receiver_id,
                                               now_sec, 0, 0);

        auto received_at = this->now();
        auto publish_time = received_at + rclcpp::Duration::from_seconds(delay);

        // Mutable copy for delayed publishing
        auto msg_copy = std::make_shared<MsgType>(*msg);

        delayed_queue_.push_back(DelayedMessage{
            msg_copy, publish_time, received_at, sender_id, delay
        });

        msg_count_++;
    }

    /// Drain timer: publish messages whose delay has expired.
    /// Emits diagnostics (target vs actual delay) for each published message.
    void drain_queue() {
        auto now = this->now();

        while (!delayed_queue_.empty() &&
               delayed_queue_.front().publish_at <= now) {

            auto & dm = delayed_queue_.front();

            double actual_delay = (now - dm.received_at).seconds();

            if (publish_diag_ && diag_pub_) {
                auto diag = std_msgs::msg::String();
                char buf[512];
                std::snprintf(buf, sizeof(buf),
                    "model=%s topo=%s sender=%d "
                    "target_delay=%.1fms actual_delay=%.1fms "
                    "queue=%zu discovered=%zu enabled=1",
                    model_name_.c_str(),
                    topology_->name().c_str(),
                    dm.sender_id,
                    dm.delay_sec * 1000.0,
                    actual_delay * 1000.0,
                    delayed_queue_.size(),
                    gid_to_agent_.size());
                diag.data = buf;
                diag_pub_->publish(diag);
            }

            publisher_->publish(*dm.msg);
            delayed_queue_.pop_front();
        }
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

    // Discovery: GID → agent_id (refreshed at 1Hz)
    std::map<uint64_t, int> gid_to_agent_;

    // Fallback round-robin (used only before discovery completes)
    int next_sender_id_ = 0;
    uint64_t undiscovered_count_ = 0;

    // Delayed message queue
    std::deque<DelayedMessage> delayed_queue_;

    // Message counter (informational)
    uint64_t msg_count_ = 0;

    // ROS2 interfaces
    rclcpp::Publisher<MsgType>::SharedPtr publisher_;
    rclcpp::Subscription<MsgType>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;
    rclcpp::TimerBase::SharedPtr drain_timer_;
    rclcpp::TimerBase::SharedPtr discovery_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        param_callback_handle_;
};

}  // namespace temporis_ros2

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<temporis_ros2::TemporisShaper>());
    rclcpp::shutdown();
    return 0;
}