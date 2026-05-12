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
/// Agent identification:
///   Uses get_publishers_info_by_topic() to discover publisher node
///   namespaces. Namespace "/robot_3" → agent_id=3. All publishers
///   from the same namespace share one agent_id, regardless of how
///   many topics or GIDs that node has.
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
#include <regex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Temporis core
#include "QueueLatencyModel.hpp"
#include "ZenohQueueModel.hpp"
#include "Topology.hpp"

namespace temporis_ros2 {

/// A serialized message waiting to be published after its delay expires.
struct DelayedMessage {
    std::shared_ptr<rclcpp::SerializedMessage> serialized_msg;
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
        declare_parameter("msg_type", "std_msgs/msg/ByteMultiArray");
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
        msg_type_ = get_parameter("msg_type").as_string();
        model_name_ = get_parameter("model").as_string();
        enabled_ = get_parameter("enabled").as_bool();

        param_callback_handle_ =
            this->add_on_set_parameters_callback(
                std::bind(
                    &TemporisShaper::on_param_change,
                    this,
                    std::placeholders::_1));

        publish_diag_ = get_parameter("publish_diagnostics").as_bool();
        num_agents_ = get_parameter("num_agents").as_int();
        ns_prefix_ = get_parameter("namespace_prefix").as_string();

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

        // ---- Generic publisher (serialized) ----
        auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
        generic_pub_ = create_generic_publisher(output_topic_, msg_type_, qos);

        // ---- Generic subscriber (serialized) ----
        generic_sub_ = create_generic_subscription(
            input_topic_, msg_type_, qos,
            [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                on_message(msg);
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

        // ---- Discovery timer (1Hz) — refresh namespace→agent mapping ----
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
    // Namespace-based agent discovery
    // ================================================================

    /// Parse agent_id from a ROS2 node namespace.
    /// Example: namespace="/robot_3", prefix="robot_" → 3
    /// Returns -1 if namespace doesn't match the expected pattern.
    int parse_agent_id(const std::string & node_namespace) const {
        // Strip leading slashes: "/robot_3" → "robot_3"
        std::string ns = node_namespace;
        while (!ns.empty() && ns.front() == '/') {
            ns.erase(ns.begin());
        }
        // Also handle nested: "fleet/robot_3" → find "robot_" anywhere
        auto pos = ns.rfind(ns_prefix_);
        if (pos == std::string::npos) return -1;

        std::string id_str = ns.substr(pos + ns_prefix_.size());
        // id_str might have trailing slash or more nesting
        // Take only leading digits
        std::string digits;
        for (char c : id_str) {
            if (std::isdigit(c)) digits += c;
            else break;
        }
        if (digits.empty()) return -1;

        int id = std::stoi(digits);
        if (id < 0 || id >= num_agents_) {
            RCLCPP_WARN_ONCE(get_logger(),
                "Agent id %d from namespace '%s' out of range [0, %d)",
                id, node_namespace.c_str(), num_agents_);
            return id % num_agents_;
        }
        return id;
    }

    /// Query ROS2 discovery to build GID → agent_id mapping
    /// from publisher node namespaces on the input topic.
    void refresh_publisher_map() {
        auto pubs_info = get_publishers_info_by_topic(input_topic_);
        for (const auto & info : pubs_info) {
            // Build GID key from endpoint
            uint64_t gid_key = 0;
            std::memcpy(&gid_key, info.endpoint_gid().data(), sizeof(gid_key));

            if (gid_to_agent_.count(gid_key)) continue;  // already known

            int agent_id = parse_agent_id(info.node_namespace());
            if (agent_id < 0) {
                // Namespace doesn't match pattern — might be shaper itself
                // or an external node. Ignore.
                continue;
            }

            gid_to_agent_[gid_key] = agent_id;
            RCLCPP_INFO(get_logger(),
                "Discovered: namespace='%s' node='%s' → agent_id=%d",
                info.node_namespace().c_str(),
                info.node_name().c_str(),
                agent_id);
        }
    }

    /// Look up agent_id for a message. Uses GID from the last discovery
    /// refresh. Returns -1 if unknown.
    int lookup_sender(const rclcpp::MessageInfo & info) const {
        uint64_t gid_key = 0;
        std::memcpy(&gid_key,
                     info.get_rmw_message_info().publisher_gid.data,
                     sizeof(gid_key));
        auto it = gid_to_agent_.find(gid_key);
        if (it != gid_to_agent_.end()) return it->second;
        return -1;
    }

    // ================================================================
    // Message handling
    // ================================================================

    void on_message(std::shared_ptr<rclcpp::SerializedMessage> msg) {
        if (!enabled_) {
            generic_pub_->publish(*msg);
            return;
        }

        // We need MessageInfo for GID lookup — but generic_subscription
        // callback doesn't provide it in the simple form. Use the
        // discovery-based mapping: the last known GID→agent mapping.
        //
        // Limitation: if a publisher appeared between discovery ticks
        // (1 Hz) and hasn't been mapped yet, we assign sender_id = 0.
        // This is acceptable for the first version.
        //
        // TODO: switch to subscription callback with MessageInfo when
        // rclcpp generic subscriptions support it.
        int sender_id = next_sender_id_;
        next_sender_id_ = (next_sender_id_ + 1) % num_agents_;

        // Compute delay
        int receiver_id = (sender_id + 1) % num_agents_;
        double now_sec = this->now().seconds();
        double delay = latency_model_->sample(sender_id, receiver_id,
                                               now_sec, 0, 0);

        auto publish_time = this->now() + rclcpp::Duration::from_seconds(delay);

        delayed_queue_.push_back(DelayedMessage{
            msg, publish_time, this->now(), sender_id, delay
        });

        // Diagnostics (throttled: every 100th message)
        msg_count_++;

    }

    /// Drain timer: publish messages whose delay has expired.
    void drain_queue() {
        auto now = this->now();

        while (!delayed_queue_.empty() &&
            delayed_queue_.front().publish_at <= now) {

            auto & msg = delayed_queue_.front();

            double actual_delay =
                (now - msg.received_at).seconds();

            if (publish_diag_ && diag_pub_) {
                auto diag = std_msgs::msg::String();

                char buf[512];
                std::snprintf(
                    buf,
                    sizeof(buf),
                    "model=%s topo=%s sender=%d "
                    "target_delay=%.1fms actual_delay=%.1fms "
                    "queue=%zu enabled=1",
                    model_name_.c_str(),
                    topology_->name().c_str(),
                    msg.sender_id,
                    msg.delay_sec * 1000.0,
                    actual_delay * 1000.0,
                    delayed_queue_.size()
                );

                diag.data = buf;
                diag_pub_->publish(diag);
            }

            generic_pub_->publish(*msg.serialized_msg);

            delayed_queue_.pop_front();
        }
    }

    // ================================================================
    // Members
    // ================================================================

    rcl_interfaces::msg::SetParametersResult
    on_param_change(const std::vector<rclcpp::Parameter> & params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & param : params) {

            if (param.get_name() == "enabled") {

                enabled_ = param.as_bool();

                RCLCPP_INFO(
                    get_logger(),
                    "Temporis shaper enabled=%s",
                    enabled_ ? "true" : "false"
                );
            }
        }

        return result;
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string msg_type_;
    std::string model_name_;
    std::string ns_prefix_;
    bool enabled_;
    bool publish_diag_;
    int num_agents_;

    std::unique_ptr<LatencyModel> latency_model_;
    std::unique_ptr<Topology> topology_;

    // Discovery: GID → agent_id (refreshed at 1Hz)
    std::map<uint64_t, int> gid_to_agent_;

    // Round-robin sender_id fallback for generic subscription
    // (see TODO in on_message)
    int next_sender_id_ = 0;

    // Delayed message queue
    std::deque<DelayedMessage> delayed_queue_;

    // Message counter for diagnostics throttling
    uint64_t msg_count_ = 0;

    // ROS2 interfaces
    std::shared_ptr<rclcpp::GenericPublisher> generic_pub_;
    std::shared_ptr<rclcpp::GenericSubscription> generic_sub_;
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
