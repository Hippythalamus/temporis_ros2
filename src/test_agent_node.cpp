/// @file test_agent_node.cpp
/// @brief Minimal agent for testing Temporis shaper in multi-agent setup.
///
/// Each agent:
///   - Publishes its agent_id + timestamp at a fixed rate
///   - Subscribes to the same topic and receives messages from other agents
///   - Measures delivery latency (receive_time - embedded_send_time)
///   - Logs statistics every 5 seconds
///
/// Message format: first 12 bytes of ByteMultiArray.data:
///   [8 bytes: send_timestamp_ns (int64)]
///   [4 bytes: sender_agent_id (int32)]
///
/// Usage:
///   ros2 run temporis_ros2 test_agent --ros-args \
///     -p agent_id:=3 -p num_agents:=5 -p rate_hz:=10.0

#include <chrono>
#include <cstdint>
#include <cstring>
#include <numeric>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

namespace temporis_ros2 {

class TestAgent : public rclcpp::Node {
public:
    TestAgent()
        : Node("test_agent")
    {
        declare_parameter("agent_id", 0);
        declare_parameter("num_agents", 5);
        declare_parameter("rate_hz", 10.0);
        declare_parameter("pub_topic", "/temporis/raw/states");
        declare_parameter("sub_topic", "/fleet/states");

        agent_id_ = get_parameter("agent_id").as_int();
        num_agents_ = get_parameter("num_agents").as_int();
        auto rate_hz = get_parameter("rate_hz").as_double();
        auto pub_topic = get_parameter("pub_topic").as_string();
        auto sub_topic = get_parameter("sub_topic").as_string();


        RCLCPP_INFO(get_logger(),
            "TestAgent [id=%d/%d] rate=%.1fHz pub=%s sub=%s",
            agent_id_,
            num_agents_,
            rate_hz,
            pub_topic.c_str(),
            sub_topic.c_str());

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        publisher_ = create_publisher<std_msgs::msg::ByteMultiArray>(
            pub_topic, qos);

        subscription_ = create_subscription<std_msgs::msg::ByteMultiArray>(
            sub_topic, qos,
            std::bind(&TestAgent::on_message, this, std::placeholders::_1));

        // Publish timer
        auto period = std::chrono::duration<double>(1.0 / rate_hz);
        pub_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&TestAgent::publish_state, this));

        // Stats timer — log every 5 seconds
        stats_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&TestAgent::log_stats, this));
    }

private:
    void publish_state() {
        auto msg = std_msgs::msg::ByteMultiArray();
        msg.data.resize(12, 0);

        auto now_ns = this->now().nanoseconds();
        int32_t id = agent_id_;

        std::memcpy(msg.data.data(), &now_ns, sizeof(now_ns));
        std::memcpy(msg.data.data() + 8, &id, sizeof(id));

        publisher_->publish(msg);
        pub_count_++;
    }

    void on_message(const std_msgs::msg::ByteMultiArray::ConstSharedPtr msg) {
        if (msg->data.size() < 12) return;

        int64_t send_ns = 0;
        int32_t sender_id = 0;
        std::memcpy(&send_ns, msg->data.data(), sizeof(send_ns));
        std::memcpy(&sender_id, msg->data.data() + 8, sizeof(sender_id));

        // Skip own messages
        if (sender_id == agent_id_) return;

        auto now_ns = this->now().nanoseconds();
        double latency_ms = static_cast<double>(now_ns - send_ns) / 1e6;

        // Store for stats
        latencies_ms_.push_back(latency_ms);
        recv_count_++;
    }

    void log_stats() {
        if (latencies_ms_.empty()) {
            RCLCPP_INFO(get_logger(),
                "[agent_%d] pub=%lu recv=%lu (no latency data yet)",
                agent_id_, pub_count_, recv_count_);
            return;
        }

        // Compute stats
        auto n = latencies_ms_.size();
        double sum = std::accumulate(latencies_ms_.begin(),
                                      latencies_ms_.end(), 0.0);
        double mean = sum / n;

        auto sorted = latencies_ms_;
        std::sort(sorted.begin(), sorted.end());
        double median = sorted[n / 2];
        double p95 = sorted[static_cast<size_t>(n * 0.95)];
        double min_val = sorted.front();
        double max_val = sorted.back();

        RCLCPP_INFO(get_logger(),
            "[agent_%d] pub=%lu recv=%lu | latency: mean=%.1fms "
            "median=%.1fms p95=%.1fms min=%.1fms max=%.1fms (n=%zu)",
            agent_id_, pub_count_, recv_count_,
            mean, median, p95, min_val, max_val, n);

        // Reset for next window
        latencies_ms_.clear();
    }

    int agent_id_ = 0;
    int num_agents_ = 5;

    uint64_t pub_count_ = 0;
    uint64_t recv_count_ = 0;
    std::vector<double> latencies_ms_;

    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace temporis_ros2

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<temporis_ros2::TestAgent>());
    rclcpp::shutdown();
    return 0;
}