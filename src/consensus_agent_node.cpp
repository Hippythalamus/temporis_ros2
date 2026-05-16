/// @file consensus_agent_node.cpp
/// @brief ROS2 consensus agent — port of consensus_demo to ROS2.
///
/// Each agent:
///   - Holds a scalar state (random init in [0, 100))
///   - At a fixed rate, publishes its state on /temporis/raw/states
///   - Subscribes to /fleet/states (delayed by Temporis shaper)
///   - On receiving a message from another agent:
///       state[i] = state[i] + alpha * (state[j] - state[i])
///   - Periodically logs variance for convergence detection
///
/// Message format (12 bytes minimum):
///   [8 bytes: state (double)]
///   [4 bytes: sender_agent_id (int32)]
///
/// Convergence detection:
///   Variance is computed locally per agent (each agent only knows
///   states it has received). Global variance requires aggregation —
///   for now we log per-agent and compute global externally from CSV.
///
/// Usage:
///   ros2 run temporis_ros2 consensus_agent --ros-args \
///     -p agent_id:=3 -p num_agents:=10 -p alpha:=0.1 -p rate_hz:=1.0

#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

namespace temporis_ros2 {

class ConsensusAgent : public rclcpp::Node {
public:
    ConsensusAgent()
        : Node("consensus_agent")
    {
        declare_parameter("agent_id", 0);
        declare_parameter("num_agents", 10);
        declare_parameter("alpha", 0.1);
        declare_parameter("rate_hz", 1.0);
        declare_parameter("pub_topic", "/temporis/raw/states");
        declare_parameter("sub_topic", "/fleet/states");
        declare_parameter("output_csv", "");  // empty = no CSV output
        declare_parameter("max_steps", 6387);
        declare_parameter("init_seed", 42);
        declare_parameter("convergence_threshold", 1e-6);
        declare_parameter("convergence_stable_steps", 5);

        agent_id_ = get_parameter("agent_id").as_int();
        num_agents_ = get_parameter("num_agents").as_int();
        alpha_ = get_parameter("alpha").as_double();
        auto rate_hz = get_parameter("rate_hz").as_double();
        auto pub_topic = get_parameter("pub_topic").as_string();
        auto sub_topic = get_parameter("sub_topic").as_string();
        output_csv_ = get_parameter("output_csv").as_string();
        max_steps_ = get_parameter("max_steps").as_int();
        convergence_threshold_ = get_parameter("convergence_threshold").as_double();
        convergence_stable_steps_ = get_parameter("convergence_stable_steps").as_int();
        int init_seed = get_parameter("init_seed").as_int();

        // Initialize state deterministically based on (init_seed, agent_id)
        // so all agents and the orchestrator agree on initial values.
        std::mt19937 rng(init_seed + agent_id_);
        std::uniform_real_distribution<double> dist(0.0, 100.0);
        state_ = dist(rng);

        // Track latest known state of each other agent (for variance)
        last_state_.assign(num_agents_, 0.0);
        has_state_.assign(num_agents_, false);
        last_state_[agent_id_] = state_;
        has_state_[agent_id_] = true;

        RCLCPP_INFO(get_logger(),
            "ConsensusAgent [id=%d/%d] alpha=%.3f rate=%.2fHz init_state=%.4f conv_thr=%.1e",
            agent_id_, num_agents_, alpha_, rate_hz, state_, convergence_threshold_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        publisher_ = create_publisher<std_msgs::msg::ByteMultiArray>(
            pub_topic, qos);
        subscription_ = create_subscription<std_msgs::msg::ByteMultiArray>(
            sub_topic, qos,
            std::bind(&ConsensusAgent::on_message, this, std::placeholders::_1));

        // Publish timer (drives consensus step)
        auto period = std::chrono::duration<double>(1.0 / rate_hz);
        step_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&ConsensusAgent::step, this));

        // Stats timer (every 2 seconds)
        stats_timer_ = create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&ConsensusAgent::log_stats, this));

        // Open CSV if requested
        if (!output_csv_.empty()) {
            csv_file_.open(output_csv_);
            if (csv_file_.is_open()) {
                csv_file_ << "step,state,local_variance,recv_count\n";
            } else {
                RCLCPP_WARN(get_logger(),
                    "Could not open CSV: %s", output_csv_.c_str());
            }
        }
    }

    ~ConsensusAgent() override {
        if (csv_file_.is_open()) csv_file_.close();
    }

private:
    void step() {
        if (step_ >= max_steps_) {
            if (!finished_) {
                finished_ = true;
                RCLCPP_INFO(get_logger(),
                    "[agent_%d] MAX_STEPS reached at step %d, final_state=%.6f var=%.2e",
                    agent_id_, step_, state_, compute_local_variance());
                step_timer_->cancel();
                if (csv_file_.is_open()) csv_file_.close();
            }
            return;
        }

        // Publish current state
        std_msgs::msg::ByteMultiArray msg;
        msg.data.resize(12, 0);
        std::memcpy(msg.data.data(), &state_, sizeof(state_));
        int32_t id = agent_id_;
        std::memcpy(msg.data.data() + 8, &id, sizeof(id));
        publisher_->publish(msg);

        double var = compute_local_variance();

        // Log to CSV
        if (csv_file_.is_open()) {
            csv_file_ << step_ << "," << state_ << ","
                      << var << ","
                      << recv_count_ << "\n";
            csv_file_.flush();  // flush so data is safe even on Ctrl+C
        }

        // Convergence check: variance below threshold for N consecutive steps.
        // Only count after all agents are known (avoid early-startup false positives).
        int known = 0;
        for (bool b : has_state_) if (b) known++;
        if (known == num_agents_ && var < convergence_threshold_) {
            convergence_count_++;
            if (convergence_count_ >= convergence_stable_steps_) {
                finished_ = true;
                RCLCPP_INFO(get_logger(),
                    "[agent_%d] CONVERGED at step %d, final_state=%.6f var=%.2e (threshold=%.1e)",
                    agent_id_, step_, state_, var, convergence_threshold_);
                step_timer_->cancel();
                if (csv_file_.is_open()) csv_file_.close();
                return;
            }
        } else {
            convergence_count_ = 0;
        }

        step_++;
    }

    void on_message(const std_msgs::msg::ByteMultiArray::ConstSharedPtr msg) {
        if (msg->data.size() < 12) return;

        double remote_state = 0.0;
        int32_t sender_id = 0;
        std::memcpy(&remote_state, msg->data.data(), sizeof(remote_state));
        std::memcpy(&sender_id, msg->data.data() + 8, sizeof(sender_id));

        // Skip own messages
        if (sender_id == agent_id_) return;
        if (sender_id < 0 || sender_id >= num_agents_) return;

        // Consensus update: state = state + alpha * (remote - state) / num_neighbors
        // Per-message version (each incoming message contributes its share):
        state_ = state_ + alpha_ * (remote_state - state_) / (num_agents_ - 1);

        // Track for variance
        last_state_[sender_id] = remote_state;
        has_state_[sender_id] = true;
        last_state_[agent_id_] = state_;  // own state always current

        recv_count_++;
    }

    /// Compute variance over known states (own + received).
    double compute_local_variance() const {
        double sum = 0.0;
        int count = 0;
        for (int i = 0; i < num_agents_; ++i) {
            if (has_state_[i]) {
                sum += last_state_[i];
                count++;
            }
        }
        if (count < 2) return 0.0;
        double mean = sum / count;
        double var = 0.0;
        for (int i = 0; i < num_agents_; ++i) {
            if (has_state_[i]) {
                double d = last_state_[i] - mean;
                var += d * d;
            }
        }
        return var / count;
    }

    void log_stats() {
        if (finished_) return;
        double var = compute_local_variance();
        int known = 0;
        for (bool b : has_state_) if (b) known++;
        RCLCPP_INFO(get_logger(),
            "[agent_%d] step=%d state=%.4f local_var=%.4e known=%d recv=%lu",
            agent_id_, step_, state_, var, known, recv_count_);
    }

    int agent_id_ = 0;
    int num_agents_ = 10;
    double alpha_ = 0.1;
    int step_ = 0;
    int max_steps_ = 6387;
    double convergence_threshold_ = 1e-6;
    int convergence_stable_steps_ = 5;
    int convergence_count_ = 0;
    bool finished_ = false;

    double state_ = 0.0;
    std::vector<double> last_state_;
    std::vector<bool> has_state_;
    uint64_t recv_count_ = 0;

    std::string output_csv_;
    std::ofstream csv_file_;

    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr step_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace temporis_ros2

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<temporis_ros2::ConsensusAgent>());
    rclcpp::shutdown();
    return 0;
}