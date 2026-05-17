/// @file consensus_agent_node.cpp
/// @brief ROS2 consensus agent — Step 8 v2 (QoS-parameterized).
///
/// Step 8 v2 changes vs Step 8 v1:
///   - Added qos_reliability parameter ("reliable" | "best_effort")
///   - Added qos_depth parameter (KeepLast queue depth)
///   - Default kept reliable+10 for backward compatibility, but tests
///     of "transport latency" should use best_effort+1 to avoid
///     QoS-induced backlog dominating the measurement.
///
/// Validation matrix:
///   - reliable+10:   measures transport + QoS buffering (what Step 8 v1 measured)
///   - best_effort+1: measures transport only (Temporis scope)
///
/// Payload unchanged from v1: 20 bytes
///   [state(8) | sender_id(4) | pub_time_ns(8)]

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

static constexpr size_t PAYLOAD_BYTES = 20;
static constexpr size_t OFF_STATE = 0;
static constexpr size_t OFF_SENDER = 8;
static constexpr size_t OFF_PUBTIME = 12;

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
        declare_parameter("output_csv", "");
        declare_parameter("latency_csv", "");
        declare_parameter("max_steps", 6387);
        declare_parameter("init_seed", 42);
        declare_parameter("convergence_threshold", 1e-6);
        declare_parameter("convergence_stable_steps", 5);

        // Step 8 v2: QoS knobs
        declare_parameter("qos_reliability", std::string("reliable"));
        declare_parameter("qos_depth", 10);

        agent_id_ = get_parameter("agent_id").as_int();
        num_agents_ = get_parameter("num_agents").as_int();
        alpha_ = get_parameter("alpha").as_double();
        auto rate_hz = get_parameter("rate_hz").as_double();
        auto pub_topic = get_parameter("pub_topic").as_string();
        auto sub_topic = get_parameter("sub_topic").as_string();
        output_csv_ = get_parameter("output_csv").as_string();
        latency_csv_ = get_parameter("latency_csv").as_string();
        max_steps_ = get_parameter("max_steps").as_int();
        convergence_threshold_ =
            get_parameter("convergence_threshold").as_double();
        convergence_stable_steps_ =
            get_parameter("convergence_stable_steps").as_int();
        int init_seed = get_parameter("init_seed").as_int();

        auto qos_rel = get_parameter("qos_reliability").as_string();
        auto qos_depth = get_parameter("qos_depth").as_int();

        std::mt19937 rng(init_seed + agent_id_);
        std::uniform_real_distribution<double> dist(0.0, 100.0);
        state_ = dist(rng);

        last_state_.assign(num_agents_, 0.0);
        has_state_.assign(num_agents_, false);
        last_state_[agent_id_] = state_;
        has_state_[agent_id_] = true;

        RCLCPP_INFO(get_logger(),
            "ConsensusAgent [id=%d/%d] alpha=%.3f rate=%.2fHz "
            "init_state=%.4f qos=%s+depth%d",
            agent_id_, num_agents_, alpha_, rate_hz, state_,
            qos_rel.c_str(), static_cast<int>(qos_depth));

        // Build QoS from parameters
        auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
        if (qos_rel == "best_effort" || qos_rel == "BEST_EFFORT") {
            qos.best_effort();
        } else {
            qos.reliable();
        }

        publisher_ = create_publisher<std_msgs::msg::ByteMultiArray>(
            pub_topic, qos);
        subscription_ = create_subscription<std_msgs::msg::ByteMultiArray>(
            sub_topic, qos,
            std::bind(&ConsensusAgent::on_message, this,
                      std::placeholders::_1));

        auto period = std::chrono::duration<double>(1.0 / rate_hz);
        step_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&ConsensusAgent::step, this));

        stats_timer_ = create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&ConsensusAgent::log_stats, this));

        if (!output_csv_.empty()) {
            csv_file_.open(output_csv_);
            if (csv_file_.is_open()) {
                csv_file_ << "step,state,local_variance,recv_count\n";
            }
        }

        if (!latency_csv_.empty()) {
            latency_file_.open(latency_csv_);
            if (latency_file_.is_open()) {
                latency_file_
                    << "recv_wall_time_ns,sender_id,latency_us,step_at_recv\n";
            }
        }
    }

    ~ConsensusAgent() override {
        if (csv_file_.is_open()) csv_file_.close();
        if (latency_file_.is_open()) latency_file_.close();
    }

private:
    static int64_t steady_ns_now() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();
    }

    void step() {
        if (step_ >= max_steps_) {
            if (!finished_) {
                finished_ = true;
                RCLCPP_INFO(get_logger(),
                    "[agent_%d] MAX_STEPS reached at step %d, "
                    "final_state=%.6f var=%.2e",
                    agent_id_, step_, state_, compute_local_variance());
                step_timer_->cancel();
                if (csv_file_.is_open()) csv_file_.close();
                if (latency_file_.is_open()) latency_file_.close();
            }
            return;
        }

        std_msgs::msg::ByteMultiArray msg;
        msg.data.resize(PAYLOAD_BYTES, 0);

        std::memcpy(msg.data.data() + OFF_STATE,  &state_,  sizeof(state_));
        int32_t id = agent_id_;
        std::memcpy(msg.data.data() + OFF_SENDER, &id,      sizeof(id));
        int64_t pub_ns = steady_ns_now();
        std::memcpy(msg.data.data() + OFF_PUBTIME, &pub_ns, sizeof(pub_ns));

        publisher_->publish(msg);

        double var = compute_local_variance();

        if (csv_file_.is_open()) {
            csv_file_ << step_ << "," << state_ << ","
                      << var << "," << recv_count_ << "\n";
            csv_file_.flush();
        }

        int known = 0;
        for (bool b : has_state_) if (b) known++;
        if (known == num_agents_ && var < convergence_threshold_) {
            convergence_count_++;
            if (convergence_count_ >= convergence_stable_steps_) {
                finished_ = true;
                RCLCPP_INFO(get_logger(),
                    "[agent_%d] CONVERGED at step %d, "
                    "final_state=%.6f var=%.2e (threshold=%.1e)",
                    agent_id_, step_, state_, var, convergence_threshold_);
                step_timer_->cancel();
                if (csv_file_.is_open()) csv_file_.close();
                if (latency_file_.is_open()) latency_file_.close();
                return;
            }
        } else {
            convergence_count_ = 0;
        }

        step_++;
    }

    void on_message(const std_msgs::msg::ByteMultiArray::ConstSharedPtr msg) {
        if (msg->data.size() < PAYLOAD_BYTES) return;

        int64_t recv_ns = steady_ns_now();

        double remote_state = 0.0;
        int32_t sender_id = 0;
        int64_t pub_ns = 0;
        std::memcpy(&remote_state, msg->data.data() + OFF_STATE,
                    sizeof(remote_state));
        std::memcpy(&sender_id, msg->data.data() + OFF_SENDER,
                    sizeof(sender_id));
        std::memcpy(&pub_ns, msg->data.data() + OFF_PUBTIME,
                    sizeof(pub_ns));

        if (sender_id == agent_id_) return;
        if (sender_id < 0 || sender_id >= num_agents_) return;

        int64_t latency_ns = recv_ns - pub_ns;

        if (latency_file_.is_open()) {
            double latency_us = static_cast<double>(latency_ns) / 1000.0;
            latency_file_
                << recv_ns << ","
                << sender_id << ","
                << latency_us << ","
                << step_ << "\n";
        }

        state_ = state_ + alpha_ * (remote_state - state_) /
                 (num_agents_ - 1);

        last_state_[sender_id] = remote_state;
        has_state_[sender_id] = true;
        last_state_[agent_id_] = state_;

        recv_count_++;
    }

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
            "[agent_%d] step=%d state=%.4f local_var=%.4e "
            "known=%d recv=%lu",
            agent_id_, step_, state_, var, known, recv_count_);

        if (latency_file_.is_open()) latency_file_.flush();
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
    std::string latency_csv_;
    std::ofstream csv_file_;
    std::ofstream latency_file_;

    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr
        subscription_;
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