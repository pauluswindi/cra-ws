// Native DDS measurements with an optional target for the isolated MuJoCo trial.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "msgs/msg/human_capsule_array.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using Markers = visualization_msgs::msg::MarkerArray;

std::int64_t stampNs(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<std::int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
}

struct Stats
{
    std::size_t count = 0;
    double first_received = 0., last_received = 0., max_gap = 0.;
    std::vector<double> ages;

    void sample(double now, double age)
    {
        ++count;
        if (count == 1) { first_received = now; }
        if (last_received > 0.) { max_gap = std::max(max_gap, now - last_received); }
        last_received = now;
        if (std::isfinite(age)) { ages.push_back(age); }
    }

    void write(std::ostream& out, double seconds)
    {
        std::sort(ages.begin(), ages.end());
        out << "{\"count\":" << count << ",\"received_hz\":" << count / seconds
            << ",\"max_callback_gap_s\":" << max_gap << ",\"age_samples\":" << ages.size();
        for (const auto& p : std::map<std::string, double>{{"median_age_ms", .5},
                                                         {"p95_age_ms", .95}, {"max_age_ms", 1.}}) {
            out << ",\"" << p.first << "\":";
            if (ages.empty()) { out << "null"; }
            else { out << ages[static_cast<std::size_t>(p.second * (ages.size() - 1))]; }
        }
        out << '}';
    }
};

struct PhaseStats
{
    std::map<std::string, Stats> streams;
    std::map<std::string, double> geometry_seconds;
    double active_seconds = 0.;
};

struct TrialEvent
{
    double elapsed_s;
    std::string stream;
    std::string state;
};

class Probe : public rclcpp::Node
{
public:
    Probe(double warmup, double duration, int phases,
          const std::optional<geometry_msgs::msg::Point>& target, const std::string& control_file)
        : Node("vision_admission_stream_probe"),
        began_(Clock::now()), warmup_(warmup), duration_(duration), phases_(phases),
        control_file_(control_file)
    {
        const auto qos = rclcpp::SensorDataQoS();
        if (!control_file_.empty()) {
            depth_relay_ = create_publisher<sensor_msgs::msg::Image>("/tracker_trial/depth", qos);
        }
        depth_ = create_subscription<sensor_msgs::msg::Image>("/depth/image_raw", qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
                latest_depth_ = stampNs(m->header.stamp);
                depths_.push_back(latest_depth_);
                if (depths_.size() > 120) { depths_.pop_front(); }
                record("depth", &m->header.stamp);
                if (depth_relay_) {
                    if (relay_enabled_) { depth_relay_->publish(*m); }
                    else { ++depth_frames_withheld_; }
                }
            });
        rgb_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/rgb/image_raw/compressed", qos,
            [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {
                record("rgb", &m->header.stamp);
            });
        body_ = create_subscription<sensor_msgs::msg::Image>("/body_index_map/image_raw", qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
                record("body_index", &m->header.stamp);
                if (measuring() && latest_depth_ > 0) {
                    const auto stamp = stampNs(m->header.stamp);
                    latest_deltas_.push_back(std::abs(stamp - latest_depth_) / 1e6);
                    auto closest = std::abs(stamp - latest_depth_);
                    for (const auto depth : depths_) { closest = std::min(closest, std::abs(stamp - depth)); }
                    nearest_deltas_.push_back(closest / 1e6);
                }
            });
        tracking_ = create_subscription<Markers>("/body_tracking_data", qos,
            [this](Markers::ConstSharedPtr m) {
                record("body_packets", m->markers.empty() ? nullptr : &m->markers[0].header.stamp);
                if (measuring() && !m->markers.empty()) { ++body_packets_with_people_; }
            });
        skeleton_ = create_subscription<Markers>("/vision/skeleton/base_frame", 10,
            [this](Markers::ConstSharedPtr m) {
                record("skeleton", m->markers.empty() ? nullptr : &m->markers[0].header.stamp);
            });
        zone_ = create_subscription<Markers>("/perception/humans_in_zone", 10,
            [this](Markers::ConstSharedPtr m) {
                integrate();
                zone_active_ = std::any_of(m->markers.begin(), m->markers.end(), [](const auto& marker) {
                    return marker.action == visualization_msgs::msg::Marker::ADD
                           && marker.ns == "humans_in_zone";
                });
                record("zone", m->markers.empty() ? nullptr : &m->markers[0].header.stamp);
            });
        geometry_ = create_subscription<msgs::msg::HumanCapsuleArray>("/vision/human/geometry", 10,
            [this](msgs::msg::HumanCapsuleArray::ConstSharedPtr m) {
                integrate();
                const auto state = !m->valid ? "invalid" : m->capsules.empty() ? "empty" : "observed";
                if (target_pub_ && (state != geometry_state_ || !control_file_.empty())) {
                    event("vision", state);
                }
                geometry_state_ = state;
                geometry_received_s_ = elapsed();
                record("geometry_" + geometry_state_, &m->header.stamp);
            });
        if (target) {
            target_ = *target;
            target_pub_ = create_publisher<geometry_msgs::msg::Point>("/simulation/set_target", 10);
            const auto status_qos = rclcpp::QoS(1).reliable().transient_local();
            planning_ = create_subscription<std_msgs::msg::String>("/planning/status", status_qos,
                [this](std_msgs::msg::String::ConstSharedPtr m) { event("planning", m->data); });
            motion_ = create_subscription<std_msgs::msg::String>(
                "/motion/execution_status", status_qos,
                [this](std_msgs::msg::String::ConstSharedPtr m) { event("motion", m->data); });
            stop_ = create_subscription<std_msgs::msg::Bool>("/motion/stop_requested", 10,
                [this](std_msgs::msg::Bool::ConstSharedPtr m) {
                    event("command", m->data ? "STOP" : "RELEASE");
                });
            path_ = create_subscription<trajectory_msgs::msg::JointTrajectory>("/motion/joint_path", 10,
                [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr m) {
                    event("trajectory", std::to_string(m->points.size()));
                });
        }
    }

    double elapsed() const { return std::chrono::duration<double>(Clock::now() - began_).count(); }
    bool finished() const { return elapsed() >= warmup_ + duration_; }

    void announcePhase()
    {
        const double current = elapsed();
        if (phases_.empty() || current < warmup_ || current >= warmup_ + duration_) { return; }
        const int index = static_cast<int>((current - warmup_) / phaseDuration());
        if (index != announced_phase_) {
            announced_phase_ = index;
            std::cout << "PROBE_PHASE " << index << std::endl;
            if (target_pub_) { event("phase", std::to_string(index)); }
            if (target_pub_ && index == 1) {
                if (!zone_active_ || geometry_state_ != "observed"
                    || current - geometry_received_s_ > .5) {
                    event("target", "SKIPPED_VISION_NOT_READY");
                } else if (target_pub_->get_subscription_count() != 1) {
                    event("target", "SKIPPED_SIMULATION_SUBSCRIBER_COUNT");
                } else {
                    target_pub_->publish(target_);
                    event("target", "SET");
                }
            }
        }
    }

    void pollControl()
    {
        if (control_file_.empty() || elapsed() - last_control_poll_ < .02) { return; }
        last_control_poll_ = elapsed();
        std::ifstream input(control_file_);
        input.seekg(control_offset_);
        std::string command;
        while (std::getline(input, command)) {
            control_offset_ += static_cast<std::streamoff>(command.size() + 1);
            if (command == "TARGET" && !controlled_target_sent_) {
                controlled_target_sent_ = true;
                if (!zone_active_ || geometry_state_ != "observed"
                    || elapsed() - geometry_received_s_ > .5) {
                    event("target", "SKIPPED_VISION_NOT_READY");
                } else if (target_pub_->get_subscription_count() != 1) {
                    event("target", "SKIPPED_SIMULATION_SUBSCRIBER_COUNT");
                } else {
                    target_pub_->publish(target_);
                    event("target", "SET");
                }
            } else if (command == "DEPTH_OFF" || command == "DEPTH_ON") {
                relay_enabled_ = command == "DEPTH_ON";
                event("fault", command);
            }
        }
    }

    void write(const std::string& file)
    {
        integrate();
        std::ofstream out(file);
        out << std::setprecision(10) << "{\"warmup_s\":" << warmup_ << ",\"measured_s\":" << duration_
            << ",\"body_packets_with_people\":" << body_packets_with_people_
            << ",\"active_zone_s\":" << active_seconds_
            << ",\"observed_while_zone_active_s\":" << active_observed_seconds_
            << ",\"geometry_state_seconds\":{";
        bool first = true;
        for (const auto& value : state_seconds_) {
            if (!first) { out << ','; } first = false;
            out << '"' << value.first << "\":" << value.second;
        }
        out << "},\"streams\":{";
        first = true;
        for (auto& value : stats_) {
            if (!first) { out << ','; } first = false;
            out << '"' << value.first << "\":";
            value.second.write(out, duration_);
        }
        out << "},\"observer_latest_depth_body_delta\":";
        Stats latest, nearest;
        latest.ages = latest_deltas_; nearest.ages = nearest_deltas_;
        latest.count = latest.ages.size(); nearest.count = nearest.ages.size();
        latest.write(out, duration_);
        out << ",\"observer_nearest_depth_body_delta\":";
        nearest.write(out, duration_);
        out << ",\"phases\":[";
        for (std::size_t index = 0; index < phases_.size(); ++index) {
            if (index != 0) { out << ','; }
            writePhase(out, index);
        }
        out << ']';
        if (target_pub_) {
            out << ",\"simulation_target_base\":[" << target_.x << ',' << target_.y << ','
                << target_.z << "],\"simulation_events\":[";
            for (std::size_t i = 0; i < events_.size(); ++i) {
                if (i != 0) { out << ','; }
                const auto& entry = events_[i];
                out << "{\"elapsed_s\":" << entry.elapsed_s
                    << ",\"stream\":" << std::quoted(entry.stream)
                    << ",\"state\":" << std::quoted(entry.state) << '}';
            }
            out << "]";
        }
        out << ",\"depth_frames_withheld\":" << depth_frames_withheld_
            << ",\"pair_delta_note\":\"Observer receipt ordering; authoritative validity remains in ReconstructionNode\"}\n";
    }

private:
    void event(const std::string& stream, const std::string& state)
    {
        events_.push_back({elapsed(), stream, state});
        if (!control_file_.empty()) {
            const auto& entry = events_.back();
            std::cout << "PROBE_EVENT {\"elapsed_s\":" << std::setprecision(10) << entry.elapsed_s
                      << ",\"stream\":" << std::quoted(stream)
                      << ",\"state\":" << std::quoted(state) << '}' << std::endl;
        }
    }

    bool measuring() const { return elapsed() >= warmup_ && !finished(); }
    double phaseDuration() const { return duration_ / phases_.size(); }

    void writePhase(std::ostream& out, std::size_t index)
    {
        auto& phase = phases_[index];
        const double start = warmup_ + index * phaseDuration();
        const double end = start + phaseDuration();
        out << "{\"index\":" << index << ",\"duration_s\":" << phaseDuration()
            << ",\"active_zone_s\":" << phase.active_seconds << ",\"geometry_state_seconds\":{";
        bool first = true;
        for (const auto& value : phase.geometry_seconds) {
            if (!first) { out << ','; }
            first = false;
            out << '"' << value.first << "\":" << value.second;
        }
        out << "},\"streams\":{";
        first = true;
        for (auto& value : phase.streams) {
            if (!first) { out << ','; }
            first = false;
            out << '"' << value.first << "\":";
            value.second.write(out, phaseDuration());
        }
        out << "},\"longest_silence_s\":{";
        first = true;
        for (const auto* name : {"skeleton", "geometry_observed"}) {
            if (!first) { out << ','; }
            first = false;
            const auto& stream = phase.streams[name];
            const double silence = stream.count == 0 ? phaseDuration() : std::max({
                stream.max_gap, stream.first_received - start, end - stream.last_received});
            out << '"' << name << "\":" << silence;
        }
        out << "}}";
    }

    void record(const std::string& stream, const builtin_interfaces::msg::Time* stamp)
    {
        const double current = elapsed();
        if (current < warmup_ || current >= warmup_ + duration_) { return; }
        const double age = stamp && stampNs(*stamp) > 0
            ? (now().nanoseconds() - stampNs(*stamp)) / 1e6 : NAN;
        stats_[stream].sample(current, age);
        if (!phases_.empty()) {
            const auto index = static_cast<std::size_t>((current - warmup_) / phaseDuration());
            phases_[std::min(index, phases_.size() - 1)].streams[stream].sample(current, age);
        }
    }
    void integrate()
    {
        const double current = elapsed();
        const double dt = std::max(0., std::min(current, warmup_ + duration_)
                                  - std::max(last_integrated_, warmup_));
        state_seconds_[geometry_state_] += dt;
        if (zone_active_) {
            active_seconds_ += dt;
            if (geometry_state_ == "observed") { active_observed_seconds_ += dt; }
        }
        for (std::size_t index = 0; index < phases_.size(); ++index) {
            const double start = warmup_ + index * phaseDuration();
            const double end = start + phaseDuration();
            const double overlap = std::max(0., std::min(current, end)
                                               - std::max(last_integrated_, start));
            phases_[index].geometry_seconds[geometry_state_] += overlap;
            if (zone_active_) { phases_[index].active_seconds += overlap; }
        }
        last_integrated_ = current;
    }
    Clock::time_point began_;
    double warmup_, duration_, last_integrated_ = 0., active_seconds_ = 0., active_observed_seconds_ = 0.;
    bool zone_active_ = false;
    std::string geometry_state_ = "invalid";
    std::size_t body_packets_with_people_ = 0;
    std::int64_t latest_depth_ = 0;
    std::deque<std::int64_t> depths_;
    std::vector<double> latest_deltas_, nearest_deltas_;
    std::map<std::string, Stats> stats_;
    std::map<std::string, double> state_seconds_;
    std::vector<PhaseStats> phases_;
    int announced_phase_ = -1;
    double geometry_received_s_ = 0.;
    std::string control_file_;
    double last_control_poll_ = -1.;
    std::streamoff control_offset_ = 0;
    bool relay_enabled_ = true, controlled_target_sent_ = false;
    std::size_t depth_frames_withheld_ = 0;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_relay_;
    geometry_msgs::msg::Point target_;
    std::vector<TrialEvent> events_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planning_, motion_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr path_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_, body_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_;
    rclcpp::Subscription<Markers>::SharedPtr tracking_, skeleton_, zone_;
    rclcpp::Subscription<msgs::msg::HumanCapsuleArray>::SharedPtr geometry_;
};
}

int main(int argc, char** argv)
{
    if (!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) != "221") { return 2; }
    double warmup = 15., duration = 30.;
    int phases = 0;
    std::string output, control_file;
    std::map<std::string, double> target_coordinates;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string key = argv[i];
        if (key == "--warmup") { warmup = std::stod(argv[i + 1]); }
        else if (key == "--duration") { duration = std::stod(argv[i + 1]); }
        else if (key == "--phases") { phases = std::stoi(argv[i + 1]); }
        else if (key == "--control-file") { control_file = argv[i + 1]; }
        else if (key == "--target-x" || key == "--target-y" || key == "--target-z") {
            target_coordinates[key] = std::stod(argv[i + 1]);
        }
        else if (key == "--output") { output = argv[i + 1]; }
        else { return 2; }
    }
    if (output.empty() || warmup < 0. || warmup > 60. || duration < 1. || duration > 120.) { return 2; }
    if (phases < 0 || phases > 10) { return 2; }
    std::optional<geometry_msgs::msg::Point> target;
    if (!target_coordinates.empty()) {
        if (target_coordinates.size() != 3 || (phases < 2 && control_file.empty())
            || !std::getenv("ROS_LOCALHOST_ONLY")
            || std::string(std::getenv("ROS_LOCALHOST_ONLY")) != "1"
            || !std::all_of(target_coordinates.begin(), target_coordinates.end(),
                             [](const auto& item) { return std::isfinite(item.second); })) { return 2; }
        target.emplace();
        target->x = target_coordinates.at("--target-x");
        target->y = target_coordinates.at("--target-y");
        target->z = target_coordinates.at("--target-z");
    }
    if (!control_file.empty() && (!target || phases != 0)) { return 2; }
    rclcpp::init(argc, argv);
    auto probe = std::make_shared<Probe>(warmup, duration, phases, target, control_file);
    while (rclcpp::ok() && !probe->finished()) {
        rclcpp::spin_some(probe);
        probe->pollControl();
        probe->announcePhase();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    probe->write(output);
    probe.reset();
    rclcpp::shutdown();
}
