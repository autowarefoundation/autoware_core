#include "generic_buffer.hpp"
#include <vector>
#include <optional>
#include <limits>
#include <cmath>
#include <chrono>
#include "motion_utils/trajectory/interpolation.hpp"
#include <functional>

struct BufferedSlowdownPose {
    double arc_length;
    TimePoint start_time;
    bool is_active;

    BufferedSlowdownPose(double arc, TimePoint time, const geometry_msgs::msg::Pose& d, bool active = false)
        : arc_length(arc), start_time(time), data(d), is_active(active) {}
};

class PathLengthBuffer : public GenericBuffer<BufferedSlowdownPose> {
public:

    PathLengthBuffer() = default;
    PathLengthBuffer(double update_distance_threshold, double min_off_duration, double min_on_duration)
        : GenericBuffer<BufferedSlowdownPose>(min_off_duration, min_on_duration),
          update_distance_th_(update_distance_threshold) {}

    virtual std::optional<geometry_msgs::msg::Pose> get_nearest_active_item() const override
    {
        if (buffer_.empty()) {
            return {};
        }

        auto nearest_item = std::min_element(
            buffer_.begin(), buffer_.end(),
            [](const BufferedSlowdownPose& a, const BufferedSlowdownPose& b) {
                if (!a.is_active) return false;
                if (!b.is_active) return true;
                return a.arc_length < b.arc_length;
            });

        if (nearest_item == buffer_.end() || !nearest_item->is_active) {
            return {};
        }

        return nearest_item->arc_length;
    }

    void update_buffer(
        const double pose_arc_length,
        const std::chrono::system_clock& clock)
    {
        TimePoint clock_now = clock.now();

        // Remove items that should be removed
        buffer_.erase(std::remove_if(
            buffer_.begin(), buffer_.end(),
            [&](const BufferedSlowdownPose& buffered_item) {
                const auto duration = (clock_now - buffered_item.start_time).seconds();
                return (buffered_item.is_active && duration > min_off_duration_) ||
                       (!buffered_item.is_active && std::abs(buffered_item.arc_length - pose_arc_length) > update_distance_th_);
            }), buffer_.end()
        );

        static constexpr double eps = 1e-3;
        if (buffer_.empty()) {
            buffer_.emplace_back(pose_arc_length, clock_now, min_on_duration_ < eps);
            return;
        }

        // Update the state of remaining items
        for (auto& buffered_item : buffer_) {
            if (!buffered_item.is_active && (clock_now - buffered_item.start_time).seconds() > min_on_duration_) {
            buffered_item.is_active = true;
            buffered_item.start_time = clock_now;
            }
        }

        auto nearest_prev_pose_it = std::min_element(
            buffer.begin(), buffer.end(),
            [&](const BufferedSlowdownPose& a, const BufferedSlowdownPose& b) {
                const auto dist_a = std::abs(a.arc_length - pose_arc_length);
                const auto dist_b = std::abs(b.arc_length - pose_arc_length);
                return dist_a < dist_b;
        });

        if (nearest_prev_pose_it == buffer_.end()) {
            buffer.emplace_back(pose_arc_length, clock_now, min_on_duration_ < eps);
            return;
        }

        const double min_relative_dist = nearest_prev_pose_it->arc_length - pose_arc_length;

        if (min_relative_dist > 0) {
            nearest_prev_pose_it->pose = *slowdown_pose;
            nearest_prev_pose_it->arc_length = slowdown_pose_arc_length;
        }

        if (nearest_prev_pose_it->is_active) {
            nearest_prev_pose_it->start_time = clock_->now();
        }
    }

private:
    double update_distance_th_;
};