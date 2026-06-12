// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simple_pure_pursuit.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>


namespace autoware::control::simple_pure_pursuit
{

    using autoware_control_msgs::msg::Control;
    using autoware_planning_msgs::msg::Trajectory;
    using autoware_planning_msgs::msg::TrajectoryPoint;
    using nav_msgs::msg::Odometry;

    namespace
    {

        constexpr auto connection_timeout = std::chrono::seconds(3);
        constexpr auto output_timeout = std::chrono::seconds(5);
        constexpr auto no_output_timeout = std::chrono::milliseconds(250);
        constexpr auto spin_sleep = std::chrono::milliseconds(10);


        /**
        * @brief Create a dummy Odometry message with some given params.
        * This dummy one is a pose on the x-y plane with a yaw angle, and a longitudinal velocity.
        *
        * @param x dummy odometry's x position.
        * @param y dummy odometry's y position.
        * @param yaw dummy odometry's yaw angle.
        * @param longitudinal_velocity dummy odometry's longitudinal velocity.
        * @param stamp dummy odometry's timestamp.
        *
        * @return Odometry dummy message.
        */
        Odometry make_odometry(
            const double x, 
            const double y, 
            const double yaw, 
            const double longitudinal_velocity,
            const builtin_interfaces::msg::Time & stamp
        ) {
            
            Odometry odom;
            odom.header.frame_id = "map";
            odom.header.stamp = stamp;
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
            odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
            odom.twist.twist.linear.x = longitudinal_velocity;

            return odom;

        };

        /**
        * @brief Create a dummy Trajectory message with some given params.
        * This dummy one is a straight line along x-axis,
        * so all y's are 0.0 and all yaw angles are 0.0.
        *
        * @param x_positions dummy trajectory's x positions.
        * @param longitudinal_velocity dummy trajectory's longitudinal velocity.
        * @param stamp dummy trajectory's timestamp.
        *
        * @return Trajectory dummy message.
        */
        Trajectory make_straight_trajectory(
            const std::vector<double> & x_positions,
            const double longitudinal_velocity,
            const builtin_interfaces::msg::Time & stamp
        ) {

            Trajectory traj;
            traj.header.frame_id = "map";
            traj.header.stamp = stamp;

            for (const auto x : x_positions) {
                TrajectoryPoint point;
                point.pose.position.x = x;
                point.pose.position.y = 0.0;
                point.longitudinal_velocity_mps = static_cast<float>(longitudinal_velocity);
                traj.points.push_back(point);
            }

            return traj;

        };

        /**
        * @brief Create a NodeOptions with some given overrides and some default config files.
        *
        * @param overrides a vector of pairs of <param name, param value> to override default ones.
        *
        * @return NodeOptions with given overrides and default config files.
        */
        rclcpp::NodeOptions make_node_options(
            const std::vector<std::pair<std::string, rclcpp::ParameterValue>> & overrides = {}
        ) {
            
            const auto autoware_test_utils_dir =
                ament_index_cpp::get_package_share_directory("autoware_test_utils");
            const auto autoware_simple_pure_pursuit_dir =
                ament_index_cpp::get_package_share_directory("autoware_simple_pure_pursuit");

            auto node_options = rclcpp::NodeOptions{};
            autoware::test_utils::updateNodeOptions(
                node_options,
                {
                    autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
                    autoware_simple_pure_pursuit_dir + "/config/simple_pure_pursuit.param.yaml"
                }
            );

            for (const auto & [name, value] : overrides) {
                node_options.append_parameter_override(name, value);
            }

            return node_options;

        };


        class SimplePurePursuitIntegrationHarness
        {
            
            public:

                /**
                * @brief Construct a new SimplePurePursuitIntegrationHarness object, 
                        which sets up test env for integration testing of SimplePurePursuitNode.
                *
                * @param node_options NodeOptions to construct target SimplePurePursuitNode, 
                                    can be used to override some default params if needed.
                */
                explicit SimplePurePursuitIntegrationHarness(
                    const rclcpp::NodeOptions & node_options
                ) {
                    
                    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

                    target_node_ = std::make_shared<SimplePurePursuitNode>(node_options);
                    input_pub_node_ = rclcpp::Node::make_shared("simple_pure_pursuit_test_input_publisher");
                    output_sub_node_ = rclcpp::Node::make_shared("simple_pure_pursuit_test_output_subscriber");

                    odom_pub_ = input_pub_node_ -> create_publisher<Odometry>(
                        "/simple_pure_pursuit/input/odometry", 
                        rclcpp::QoS{1}
                    );
                    traj_pub_ = input_pub_node_ -> create_publisher<Trajectory>(
                        "/simple_pure_pursuit/input/trajectory", 
                        rclcpp::QoS{1}
                    );
                    control_sub_ = output_sub_node_ -> create_subscription<Control>(
                        "/simple_pure_pursuit/output/control_command", 
                        rclcpp::QoS{1}.transient_local(),
                        [this](const Control::SharedPtr msg) {
                            {
                            std::scoped_lock lock(received_control_mutex_);
                            received_control_ = msg;
                            }
                            is_received_.store(true);
                        }
                    );

                    executor_ -> add_node(target_node_);
                    executor_ -> add_node(input_pub_node_);
                    executor_ -> add_node(output_sub_node_);

                    // Spin it up!
                    executor_thread_ = std::thread([this]() { executor_ -> spin(); });
                    (void)wait_for_connections(connection_timeout);

                };

                // Delete copy and move constructors/assignment operators
                SimplePurePursuitIntegrationHarness(const SimplePurePursuitIntegrationHarness &) = delete;
                SimplePurePursuitIntegrationHarness & operator=(const SimplePurePursuitIntegrationHarness &) = delete;
                SimplePurePursuitIntegrationHarness(SimplePurePursuitIntegrationHarness &&) = delete;
                SimplePurePursuitIntegrationHarness & operator=(SimplePurePursuitIntegrationHarness &&) = delete;

                // Just simple destructor to clean up env resources
                ~SimplePurePursuitIntegrationHarness()
                {
                    
                    executor_ -> cancel();
                    if (executor_thread_.joinable()) {
                        executor_thread_.join();
                    }

                    control_sub_.reset();
                    traj_pub_.reset();
                    odom_pub_.reset();
                    output_sub_node_.reset();
                    input_pub_node_.reset();
                    target_node_.reset();
                    executor_.reset();

                };

                // Get time now from input_pub_node's clock
                [[nodiscard]] builtin_interfaces::msg::Time now() const
                {
                    return input_pub_node_ -> get_clock() -> now();
                };

                // Funcs to publish odometry, trajectory or both as inputs to target node
                void publish_odometry(
                    const Odometry & odom
                ) {
                    clear_received_control();
                    odom_pub_ -> publish(odom);
                };

                void publish_trajectory(
                    const Trajectory & traj
                ) {
                    clear_received_control();
                    traj_pub_ -> publish(traj);
                };

                void publish_inputs(
                    const Odometry & odom, 
                    const Trajectory & traj
                ) {
                    clear_received_control();
                    odom_pub_ -> publish(odom);
                    traj_pub_ -> publish(traj);
                };

                // Return true if received control command within timeout, false otherwise
                [[nodiscard]] bool wait_for_output(
                    const std::chrono::milliseconds timeout
                ) const {
                    
                    const auto start = std::chrono::steady_clock::now();

                    while (!is_received_.load()) {
                        if (std::chrono::steady_clock::now() - start > timeout) {
                            return false;
                        }
                        std::this_thread::sleep_for(spin_sleep);
                    }

                    return true;
                    
                };

                // Return true if control command received
                [[nodiscard]] Control::SharedPtr received_control() const
                {
                    std::scoped_lock lock(received_control_mutex_);
                    return received_control_;
                };

            private:
                
                // Simple nuke received control command to prepare for next test case
                void clear_received_control()
                {
                    is_received_.store(false);
                    std::scoped_lock lock(received_control_mutex_);
                    received_control_.reset();
                };

                // Wait until all publishers and subscribers are connected or timeout happens, 
                // return true if all connected, false otherwise
                bool wait_for_connections(
                    const std::chrono::milliseconds timeout
                ) {
                    
                    const auto start = std::chrono::steady_clock::now();
                    // Wait until all connections are established or timeout happens
                    while (
                        (
                            (odom_pub_ -> get_subscription_count() == 0U) || 
                            (traj_pub_ -> get_subscription_count() == 0U) ||
                            (control_sub_ -> get_publisher_count() == 0U)
                        ) && (
                            (std::chrono::steady_clock::now() - start <= timeout)
                        )
                    ) {
                        std::this_thread::sleep_for(spin_sleep);
                    }

                    return (
                        (odom_pub_ -> get_subscription_count() > 0U) && 
                        (traj_pub_ -> get_subscription_count() > 0U) &&
                        (control_sub_->get_publisher_count() > 0U)
                    );

                };

                // Consts for connection and receiving timeouts and spin sleep duration
                std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_{nullptr};
                std::thread executor_thread_;

                std::shared_ptr<SimplePurePursuitNode> target_node_{nullptr};
                rclcpp::Node::SharedPtr input_pub_node_{nullptr};
                rclcpp::Node::SharedPtr output_sub_node_{nullptr};
                rclcpp::Publisher<Odometry>::SharedPtr odom_pub_{nullptr};
                rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_{nullptr};
                rclcpp::Subscription<Control>::SharedPtr control_sub_{nullptr};

                mutable std::mutex received_control_mutex_;
                std::atomic_bool is_received_{false};
                Control::SharedPtr received_control_;

            };

            // ================== TESTING AREA HERE ==================

            // TEST 1. Standard happy case test:
            // - Straight trajectory
            // - Vehicle starts at beginning, 0 yaw, 1 m/s velocity
            // => Expect to receive 1 m/s velocity, and 0 steering angle within timeout
            TEST(
                SimplePurePursuitIntegrationTest, 
                PublishesControlCommandForStraightTrajectory
            ) {

                SimplePurePursuitIntegrationHarness harness(make_node_options());

                const auto stamp = harness.now();
                const auto odom = make_odometry(
                    0.0, 0.0, 0.0, 0.0, 
                    stamp
                );
                const auto traj = make_straight_trajectory(
                    {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}, 
                    1.0, 
                    stamp
                );

                harness.publish_inputs(
                    odom, 
                    traj
                );

                // Receive control command within timeout
                ASSERT_TRUE(
                    harness.wait_for_output(
                        std::chrono::duration_cast<std::chrono::milliseconds>(output_timeout)
                    )
                );
                
                // Check non-nullptr and expected values 
                const auto control = harness.received_control();
                ASSERT_NE(
                    control, 
                    nullptr
                );

                // Check expected values
                EXPECT_EQ(
                    control->stamp, 
                    stamp
                );
                EXPECT_FLOAT_EQ(
                    control->longitudinal.velocity, 
                    1.0F
                );
                EXPECT_FLOAT_EQ(
                    control->longitudinal.acceleration, 
                    1.0F
                );
                EXPECT_FLOAT_EQ(
                    control->lateral.steering_tire_angle, 
                    0.0F
                );

            };

            // TEST 2. Synchronization safety check
            // Same standard happy case as above, but delayed between odom and traj publishes
            TEST(
                SimplePurePursuitIntegrationTest, 
                DoesNotPublishUntilBothInputsAreAvailable
            ) {
                
                SimplePurePursuitIntegrationHarness harness(make_node_options());

                const auto stamp = harness.now();
                const auto odom = make_odometry(
                    0.0, 0.0, 0.0, 0.0, 
                    stamp
                );
                const auto traj = make_straight_trajectory(
                    {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}, 
                    1.0, 
                    stamp
                );

                // Publish odom first
                // Expect no received command cuz traj is not published yet
                harness.publish_odometry(odom);
                EXPECT_FALSE(
                    harness.wait_for_output(
                        std::chrono::duration_cast<std::chrono::milliseconds>(no_output_timeout)
                    )
                );

                // Then publish traj
                // Expect to receive command within timeout
                harness.publish_inputs(
                    odom, 
                    traj
                );
                ASSERT_TRUE(
                    harness.wait_for_output(
                        std::chrono::duration_cast<std::chrono::milliseconds>(output_timeout)
                    )
                );

            };

            // TEST 3. Goal stop command test
            // Same straight traj, vehicle starts at traj end
            // Expected 0 velocity
            TEST(
                SimplePurePursuitIntegrationTest, 
                PublishesGoalStopCommandAtTrajectoryEnd
            ) {
            
                SimplePurePursuitIntegrationHarness harness(make_node_options());

                const auto stamp = harness.now();
                const auto odom = make_odometry(
                    6.0, 0.0, 0.0, 0.0, 
                    stamp
                );
                const auto traj = make_straight_trajectory(
                    {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}, 
                    1.0, 
                    stamp
                );

                harness.publish_inputs(odom, traj);

                // Receive control command within timeout
                ASSERT_TRUE(
                    harness.wait_for_output(
                        std::chrono::duration_cast<std::chrono::milliseconds>(output_timeout)
                    )
                );

                // Check non-nullptr and expected values
                const auto control = harness.received_control();
                ASSERT_NE(control, nullptr);

                // Expected 0 velocity
                EXPECT_FLOAT_EQ(
                    control->longitudinal.velocity, 
                    0.0F
                );

                // Expected strong deceleration
                // Here I already checked during development that terminal decel value is -10
                // Seems like this module hard-coded this value. Imma freezin it here as characterization test
                EXPECT_FLOAT_EQ(
                    control->longitudinal.acceleration, 
                    -10.0F
                );
                EXPECT_TRUE(control->longitudinal.is_defined_acceleration);

            };

            // TEST 4. Empty trajectory safety check
            // Same vehicle state as TEST 1 & 2 but empty trajectory
            // Expect to receive 0 velocity command within timeout (at least it ain't no command or crash)
            TEST(
                SimplePurePursuitIntegrationTest, 
                PublishesZeroCommandForEmptyTrajectory
            ) {
                
                SimplePurePursuitIntegrationHarness harness(make_node_options());

                const auto stamp = harness.now();
                const auto odom = make_odometry(
                    0.0, 0.0, 0.0, 0.0, 
                    stamp
                );
                // Empty traj
                Trajectory traj;
                traj.header.frame_id = "map";
                traj.header.stamp = stamp;
                
                harness.publish_inputs(odom, traj);

                // Check receiving control command within timeout
                ASSERT_TRUE(
                    harness.wait_for_output(
                        std::chrono::duration_cast<std::chrono::milliseconds>(output_timeout)
                    )
                );

                // Check non-nullptr and expected values
                const auto control = harness.received_control();
                ASSERT_NE(control, nullptr);

                // Expected 0 velocity and 0 accel
                EXPECT_FLOAT_EQ(
                    control->longitudinal.velocity, 
                    0.0F
                );
                EXPECT_FLOAT_EQ(
                    control->longitudinal.acceleration, 
                    0.0F
                );

            };

            // TEST 5. External target velocity test
            // Same straight traj and vehicle state as TEST 1, but with external target velocity enabled at 2.5 m/s
            // Expect to receive 2.5 m/s velocity command within timeout (that ext. vel is used properly)
            TEST(
                SimplePurePursuitIntegrationTest, 
                UsesExternalTargetVelocityWhenEnabled
            ) {
                
                SimplePurePursuitIntegrationHarness harness(
                    make_node_options({
                        {"use_external_target_vel", rclcpp::ParameterValue{true}},
                        {"external_target_vel", rclcpp::ParameterValue{2.5}},
                    })
                );

                const auto stamp = harness.now();
                const auto odom = make_odometry(
                    0.0, 0.0, 0.0, 0.5, 
                    stamp
                );
                const auto traj = make_straight_trajectory(
                    {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}, 
                    1.0, 
                    stamp
                );

                harness.publish_inputs(odom, traj);

                // Check receiving control command within timeout
                ASSERT_TRUE(
                    harness.wait_for_output(
                        std::chrono::duration_cast<std::chrono::milliseconds>(output_timeout)
                    )
                );

                const auto control = harness.received_control();

                // Check non-nullptr and expected values
                ASSERT_NE(control, nullptr);

                // Expected ext. vel 2.5 m/s, not traj's 1.0 m/s
                EXPECT_FLOAT_EQ(
                    control->longitudinal.velocity, 
                    2.5F
                );

                // Car goin 0.5 m/s, target 2.5 m/s, so expect accel 2.0 m/s^2
                // Already confirmed via test run during dev
                EXPECT_FLOAT_EQ(
                    control->longitudinal.acceleration, 
                    2.0F
                );

                // Expected 0 steering angle for straight traj
                EXPECT_FLOAT_EQ(
                    control->lateral.steering_tire_angle, 
                    0.0F
                );

            };

    }; // namespace

}; // namespace autoware::control::simple_pure_pursuit

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);

    const auto result = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return result;

}