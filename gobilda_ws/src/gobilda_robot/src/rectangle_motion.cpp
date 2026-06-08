#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
// #include <chrono>
// #include <thread>

// FSM states
enum class MovementState {
    FORWARD,
    STOPPING_BEFORE_TURN,
    STOPPING_BEFORE_FORWARD,
    TURNING,
    DONE
};

// creating a class that inherits from rclcpp::Node
class RectangleMotion : public rclcpp::Node {
    public:
        // constructor initializer list (calls the parent node class constructor with the string "rectangle_motion", which becomes the nodes name in the ROS2 graph)
        RectangleMotion() : Node("rectangle_motion") {
            // creating a publisher (geometry_msgs::msg::TwistStamped is the message type, /gobilda/cmd_vel is the topic name, 10 is the queue size)
            publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/gobilda/cmd_vel", 10);

            // creating the subscriber (receives odometry feedback)
            subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/gobilda_base_controller/odom", 10, 
                                                                            std::bind(&RectangleMotion::odom_callback, this, std::placeholders::_1));

            // creating the timer for FSM (~20Hz)
            timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&RectangleMotion::timer_callback, this));

            RCLCPP_INFO(this->get_logger(), "Rectangle Motion has started, waiting for odom");
        }

        // // function to run the rectangle
        // void run_rectangle(){
        //     std::cout << "Running rectangle motion..." << std::endl;

        //     // going forward ~1 meters (takes 5 seconds)
        //     publish_command(0.2, 0.0, 5000);

        //     // stop for about 500 ms
        //     publish_command(0.0, 0.0, 500);

        //     // turn to the left 90 degrees
        //     publish_command(0.1, 0.3, 4000);

        //     // stop for about 500 ms
        //     publish_command(0.0, 0.0, 500);

        //     // going forward ~0.4 meters
        //     publish_command(0.1, 0.0, 4000);

        //     // stop for about 500 ms
        //     publish_command(0.0, 0.0, 500);

        //     // turn to the left 90 degrees
        //     publish_command(0.0, 0.3, 5240);

        //     // stop for about 500 ms
        //     publish_command(0.0, 0.0, 500);

        //     // going forward ~3 meters
        //     publish_command(0.3, 0.0, 10000);

        //     // stop for about 500 ms
        //     publish_command(0.0, 0.0, 500);

        //     // turn to the left 90 degrees
        //     publish_command(0.0, 0.3, 5240);

        //     // stop for about 500 ms
        //     publish_command(0.0, 0.0, 500);

        //     // going forward ~0.9 meters
        //     publish_command(0.3, 0.0, 3000);

        //     // stop for about 500 ms
        //     publish_command(0.0, 0.0, 500);

        //     // turn to the left 90 degrees
        //     publish_command(0.0, 0.3, 5240);

        //     // stop for about 500 ms
        //     publish_command(0.0, 0.0, 500);  
        //}
        

    private:

        // ----- Member Variables -------

        // declaring the publisher, subsriber, and timer as a member variable using a shared pointer
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;  
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;  
        rclcpp::TimerBase::SharedPtr timer_;

        // FSM state
        MovementState state_ = MovementState::FORWARD;

        // current odometry
        double current_x_ = 0.0;
        double current_y_ = 0.0;
        double current_yaw_ = 0.0;
        bool init_ = false;

        // snapshot start of each phase
        double start_x_ = 0.0;
        double start_y_ = 0.0;
        double start_yaw_ = 0.0;

        // Alternate the rectangle between 1m and 0.5m
        const double side_lengths_[4] = {1.0,0.5,1.0,0.5};
        int sides_completed_ = 0;
        int turns_completed_ = 0;

        // Stop phase timer
        int stop_ticks_ = 0;
        const int STOP_TICKS = 10;
        
        
        // shortest signed angle difference
        double angle_diff(double a, double b){
            return std::fmod((a - b) + M_PI, 2.0 * M_PI) - M_PI;
        }

        // Odom callback function
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){

            current_x_ = msg->pose.pose.position.x;
            current_y_ = msg->pose.pose.position.y;

            // get yaw from quaternion
            double qx = msg->pose.pose.orientation.x;
            double qy = msg->pose.pose.orientation.y;
            double qz = msg->pose.pose.orientation.z;
            double qw = msg->pose.pose.orientation.w;

            // quaternion to yaw conversion
            double siny_cosp = 2.0 * (qw * qz + qx * qy);
            double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

            if (!init_){

                // Get the starting position on first odom message
                start_x_ = current_x_;
                start_y_ = current_y_;
                start_yaw_ = current_yaw_;
                init_ = true;
                RCLCPP_INFO(this->get_logger(), "Odometry received. Starting rectangle.");
            }
        }

        // publishing a twist command
        void publish_command(float linear_x, float angular_z){
            geometry_msgs::msg::TwistStamped msg;
            msg.header.stamp = this->now();
            msg.twist.linear.x = linear_x;
            msg.twist.angular.z = angular_z;
            publisher_->publish(msg);   
        }


        // ---- FSM timer callback (20Hz) ----
        void timer_callback() {
 
            // Wait for first odom message
            if (!init_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Waiting for odometry...");
                return;
            }
    
            switch (state_) {
    
                case MovementState::FORWARD: {
                    double target_dist = side_lengths_[sides_completed_ % 4];
                    double dist = std::hypot(current_x_ - start_x_, current_y_ - start_y_);
 
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "FORWARD: %.2f / %.2f m", dist, target_dist);
 
                    if (dist < target_dist) {
                        publish_command(0.3, 0.0);
                    } else {
                        // reached target distance — stop before turning
                        publish_command(0.0, 0.0);
                        sides_completed_++;
                        stop_ticks_ = 0;
                        state_ = MovementState::STOPPING_BEFORE_TURN;
                        RCLCPP_INFO(this->get_logger(),
                            "Side %d complete. Stopping before turn.", sides_completed_);
                    }
                    break;
                }
    
                case MovementState::STOPPING_BEFORE_TURN: {
                    publish_command(0.0, 0.0);
                    stop_ticks_++;
 
                    if (stop_ticks_ >= STOP_TICKS) {
                        // snapshot heading and start turning
                        start_yaw_ = current_yaw_;
                        state_ = MovementState::TURNING;
                        RCLCPP_INFO(this->get_logger(), "Starting turn %d.", turns_completed_ + 1);
                    }
                    break;
                }
    
                case MovementState::TURNING: {
                    double turned = std::abs(angle_diff(current_yaw_, start_yaw_));
 
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "TURNING: %.3f / %.3f rad", turned, M_PI / 2.0);
 
                    if (turned < M_PI / 2.0) {
                        publish_command(0.0, 0.4);
                    } else {
                        // reached 90 degrees — stop before next forward
                        publish_command(0.0, 0.0);
                        turns_completed_++;
                        stop_ticks_ = 0;
                        state_ = MovementState::STOPPING_BEFORE_FORWARD;
                        RCLCPP_INFO(this->get_logger(),
                            "Turn %d complete. Stopping before next side.", turns_completed_);
                    }
                    break;
                }

                case MovementState::STOPPING_BEFORE_FORWARD: {
                    publish_command(0.0, 0.0);
                    stop_ticks_++;
 
                    if (stop_ticks_ >= STOP_TICKS) {
                        if (turns_completed_ >= 4) {
                            // all 4 turns done — rectangle complete
                            state_ = MovementState::DONE;
                            RCLCPP_INFO(this->get_logger(), "Rectangle complete!");
                        } else {
                            // snapshot position and start next side
                            start_x_ = current_x_;
                            start_y_ = current_y_;
                            state_ = MovementState::FORWARD;
                            RCLCPP_INFO(this->get_logger(),
                                "Starting side %d.", sides_completed_ + 1);
                        }
                    }
                    break;
                }
    
                case MovementState::DONE: {
                    publish_command(0.0, 0.0);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Rectangle complete. Robot stopped.");
                    break;
                }
            }
        }
};


// main that calls rectangle movment
int main(int argc, char* argv[]){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<RectangleMotion>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}