#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <stdio.h>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#define PI 3.14159

// ANSI escape codes for colors
#define GREEN_TEXT "\033[0;32m"
#define BLUE_TEXT "\033[0;34m"
#define RESET_COLOR "\033[0m"

using std::placeholders::_1;
using namespace std::chrono_literals;

class JointStateSubscriber : public rclcpp::Node
{
  public:
    JointStateSubscriber()
    : Node("state_space_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&JointStateSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("effort_controller/commands", 10);
      publisher_position_ = this->create_publisher<std_msgs::msg::Float64MultiArray>( "position_controller/commands", 10);

    }

  private:
    void topic_callback(const sensor_msgs::msg::JointState & msg)
    {
        // Find the indices of the "slider" and "swinger" joints
        auto slider_it = std::find(msg.name.begin(), msg.name.end(), "slider_1");
        auto swinger1_it = std::find(msg.name.begin(), msg.name.end(), "continuous_revolute_1");
        auto swinger2_it = std::find(msg.name.begin(), msg.name.end(), "continuous_revolute_2");

        
            double slider_position;
            double slider_velocity;

            
            double swinger1_position;
            double swinger1_velocity;

            
            double swinger2_position;
            double swinger2_velocity;


        if (slider_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), slider_it);
            slider_position = msg.position[index];
            slider_velocity = msg.velocity[index];

            //RCLCPP_INFO(this->get_logger(), "Slider  -> Position: %.2f, Velocity: %.2f", slider_position, slider_velocity);
        }

        else
        {
            RCLCPP_WARN(this->get_logger(), "Slider joint not found in the message.");
        }

        if (swinger1_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger1_it);
            swinger1_position = msg.position[index];
            swinger1_velocity = msg.velocity[index];

            //RCLCPP_INFO(this->get_logger(), "Swinger -> Position: %.2f, Velocity: %.2f", swinger_position, swinger_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger1 joint not found in the message.");
        }

        if(swinger2_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger2_it);
            swinger2_position = msg.position[index];
            swinger2_velocity = msg.velocity[index];

            //RCLCPP_INFO(this->get_logger(), "Swinger -> Position: %.2f, Velocity: %.2f", swinger_position, swinger_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger2 joint not found in the message.");
        }

        double f;

        if((slider_it != msg.name.end()) || (swinger1_it != msg.name.end()) || (swinger2_it != msg.name.end()))
        {
              // Define the range boundaries
              double lower_limit = PI - 0.523599;
              double upper_limit = PI + 0.523599;

                  // Normalize swinger_position to [0, 2*PI]
              double normalized_position = fmod(swinger2_position, 2 * PI);

                  // If the normalized position is negative, adjust it by adding 2*PI
              if (normalized_position < 0) {
                  normalized_position += 2 * PI;
              }        
         //RCLCPP_INFO(this->get_logger(), "normalised_position: %.4f", normalized_position);

         //Before the pendulum is in the homoclinic orbit
        if( normalized_position < lower_limit || normalized_position > upper_limit)
        {
                // Control parameters
                /*--------------------------------------------------------------------------------*/
                //THIS SHOULD BE ADDED TO A YAML FILE
                double kd = 1.0;
                double ke = 0.001;
                double kp = 100.0;
                /*-------------------------------------------------------------------------------*/

                double mass_1 = 0.48763073080899393963;
                double mass_2 = 0.48763073080899393963;
                double lenght_1 = 0.4;
                double lenght_2 = 0.4;
                double lenghtc1 = lenght_1/2;
                double lengthc2 = lenght_2/2;
                double g = 9.81;

                double inertia1 = 150.0;
                double inertia2 = 150.0;

                double q1 = M_PI_2 + swinger1_position; //Angle that link 1 makes with the horizontal
                double q2 = M_PI_2 + swinger1_position + swinger2_position; //Angle that link 2 makes with link 1

                double q_dash = (q1 - M_PI_2);
                double E_dash = 0.0;

                double dq1 = swinger1_velocity; //
                double dq2 = swinger2_velocity; //Angle that link

                double theta_design_1 = mass_1 * lenghtc1 * lenghtc1 + mass_2 * lenght_1 * lenght_1 + inertia1;
                double theta_design_2 = mass_2 * lengthc2 * lengthc2 + inertia2;
                double theta_design_3 = mass_2 * lenght_1 * lengthc2;
                double theta_design_4 = mass_1 * lenghtc1 + mass_2 * lenght_1;
                double theta_design_5 = mass_2 * lengthc2;

                double force = theta_design_2 * theta_design_3 * sin(q2) * (dq1 + dq2) * (dq1 + dq2) + theta_design_3 * theta_design_3 * cos(q2) * sin(q2) * (dq1 * dq1) - theta_design_2 * theta_design_4 * g * cos(q1) + theta_design_3 * theta_design_5 * g * cos(q2) * cos(q1 + q2);
                double torque_numerator = -kd * force - (theta_design_1 * theta_design_2 - theta_design_3 * theta_design_3 * cos(q2)*cos(q2))*(dq1 + kp * q_dash);
                double torque_denominator = (theta_design_1 * theta_design_2 - theta_design_3 * theta_design_3 * cos(q2) * cos(q2)) * ke * E_dash + kd * theta_design_2;

                double torque = torque_numerator / torque_denominator;

                f = torque;
                RCLCPP_INFO(this->get_logger(), BLUE_TEXT"F: %.4f", f);
            
            }

            else 
            {
                double gains_[4] = { -141.4214, -77.6558, -238.7684, -36.5906};

                double from_output = (gains_[2]*swinger2_position);
                double first = (gains_[0]*swinger1_position) + (gains_[1]*swinger1_velocity) + (gains_[3]*swinger2_velocity);

                double final_gain = (from_output - first);
                f = final_gain;
                RCLCPP_INFO(this->get_logger(), GREEN_TEXT"F: %.4f", f);

            }

                auto message = std_msgs::msg::Float64MultiArray();
                message.data.push_back(f);
                publisher_->publish(message);

                auto position_message = std_msgs::msg::Float64MultiArray();
                position_message.data.push_back(0.0);
                publisher_position_->publish(position_message);
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_; // effort controller publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_position_; // effort controller publisher
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateSubscriber>());
  rclcpp::shutdown();
  return 0;
}