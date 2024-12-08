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
#define RED_TEXT "\033[0;31m"
#define RESET_COLOR "\033[0m"

using std::placeholders::_1;
using namespace std::chrono_literals;

class JointStateSubscriber : public rclcpp::Node
{
public:
    JointStateSubscriber()
        : Node("state_space_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&JointStateSubscriber::process_the_input, this, _1));
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("effort_controller/commands", 10);

        this->declare_parameter("K", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("kd", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("ke", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kp", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("lqr_transition_angle", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("rviz_test", rclcpp::PARAMETER_BOOL);
        this->declare_parameter("PID_KP", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("PID_KD", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("negative_clamp_limit", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("positive_clamp_limit", rclcpp::PARAMETER_DOUBLE);

        try
        {
            K_ = Eigen::Vector4d(this->get_parameter("K").as_double_array().data());
            kd_ = this->get_parameter("kd").as_double();
            ke_ = this->get_parameter("ke").as_double();
            kp_ = this->get_parameter("kp").as_double();
            lqr_transition_angle_ = this->get_parameter("lqr_transition_angle").as_double();
            rviz_test = this->get_parameter("rviz_test").as_bool();
            PID_k_ = this->get_parameter("PID_KP").as_double();
            PID_d_ = this->get_parameter("PID_KD").as_double();
            negative_clamp_limit_ = this->get_parameter("negative_clamp_limit").as_double();
            positive_clamp_limit_ = this->get_parameter("positive_clamp_limit").as_double();

            RCLCPP_WARN(this->get_logger(), "K: %.4f, %.4f, %.4f, %.4f", K_(0), K_(1), K_(2), K_(3));
            RCLCPP_WARN(this->get_logger(), "kd: %.4f", kd_);
            RCLCPP_WARN(this->get_logger(), "ke: %.4f", ke_);
            RCLCPP_WARN(this->get_logger(), "kp: %.4f", kp_);
            RCLCPP_WARN(this->get_logger(), "PID_KP: %.4f", PID_k_);
            RCLCPP_WARN(this->get_logger(), "PID_KD: %.4f", PID_d_);
            RCLCPP_WARN(this->get_logger(), "lqr_transition_angle_: %.4f", lqr_transition_angle_);
            RCLCPP_WARN(this->get_logger(), "lower clamp: %.4f", negative_clamp_limit_);
            RCLCPP_WARN(this->get_logger(), "higher clamp: %.4f", positive_clamp_limit_);
            if (rviz_test)
            {
                RCLCPP_WARN(this->get_logger(), "RVIZ_TEST: TRUE");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "RVIZ_TEST: FALSE");
            }
        }
        catch (const rclcpp::exceptions::ParameterUninitializedException &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
            throw e;
        }
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState &msg)
    {
        // Find the indices of the "slider" and "swinger" joints
        // auto slider_it = std::find(msg.name.begin(), msg.name.end(), "slider_1");
        auto swinger1_it = std::find(msg.name.begin(), msg.name.end(), "continuous_revolute_1");
        auto swinger2_it = std::find(msg.name.begin(), msg.name.end(), "continuous_revolute_2");

        double slider_position;
        double slider_velocity;

        double swinger1_position;
        double swinger1_velocity;

        double swinger2_position;
        double swinger2_velocity;

        double switching_range = lqr_transition_angle_;
        double f;

        if (swinger1_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger1_it);
            swinger1_position = msg.position[index];
            if (!rviz_test)
            {
                swinger1_velocity = msg.velocity[index];
            }
            else
            {
                swinger1_velocity = swinger1_position - swinger1_previous_position;
                swinger1_previous_position = swinger1_position;
            }

            // RCLCPP_INFO(this->get_logger(), "Swinger -> Position: %.2f, Velocity: %.2f", swinger_position, swinger_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger1 joint not found in the message.");
        }

        if (swinger2_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger2_it);
            swinger2_position = msg.position[index];
            if (!rviz_test)
            {
                swinger2_velocity = msg.velocity[index];
            }
            else
            {
                swinger2_velocity = swinger2_position - swinger2_previous_position;
                swinger2_previous_position = swinger2_position;
            }

            // RCLCPP_INFO(this->get_logger(), "Swinger -> Position: %.2f, Velocity: %.2f", swinger_position, swinger_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger2 joint not found in the message.");
        }

        // GET THE ANGLE POSITION
        if (swinger1_it != msg.name.end() && swinger2_it != msg.name.end())
        {

            // Control parameters
            /*--------------------------------------------------------------------------------*/
            // THIS SHOULD BE ADDED TO A YAML FILE
            double kd = kd_;
            double ke = ke_;
            double kp = kp_;
            /*-------------------------------------------------------------------------------*/

            double mass_1 = 0.48763073080899393963;
            double mass_2 = 0.48763073080899393963;
            double lenght_1 = 0.4;
            double lenght_2 = 0.4;
            double lenghtc1 = lenght_1 / 2;
            double lengthc2 = lenght_2 / 2;
            double g = 9.81;

            double inertia1 = 0.0025009745471827196997;
            double inertia2 = 0.0025009745471827196997;

            double converted_position_1 = solve_for_angle_from_horizontal(swinger1_position);
            double converted_position_2 = solve_for_theta_2(swinger2_position);

            double converted_velocity_1 = -1 * swinger1_velocity;
            double converted_velocity_2 = 1 * swinger2_velocity;

            RCLCPP_WARN(this->get_logger(), "Swinger1 -> %.4f, %.4f", convert_to_degrees(converted_position_1), convert_to_degrees(converted_velocity_1));
            RCLCPP_WARN(this->get_logger(), "Swinger2 -> %.4f, %.4f", convert_to_degrees(converted_position_2), convert_to_degrees(converted_velocity_2));

            double theta_design_1 = mass_1 * lenghtc1 * lenghtc1 + mass_2 * lenght_1 * lenght_1 + inertia1;
            double theta_design_2 = mass_2 * lengthc2 * lengthc2 + inertia2;
            double theta_design_3 = mass_2 * lenght_1 * lengthc2;
            double theta_design_4 = mass_1 * lenghtc1 + mass_2 * lenght_1;
            double theta_design_5 = mass_2 * lengthc2;

            // E = (1/2) * q^T * D(q)*q_dot + P(q)
            //  Matrix D(q)
            Eigen::Matrix2d Dq;
            Dq << theta_design_1 + theta_design_2 + 2 * theta_design_3 * cos(converted_position_2), theta_design_2 + theta_design_3 * cos(converted_position_2),
                theta_design_2 + theta_design_3 * cos(converted_position_2), theta_design_2;

            // Gravity vector G(q)
            Eigen::Vector2d Pq;
            Pq << converted_velocity_1,
                converted_velocity_2;

            // Gravity vector G(q) top
            Eigen::Vector2d Pq_top;
            Pq_top << 0.0,
                0.0;

            double E = 0.5 * Pq.transpose() * Dq * Pq + theta_design_4 * g * sin(converted_position_1) + theta_design_5 * g * sin(converted_position_1 + converted_position_2);
            double E_top = (theta_design_4 + theta_design_5) * g; // TOP POSITION q_1 = PI/2 q_2 = 0.0
            double E_dash = E - E_top;
            double q_dash = converted_position_1 - (PI / 2);

            double force = theta_design_2 * theta_design_3 * sin(converted_position_2) * (converted_velocity_1 + converted_velocity_2) * (converted_velocity_1 + converted_velocity_2) + theta_design_3 * theta_design_3 * cos(converted_position_2) * sin(converted_position_2) * (converted_velocity_1 * converted_velocity_1) - theta_design_2 * theta_design_4 * g * cos(converted_position_1) + theta_design_3 * theta_design_5 * g * cos(converted_position_2) * cos(converted_position_1 + converted_position_2);
            double torque_numerator = -kd * force - (theta_design_1 * theta_design_2 - theta_design_3 * theta_design_3 * cos(converted_position_2) * cos(converted_position_2)) * (converted_velocity_1 + kp * q_dash);
            double torque_denominator = (theta_design_1 * theta_design_2 - theta_design_3 * theta_design_3 * cos(converted_position_2) * cos(converted_position_2)) * ke * E_dash + kd * theta_design_2;

            double torque = torque_numerator / torque_denominator;

            f = torque;
            f = -1 * f;
            f = clamp_force_output(f);
            RCLCPP_INFO(this->get_logger(), BLUE_TEXT "F: %.4f Q: %.4f E_d: %.4f", f, q_dash, E_dash);
        }
        auto message = std_msgs::msg::Float64MultiArray();
        message.data.push_back(f);
        publisher_->publish(message);
    }

    void initialization_proccess(const sensor_msgs::msg::JointState &msg)
    {
        /*READ THE PSITION AND VELOCITY*/
        // Find the indices of the "slider" and "swinger" joints
        // auto slider_it = std::find(msg.name.begin(), msg.name.end(), "slider_1");
        auto swinger1_it = std::find(msg.name.begin(), msg.name.end(), "continuous_revolute_1");
        auto swinger2_it = std::find(msg.name.begin(), msg.name.end(), "continuous_revolute_2");

        double swinger1_position;
        double swinger1_velocity;

        double swinger2_position;
        double swinger2_velocity;

        double switching_range = lqr_transition_angle_;

        if (swinger1_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger1_it);
            swinger1_position = msg.position[index];
            if (!rviz_test)
            {
                swinger1_velocity = msg.velocity[index];
            }
            else
            {
                swinger1_velocity = swinger1_position - swinger1_previous_position;
                swinger1_previous_position = swinger1_position;
            }

            // RCLCPP_INFO(this->get_logger(), "Swinger -> Position: %.2f, Velocity: %.2f", swinger_position, swinger_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger1 joint not found in the message.");
        }

        if (swinger2_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger2_it);
            swinger2_position = msg.position[index];
            if (!rviz_test)
            {
                swinger2_velocity = msg.velocity[index];
            }
            else
            {
                swinger2_velocity = swinger2_position - swinger2_previous_position;
                swinger2_previous_position = swinger2_position;
            }

            // RCLCPP_INFO(this->get_logger(), "Swinger -> Position: %.2f, Velocity: %.2f", swinger_position, swinger_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger2 joint not found in the message.");
        }

        double converted_position_1 = -(PI / 2) + swinger1_position;
        double converted_position_2 = -1 * swinger2_position;

        double converted_velocity_1 = -1 * swinger1_velocity;
        double converted_velocity_2 = -1 * swinger2_velocity;

        /*--------------------------------DECLARING PARAMETERS-------------------------------------*/
        double mass_1 = 0.48763073080899393963;
        double mass_2 = 0.48763073080899393963;
        double lenght_1 = 0.4;
        double lenght_2 = 0.4;
        double lenghtc1 = lenght_1 / 2;
        double lengthc2 = lenght_2 / 2;
        double g = 9.81;

        double inertia1 = 0.0025009745471827196997;
        double inertia2 = 0.0025009745471827196997;

        double theta_design_1 = mass_1 * lenghtc1 * lenghtc1 + mass_2 * lenght_1 * lenght_1 + inertia1;
        double theta_design_2 = mass_2 * lengthc2 * lengthc2 + inertia2;
        double theta_design_3 = mass_2 * lenght_1 * lengthc2;
        double theta_design_4 = mass_1 * lenghtc1 + mass_2 * lenght_1;
        double theta_design_5 = mass_2 * lengthc2;

        /*---------------------CHECK FIRST REQUIREMENT---------------------------------------*/
        double check_1 = 2 * theta_design_1 * (theta_design_4 + theta_design_5) * g;
        if ((kd_ / ke_) > check_1)
        {
            RCLCPP_ERROR(this->get_logger(), GREEN_TEXT "The first requirement is satisfied.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), RED_TEXT "The first requirement is not satisfied.");
            // rclcpp::shutdown(); // Cleanly shuts down the node
            // return;
        }

        /*---------------------CHECK THE SECOND REQUIREMENT-----------------------------------*/
        // E = (1/2) * q^T * D(q)*q_dot + P(q)
        //  Matrix D(q)
        Eigen::Matrix2d Dq;
        Dq << theta_design_1 + theta_design_2 + 2 * theta_design_3 * cos(converted_position_2), theta_design_2 + theta_design_3 * cos(converted_position_2),
            theta_design_2 + theta_design_3 * cos(converted_position_2), theta_design_2;

        // Gravity vector G(q)
        Eigen::Vector2d Pq;
        Pq << converted_velocity_1,
            converted_velocity_2;

        // Gravity vector G(q) top
        Eigen::Vector2d Pq_top;
        Pq_top << 0.0,
            0.0;

        double E = 0.5 * Pq.transpose() * Dq * Pq + theta_design_4 * g * sin(converted_position_1) + theta_design_5 * g * sin(converted_position_1 + converted_position_2);
        double E_top = 0.5 * Pq_top.transpose() * Dq * Pq_top + theta_design_4 * g * sin(converted_position_1) + theta_design_5 * g * sin(((PI) / 2) + 0.0); // TOP POSITION q_1 = PI/2 q_2 = 0.0
        double E_dash = E - E_top;
        double converted_velocity_1_dash = converted_position_1 - ((PI) / 2);

        double c = std::min(2 * theta_design_4 * g, 2 * theta_design_5 * g);
        double V_0 = (ke_ / 2) * E_dash * E_dash + (kd_ / 2) * converted_velocity_2 * converted_velocity_2 + (kp_ / 2) * converted_velocity_1_dash * converted_velocity_1_dash;

        double check_2 = ke_ * ((c * c) / 2);
        if ((V_0) > check_2)
        {
            RCLCPP_ERROR(this->get_logger(), GREEN_TEXT "The first requirement is satisfied.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), RED_TEXT "The second requirement is not satisfied.");
            // rclcpp::shutdown(); // Cleanly shuts down the node
            // return;
        }
    }

    void get_swinger1_to_top_position(const sensor_msgs::msg::JointState &msg)
    { // Find the indices of the "slider" and "swinger" joints
        // auto slider_it = std::find(msg.name.begin(), msg.name.end(), "slider_1");
        auto swinger1_it = std::find(msg.name.begin(), msg.name.end(), "continuous_revolute_1");
        auto swinger2_it = std::find(msg.name.begin(), msg.name.end(), "continuous_revolute_2");

        double slider_position;
        double slider_velocity;

        double swinger1_position;
        double swinger1_velocity;

        double swinger2_position;
        double swinger2_velocity;

        double switching_range = lqr_transition_angle_;
        double f;

        if (swinger1_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger1_it);
            swinger1_position = msg.position[index];
            if (!rviz_test)
            {
                swinger1_velocity = msg.velocity[index];
            }
            else
            {
                swinger1_velocity = swinger1_position - swinger1_previous_position;
                swinger1_previous_position = swinger1_position;
            }

            // RCLCPP_INFO(this->get_logger(), "Swinger -> Position: %.2f, Velocity: %.2f", swinger_position, swinger_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger1 joint not found in the message.");
        }

        if (swinger2_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger2_it);
            swinger2_position = msg.position[index];
            if (!rviz_test)
            {
                swinger2_velocity = msg.velocity[index];
            }
            else
            {
                swinger2_velocity = swinger2_position - swinger2_previous_position;
                swinger2_previous_position = swinger2_position;
            }

            // RCLCPP_INFO(this->get_logger(), "Swinger -> Position: %.2f, Velocity: %.2f", swinger_position, swinger_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger2 joint not found in the message.");
        }

        // GET THE ANGLE POSITION
        if (swinger1_it != msg.name.end() && swinger2_it != msg.name.end())
        {
            double converted_position_1 = solve_for_angle_from_horizontal(swinger1_position);
            double converted_position_2 = solve_for_theta_2(swinger2_position);

            double converted_velocity_1 = -1 * swinger1_velocity;
            double converted_velocity_2 = 1 * swinger2_velocity;

            double computed_error = (PI / 2) - converted_position_1;
            double change_error_rate = computed_error - previous_computed_error;
            previous_computed_error = computed_error;

            RCLCPP_WARN(this->get_logger(), "Swinger1 -> %.4f, %.4f", convert_to_degrees(converted_position_1), convert_to_degrees(converted_velocity_1));
            RCLCPP_WARN(this->get_logger(), "Swinger2 -> %.4f, %.4f", convert_to_degrees(converted_position_2), convert_to_degrees(converted_velocity_2));

            f = PID_k_ * computed_error + PID_d_ * change_error_rate;
            f = -1 * f; // REVERSE THIS SINCE EVERYTHING IS RE ERSED
            f = clamp_force_output(f);
            RCLCPP_WARN(this->get_logger(), "force -> %.4f, error -> %.4f", f, computed_error);
        }
        auto message = std_msgs::msg::Float64MultiArray();
        message.data = {f};
        publisher_->publish(message);
    }

    double clamp_force_output(double output)
    {
        if (output < (negative_clamp_limit_))
        {
            return negative_clamp_limit_;
        }
        else if (output > (positive_clamp_limit_))
        {
            return positive_clamp_limit_;
        }
        else
        {
            return output;
        }
    }

    void process_the_input(const sensor_msgs::msg::JointState &msg)
    {
        if (!initialized_)
        {
            initialization_proccess(msg);
            initialized_ = true;
        }
        else
        {
            topic_callback(msg);
        }
    }

    double solve_for_angle_from_horizontal(double angle_in_rads)
    {
        return ((-1 * angle_in_rads) - PI / 2);
    }

    double solve_for_theta_2(double angle_in_rads)
    {
        return ((1 * angle_in_rads));
    }

    double convert_to_degrees(double rads)
    {
        return (rads * 180 / M_PI);
    }

    double convert_to_rads(double degrees)
    {
        return (degrees * M_PI / 180);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_; // effort controller publisher
    Eigen::Vector4d K_;
    double kd_;
    double ke_;
    double kp_;
    double lqr_transition_angle_;
    bool initialized_ = false; // whether
    bool rviz_test = true;
    double swinger1_previous_position;
    double swinger2_previous_position;
    double PID_k_;
    double PID_d_;
    double previous_computed_error;
    double negative_clamp_limit_;
    double positive_clamp_limit_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateSubscriber>());
    rclcpp::shutdown();
    return 0;
}