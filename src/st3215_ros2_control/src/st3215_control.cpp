#include <cmath>
#include "SCServo_Linux/SCServo.h"
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

using namespace std; 

typedef struct {
    int64_t id;
    int64_t invert;
} motor_data;

class STControl : public rclcpp::Node
{
    public:

    STControl() : Node("st3215_control") {
        speed_        = declare_parameter<int>("speed", 400); // steps per second
        acceleration_ = declare_parameter<int>("acceleration", 200); // 0 - 255
        stop_button_  = declare_parameter<int>("stop_button", 4); // defaults to SixAxis/SteamDeck:L1

        auto usb_port = declare_parameter<string>("usb_port", "/dev/ttyACM1");
        auto baud_rate = declare_parameter<int>("baud_rate", 1000000); //115200 for sms, 1000000 for sts

        std::vector<std::string> default_names = {"Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"};
        joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", default_names);

        joint_ids_ = declare_parameter<vector<int>>("joint_ids", {3, 6, 9, 12, 15, 18});
        invert_motor_ = declare_parameter<vector<bool>>("invert_motor", {false, false, false, false, false, false});

        if (joint_names_.size() != joint_ids_.size() || joint_names_.size() != invert_motor_.size())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Error, joint names,  joint id's and invert motor list must have the same number of elements!");
        }

        for (int i = 0; i < joint_names_.size(); ++i) {

            std::shared_ptr<motor_data> m_data = std::make_shared<motor_data>();
            m_data->id = joint_ids_[i];
            m_data->invert = invert_motor_[i] ? -1 : 1;

            joint_id_map_[joint_names_[i]] = m_data;
        }


        RCLCPP_INFO_STREAM(this->get_logger(), "Port: " << usb_port << " baud_rate: " << baud_rate << ", speed: " << speed_ << " acceleration: " << acceleration_);


        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/current_joint_states", 10);
        joint_cmd_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/desired_joint_states", 10, 
                bind(&STControl::JointCmdCallback, this, placeholders::_1));
        diagnostics_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

        timer_ = this->create_wall_timer(chrono::milliseconds(20), bind(&STControl::TimerCallback, this));

        if(!st3215_.begin(baud_rate, usb_port.c_str())) {
            stop_ = true;
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to initialize motors. Port: " << usb_port << " Baud Rate: " <<baud_rate);
            return;
        }

        for (auto [name, data] : joint_id_map_) {
            st3215_.Mode(static_cast<u8>(data->id), 0); // set to closed loop servo
        }

    }

    ~STControl() {
        DisableMotors();
    }

    private:

    SMS_STS st3215_;

    int speed_, acceleration_, stop_button_;
    bool stop_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Joy::SharedPtr joy_msg_;
    sensor_msgs::msg::JointState::SharedPtr joint_cmd_msg_;

    map<string,  std::shared_ptr<motor_data> > joint_id_map_;
    vector<string> joint_names_;
    vector<int64_t> joint_ids_;
    vector<bool> invert_motor_;


    void DisableMotors(void) {
        for (auto [name, data] : joint_id_map_) {
            stop_ = true;
            st3215_.EnableTorque(static_cast<uint8_t>(data->id), 0);
        }
        st3215_.end();
        RCLCPP_INFO(this->get_logger(), "Motors Disabled");
    };

    void JointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg) { joint_cmd_msg_ = msg; }

    void TimerCallback() {

        if (joint_cmd_msg_) {
            for (int i = 0; i < joint_cmd_msg_->name.size(); ++i)
            {
                auto maybe_motor_data = joint_id_map_.find(joint_cmd_msg_->name[i]);
                if (maybe_motor_data != joint_id_map_.end())
                {
                    auto motor_data = maybe_motor_data->second;

                    auto u8_id = static_cast<uint8_t>(motor_data->id);

                    auto target_pos = (i < joint_cmd_msg_->position.size()) ? joint_cmd_msg_->position[i] : -1;

                    target_pos = (M_PI + motor_data->invert * target_pos) * 4096.0 / (2 * M_PI);

                    int int_pos = static_cast<int>(round(target_pos));

                    if (target_pos != -1) {
                        st3215_.WritePosEx(u8_id, int_pos, speed_, acceleration_);
                    }
                }
                else
                {
                    RCLCPP_WARN_STREAM_ONCE(this->get_logger(), "Warning: " << joint_cmd_msg_->name[i].c_str() << " in desired joint states but not in joint_names list");
                }
            }
        }

        auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
        //auto diagnostic_msg = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();

        for (auto [name, motor_data] : joint_id_map_) {

            auto u8_id = static_cast<uint8_t>(motor_data->id);
            
            if (st3215_.FeedBack(u8_id) == -1)
            {
                RCLCPP_WARN_STREAM_ONCE(this->get_logger(), "No feedback from motor: " << (int)u8_id );
                continue; // skip this motor
            }

            // read feedback data
            double position = 0.0, speed = 0.0, pwm = 0.0, temperature = 0.0, voltage = 0.0, current = 0.0;
            int move = 0;
            
            position = M_PI + motor_data->invert * st3215_.ReadPos(u8_id)*2*M_PI/4096.0; // 1 step=2*PI/4096.0 rad, 
            speed = -1 * st3215_.ReadSpeed(u8_id)*2*M_PI/4096.0;  // 1 steps/s=2*PI/4096.0 rads/sec
            pwm = -1 * st3215_.ReadLoad(u8_id)/10.0; // 0-1000 : 0-100%
            move = st3215_.ReadMove(u8_id);
            temperature = st3215_.ReadTemper(u8_id); // 1 : 1 degree Celcius
            voltage = st3215_.ReadVoltage(u8_id)/10; // 1 : 0.1 V
            current = st3215_.ReadCurrent(u8_id)*12/1000; // 1 : 12/1000 A

            joint_state_msg->name.push_back(name);
            joint_state_msg->position.push_back(position);
            joint_state_msg->velocity.push_back(speed);
            joint_state_msg->effort.push_back(current);
        }
        joint_state_msg->header.stamp = now();
        joint_state_publisher_->publish(*joint_state_msg);
    }

};


int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto node = make_shared<STControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}