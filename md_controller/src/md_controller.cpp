#include "md_controller/com.hpp"
#include "md_controller/kinematics.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

Communication Com;  
MotorVar Motor;

geometry_msgs::msg::TransformStamped odom_tf;
sensor_msgs::msg::JointState joint_states;

BYTE SendCmdRpm = OFF;
int left_rpm_ = 0;
int right_rpm_ = 0;

int left_sign_ = 1;
bool right_enabled_ = true;
int right_driver_id_ = 1;
int right_driver_mdt_ = 183;
int right_gear_ratio_ = 25;
int right_sign_ = 1;
bool right_use_separate_port_ = true;
std::string right_port_ = "/dev/ttyMotorRight";
int right_baudrate_ = 57600;
int cmd_timeout_ms_ = 300;
int max_driver_rpm_ = 3000;
bool right_serial_ready_ = false;
bool publish_odom_tf_ = true;
std::string odom_frame_id_ = "odom";
std::string base_frame_id_ = "base_link";

double odom_x_ = 0.0;
double odom_y_ = 0.0;
double odom_yaw_ = 0.0;

serial::Serial right_ser_;

std::chrono::steady_clock::time_point last_cmd_time_ = std::chrono::steady_clock::now();

inline int ClampRpm(int value, int max_abs) {
    if (value > max_abs) return max_abs;
    if (value < -max_abs) return -max_abs;
    return value;
}

struct SerialPacketParser {
    BYTE buf[MAX_PACKET_SIZE] = {0};
    BYTE step = 0;
    BYTE packet_num = 0;
    BYTE max_data_num = 0;
    BYTE data_num = 0;
    BYTE chk_sum = 0;
    BYTE chk_com_error = 0;
    BYTE sync_count = 0;
};

SerialPacketParser right_parser_;
bool right_feedback_valid_ = false;
int32_t right_feedback_ticks_ = 0;

bool encoder_odom_initialized_ = false;
int32_t last_left_ticks_ = 0;
int32_t last_right_ticks_ = 0;
double left_tick_to_rad_ = 0.0;
double right_tick_to_rad_ = 0.0;

void ResetParser(SerialPacketParser& parser) {
    parser.step = 0;
    parser.packet_num = 0;
    parser.max_data_num = 0;
    parser.data_num = 0;
    parser.chk_sum = 0;
    parser.sync_count = 0;
}

bool FeedParserByte(SerialPacketParser& parser, BYTE value) {
    if (parser.packet_num >= MAX_PACKET_SIZE) {
        ResetParser(parser);
        return false;
    }

    switch (parser.step) {
        case 0:
            if ((value == 184) || (value == 183)) {
                parser.chk_sum += value;
                parser.buf[parser.packet_num++] = value;
                parser.chk_com_error = 0;
                parser.sync_count++;
                if (parser.sync_count == 2) {
                    parser.sync_count = 0;
                    parser.step = 1;
                }
            } else {
                parser.chk_com_error++;
                ResetParser(parser);
            }
            break;
        case 1:
            if (value >= 1 && value <= 254) {
                parser.chk_sum += value;
                parser.buf[parser.packet_num++] = value;
                parser.step = 2;
                parser.chk_com_error = 0;
            } else {
                parser.chk_com_error++;
                ResetParser(parser);
            }
            break;
        case 2:
            parser.chk_sum += value;
            parser.buf[parser.packet_num++] = value;
            parser.step = 3;
            break;
        case 3:
            parser.max_data_num = value;
            parser.data_num = 0;
            parser.chk_sum += value;
            parser.buf[parser.packet_num++] = value;
            parser.step = 4;
            break;
        case 4:
            parser.buf[parser.packet_num++] = value;
            parser.chk_sum += value;
            if (++parser.data_num >= MAX_DATA_SIZE) {
                ResetParser(parser);
                return false;
            }
            if (parser.data_num >= parser.max_data_num) {
                parser.step = 5;
            }
            break;
        case 5: {
            parser.chk_sum += value;
            parser.buf[parser.packet_num++] = value;
            const bool packet_ok = (parser.chk_sum == 0);
            ResetParser(parser);
            return packet_ok;
        }
        default:
            ResetParser(parser);
            break;
    }

    return false;
}

void ProcessRightPacket(const BYTE* packet) {
    const BYTE packet_id = packet[2];
    const BYTE packet_pid = packet[3];
    if (packet_pid != PID_MAIN_DATA) {
        return;
    }
    if (packet_id != static_cast<BYTE>(right_driver_id_)) {
        return;
    }

    right_feedback_ticks_ = static_cast<int32_t>(
        Byte2LInt(packet[15], packet[16], packet[17], packet[18]));
    right_feedback_valid_ = true;
}

void ReceiveRightDataFromController() {
    if (!right_enabled_ || !right_use_separate_port_ || !right_serial_ready_) {
        return;
    }

    const size_t available = right_ser_.available();
    if (available == 0) {
        return;
    }

    std::vector<BYTE> rx_buffer(available);
    const size_t read_len = right_ser_.read(rx_buffer, available);

    for (size_t i = 0; i < read_len; ++i) {
        if (FeedParserByte(right_parser_, rx_buffer[i])) {
            ProcessRightPacket(right_parser_.buf);
        }
    }
}

bool InitRightSerial() {
    try {
        right_ser_.setPort(right_port_);
        right_ser_.setBaudrate(right_baudrate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1667);
        right_ser_.setTimeout(to);
        right_ser_.open();
    } catch (serial::IOException& e) {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("rclcpp"),
            "Unable to open right motor port: " << right_port_);
        return false;
    }

    if (!right_ser_.isOpen()) {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("rclcpp"),
            "Right motor port is not open: " << right_port_);
        return false;
    }

    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("rclcpp"),
        "Right serial port initialized: " << right_port_);
    return true;
}

int PutMdDataToSerial(serial::Serial& ser, BYTE byPID, BYTE byMID, int id_num, int nArray[]) {
    BYTE bySndBuf[MAX_PACKET_SIZE] = {0};
    BYTE byPidDataSize = 0;
    BYTE byDataSize = 0;
    BYTE byTempDataSum = 0;

    bySndBuf[0] = byMID;
    bySndBuf[1] = 184;
    bySndBuf[2] = id_num;
    bySndBuf[3] = byPID;

    switch (byPID) {
        case PID_REQ_PID_DATA:
            byDataSize = 1;
            byPidDataSize = 7;
            bySndBuf[4] = byDataSize;
            bySndBuf[5] = static_cast<BYTE>(nArray[0]);
            break;
        case PID_POSI_RESET:
            byDataSize = 1;
            byPidDataSize = 7;
            bySndBuf[4] = byDataSize;
            bySndBuf[5] = static_cast<BYTE>(nArray[0]);
            break;
        case PID_COMMAND:
            byDataSize = 1;
            byPidDataSize = 7;
            bySndBuf[4] = byDataSize;
            bySndBuf[5] = static_cast<BYTE>(nArray[0]);
            break;
        case PID_VEL_CMD:
            byDataSize = 2;
            byPidDataSize = 8;
            bySndBuf[4] = byDataSize;
            bySndBuf[5] = static_cast<BYTE>(nArray[0]);
            bySndBuf[6] = static_cast<BYTE>(nArray[1]);
            break;
        case PID_PNT_VEL_CMD:
            byDataSize = 7;
            byPidDataSize = 13;
            bySndBuf[4] = byDataSize;
            bySndBuf[5] = static_cast<BYTE>(nArray[0]);
            bySndBuf[6] = static_cast<BYTE>(nArray[1]);
            bySndBuf[7] = static_cast<BYTE>(nArray[2]);
            bySndBuf[8] = static_cast<BYTE>(nArray[3]);
            bySndBuf[9] = static_cast<BYTE>(nArray[4]);
            bySndBuf[10] = static_cast<BYTE>(nArray[5]);
            bySndBuf[11] = static_cast<BYTE>(nArray[6]);
            break;
        default:
            return FAIL;
    }

    for (BYTE i = 0; i < (byPidDataSize - 1); i++) {
        byTempDataSum += bySndBuf[i];
    }
    bySndBuf[byPidDataSize - 1] = static_cast<BYTE>(~(byTempDataSum) + 1);

    ser.write(bySndBuf, byPidDataSize);
    return SUCCESS;
}

int SendRightMdData(BYTE byPID, int nArray[]) {
    if (!right_enabled_) return SUCCESS;

    if (right_use_separate_port_) {
        if (!right_serial_ready_) return FAIL;
        return PutMdDataToSerial(right_ser_, byPID, right_driver_mdt_, right_driver_id_, nArray);
    }

    return PutMdData(byPID, right_driver_mdt_, right_driver_id_, nArray);
}

void SendSideDualChannelRpm(int mdt_id, int driver_id, int wheel_rpm, int gear_ratio, int sign) {
    int scaled_rpm = ClampRpm(wheel_rpm * gear_ratio * sign, max_driver_rpm_);
    IByte rpm_data = Short2Byte(static_cast<short>(scaled_rpm));
    int nArray[20] = {0};

    // One side driver controls two motors (front/rear) with same target RPM.
    nArray[0] = 1;
    nArray[1] = rpm_data.byLow;
    nArray[2] = rpm_data.byHigh;
    nArray[3] = 1;
    nArray[4] = rpm_data.byLow;
    nArray[5] = rpm_data.byHigh;
    nArray[6] = 0;

    PutMdData(PID_PNT_VEL_CMD, mdt_id, driver_id, nArray);
}

void SendRightSideDualChannelRpm(int wheel_rpm) {
    int scaled_rpm = ClampRpm(wheel_rpm * right_gear_ratio_ * right_sign_, max_driver_rpm_);
    IByte rpm_data = Short2Byte(static_cast<short>(scaled_rpm));
    int nArray[20] = {0};

    nArray[0] = 1;
    nArray[1] = rpm_data.byLow;
    nArray[2] = rpm_data.byHigh;
    nArray[3] = 1;
    nArray[4] = rpm_data.byLow;
    nArray[5] = rpm_data.byHigh;
    nArray[6] = 0;

    SendRightMdData(PID_PNT_VEL_CMD, nArray);
}

void CmdVelCallBack(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // cmd_vel에서 linear.x와 angular.z 추출
    float linear_x = msg->linear.x;   // 직진 속도 (m/s)
    float angular_z = msg->angular.z; // 회전 속도 (rad/s)
    cmdVelToRpm(linear_x, angular_z, left_rpm_, right_rpm_); //convert /cmd_vel to left_rpm_ , right_rpm_
    last_cmd_time_ = std::chrono::steady_clock::now();
    SendCmdRpm = ON;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("md_controller_node");

    // create TF / odom publisher
    tf2_ros::TransformBroadcaster tf_broadcaster_(node);
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 20);
    auto last_odom_time = node->now();

    //subscriber
    auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, CmdVelCallBack);

    //Motor driver settup-------------------------------------------------------------------------------
    node->declare_parameter("MDUI", 184);
    node->declare_parameter("MDT", 183);
    node->declare_parameter("Port", "/dev/ttyMotorLeft");
    node->declare_parameter("Baudrate", 57600);
    node->declare_parameter("ID", 1); //fix
    node->declare_parameter("GearRatio", 25);
    node->declare_parameter("poles", 8);
    node->declare_parameter("left_sign", 1);
    node->declare_parameter("right_enabled", true);
    node->declare_parameter("RightID", 1);
    node->declare_parameter("RightMDT", 183);
    node->declare_parameter("RightGearRatio", 25);
    node->declare_parameter("right_sign", 1);
    node->declare_parameter("RightUseSeparatePort", true);
    node->declare_parameter("RightPort", "/dev/ttyMotorRight");
    node->declare_parameter("RightBaudrate", 57600);
    node->declare_parameter("cmd_timeout_ms", 300);
    node->declare_parameter("max_driver_rpm", 3000);
    node->declare_parameter("wheel_radius", 0.103);
    node->declare_parameter("wheel_base", 0.4);
    node->declare_parameter("publish_odom_tf", true);
    node->declare_parameter("odom_frame_id", "odom");
    node->declare_parameter("base_frame_id", "base_link");

    node->get_parameter("MDUI", Com.nIDMDUI);
    node->get_parameter("MDT", Com.nIDMDT);
    node->get_parameter("Port", Com.nPort);
    node->get_parameter("Baudrate", Com.nBaudrate);
    node->get_parameter("ID", Motor.ID);
    node->get_parameter("GearRatio", Motor.GearRatio);
    node->get_parameter("poles", Motor.poles);
    node->get_parameter("left_sign", left_sign_);
    node->get_parameter("right_enabled", right_enabled_);
    node->get_parameter("RightID", right_driver_id_);
    node->get_parameter("RightMDT", right_driver_mdt_);
    node->get_parameter("RightGearRatio", right_gear_ratio_);
    node->get_parameter("right_sign", right_sign_);
    node->get_parameter("RightUseSeparatePort", right_use_separate_port_);
    node->get_parameter("RightPort", right_port_);
    node->get_parameter("RightBaudrate", right_baudrate_);
    node->get_parameter("cmd_timeout_ms", cmd_timeout_ms_);
    node->get_parameter("max_driver_rpm", max_driver_rpm_);
    node->get_parameter("publish_odom_tf", publish_odom_tf_);
    node->get_parameter("odom_frame_id", odom_frame_id_);
    node->get_parameter("base_frame_id", base_frame_id_);

    //Robot Param load for kinematics.cpp
    float wheel_radius_param, wheel_base_param;
    node->get_parameter("wheel_radius", wheel_radius_param);
    node->get_parameter("wheel_base", wheel_base_param);
    setRobotParams(wheel_radius_param, wheel_base_param);

    Motor.PPR       = Motor.poles*3*Motor.GearRatio;           //poles * 3(HALL U,V,W) * gear ratio
    Motor.Tick2RAD  = (360.0/Motor.PPR)*PI / 180;
    left_tick_to_rad_ = Motor.Tick2RAD;
    const double right_ppr = std::max(1, Motor.poles * 3 * right_gear_ratio_);
    right_tick_to_rad_ = (360.0 / right_ppr) * PI / 180.0;

    int nArray[20];
    static BYTE fgInitsetting, byCntInitStep, byCntComStep, byCnt2500us, byCntStartDelay, byCntCase[5];
    
    byCntInitStep     = 1;
    Motor.InitMotor   = ON;
    fgInitsetting     = OFF;
    Motor.InitError   = 0;
    Motor.last_rad    = 0;
    Motor.last_tick   = 0;

    InitSerial();   //communication initialization in com.cpp
    if (right_enabled_ && right_use_separate_port_) {
        right_serial_ready_ = InitRightSerial();
        if (!right_serial_ready_) {
            RCLCPP_WARN(
                rclcpp::get_logger("rclcpp"),
                "Right driver disabled because right serial initialization failed.");
            right_enabled_ = false;
        }
    }
    while (rclcpp::ok()) {
        
        ReceiveDataFromController(Motor.InitMotor);
        ReceiveRightDataFromController();
        if(++byCnt2500us == 50)
        {
            byCnt2500us = 0;
            
            if(fgInitsetting == ON)
            {
                switch(++byCntComStep)
                {
                case 1:{ // publish /odom and odom->base_link TF
                    const auto now = node->now();
                    double dt = (now - last_odom_time).seconds();
                    if (dt < 0.0) {
                        dt = 0.0;
                    } else if (dt > 0.5) {
                        // Ignore abnormally large dt to avoid odom jumps after pauses.
                        dt = 0.0;
                    }
                    last_odom_time = now;

                    // Left feedback is parsed by com.cpp and stored in Com.
                    const int32_t left_feedback_ticks = static_cast<int32_t>(Com.position);
                    const bool left_feedback_valid = (Motor.InitMotor == OFF);
                    const bool right_feedback_available =
                        (!right_enabled_) || (right_use_separate_port_ && right_feedback_valid_);

                    double linear_x = 0.0;
                    double angular_z = 0.0;

                    if (left_feedback_valid && right_feedback_available) {
                        const int32_t right_ticks =
                            right_enabled_ ? right_feedback_ticks_ : left_feedback_ticks;

                        if (!encoder_odom_initialized_) {
                            last_left_ticks_ = left_feedback_ticks;
                            last_right_ticks_ = right_ticks;
                            encoder_odom_initialized_ = true;
                            break;
                        }

                        const int32_t delta_left_ticks = left_feedback_ticks - last_left_ticks_;
                        const int32_t delta_right_ticks = right_ticks - last_right_ticks_;
                        last_left_ticks_ = left_feedback_ticks;
                        last_right_ticks_ = right_ticks;

                        // Ignore abnormal jumps (e.g. reconnect/reset) and re-sync.
                        if (std::abs(delta_left_ticks) > 200000 || std::abs(delta_right_ticks) > 200000) {
                            break;
                        }

                        const double left_distance = static_cast<double>(delta_left_ticks) * left_tick_to_rad_ * wheel_radius;
                        const double right_distance = static_cast<double>(delta_right_ticks) * right_tick_to_rad_ * wheel_radius;

                        const double delta_s = 0.5 * (left_distance + right_distance);
                        const double delta_yaw = (wheel_base > 1e-6)
                            ? ((right_distance - left_distance) / wheel_base)
                            : 0.0;

                        odom_x_ += delta_s * std::cos(odom_yaw_ + 0.5 * delta_yaw);
                        odom_y_ += delta_s * std::sin(odom_yaw_ + 0.5 * delta_yaw);
                        odom_yaw_ += delta_yaw;

                        if (dt > 1e-6) {
                            linear_x = delta_s / dt;
                            angular_z = delta_yaw / dt;
                        }
                    } else {
                        // Fallback: integrate cmd-based wheel speeds when feedback is not ready.
                        const double left_wheel_mps =
                            (static_cast<double>(left_rpm_) * 2.0 * PI * wheel_radius) / 60.0;
                        const double right_wheel_mps = right_enabled_
                            ? (static_cast<double>(right_rpm_) * 2.0 * PI * wheel_radius) / 60.0
                            : left_wheel_mps;

                        linear_x = 0.5 * (left_wheel_mps + right_wheel_mps);
                        angular_z = (wheel_base > 1e-6)
                            ? ((right_wheel_mps - left_wheel_mps) / wheel_base)
                            : 0.0;

                        odom_x_ += linear_x * std::cos(odom_yaw_) * dt;
                        odom_y_ += linear_x * std::sin(odom_yaw_) * dt;
                        odom_yaw_ += angular_z * dt;
                    }

                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, odom_yaw_);

                    nav_msgs::msg::Odometry odom_msg;
                    odom_msg.header.stamp = now;
                    odom_msg.header.frame_id = odom_frame_id_;
                    odom_msg.child_frame_id = base_frame_id_;
                    odom_msg.pose.pose.position.x = odom_x_;
                    odom_msg.pose.pose.position.y = odom_y_;
                    odom_msg.pose.pose.position.z = 0.0;
                    odom_msg.pose.pose.orientation.x = q.x();
                    odom_msg.pose.pose.orientation.y = q.y();
                    odom_msg.pose.pose.orientation.z = q.z();
                    odom_msg.pose.pose.orientation.w = q.w();
                    odom_msg.twist.twist.linear.x = linear_x;
                    odom_msg.twist.twist.angular.z = angular_z;
                    odom_pub->publish(odom_msg);

                    if (publish_odom_tf_) {
                        geometry_msgs::msg::TransformStamped odom_tf_msg;
                        odom_tf_msg.header.stamp = now;
                        odom_tf_msg.header.frame_id = odom_frame_id_;
                        odom_tf_msg.child_frame_id = base_frame_id_;
                        odom_tf_msg.transform.translation.x = odom_x_;
                        odom_tf_msg.transform.translation.y = odom_y_;
                        odom_tf_msg.transform.translation.z = 0.0;
                        odom_tf_msg.transform.rotation.x = q.x();
                        odom_tf_msg.transform.rotation.y = q.y();
                        odom_tf_msg.transform.rotation.z = q.z();
                        odom_tf_msg.transform.rotation.w = q.w();
                        tf_broadcaster_.sendTransform(odom_tf_msg);
                    }
                    break;
                }
                case 2: //Control motor & request motor info
                    if(++byCntCase[byCntComStep] == TIME_100MS)
                    {
                        byCntCase[byCntComStep] = 0;

                        const auto now = std::chrono::steady_clock::now();
                        const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - last_cmd_time_).count();

                        // Safety: stop motors if cmd_vel is stale.
                        if (dt_ms > cmd_timeout_ms_) {
                            left_rpm_ = 0;
                            right_rpm_ = 0;
                            SendCmdRpm = ON;
                        }

                        if(SendCmdRpm)
                        {
                            // Left side driver (A)
                            SendSideDualChannelRpm(Com.nIDMDT, Motor.ID, left_rpm_, Motor.GearRatio, left_sign_);

                            // Right side driver (B)
                            if (right_enabled_) {
                                SendRightSideDualChannelRpm(right_rpm_);
                            }

                            //n대의 모터드라이버에게 동시에 main data를 요청할 경우 data를 받을 때 데이터가 섞임을 방지.
                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);  // Main data request
                            if (right_enabled_) {
                                SendRightMdData(PID_REQ_PID_DATA, nArray);                // Right main data request
                            }

                            SendCmdRpm = OFF;
                        }
                        else
                        {
                            //n대의 모터드라이버에게 동시에 main data를 요청할 경우 data를 받을 때 데이터가 섞임을 방지.
                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);  // Main data request
                            if (right_enabled_) {
                                SendRightMdData(PID_REQ_PID_DATA, nArray);                // Right main data request
                            }
                            //------------------------------------------------------------------
                        }
                        
                    }
                    byCntComStep=0;
                    break;  
                }
            }
            else
            {
                if(byCntStartDelay <= 200) byCntStartDelay++;
                else
                {
                    switch (byCntInitStep)
                    {
                    case 1: //Motor connect check
                        nArray[0] = PID_MAIN_DATA;
                        PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);
                        if (right_enabled_) {
                            SendRightMdData(PID_REQ_PID_DATA, nArray);
                        }

                        if(Motor.InitMotor == ON)
                            Motor.InitError++;
                        else
                            byCntInitStep++;

                        if(Motor.InitError > 10){
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"ID %d MOTOR INIT ERROR!!", Motor.ID); 
                            return 0;
                        }
                    
                        break;
                    case 2:
                        byCntInitStep++;
                        break;

                    case 3: //Motor torque ON
                        nArray[0] = 0;
                        nArray[1] = 0;
                        PutMdData(PID_VEL_CMD, Com.nIDMDT, Motor.ID, nArray);
                        if (right_enabled_) {
                            SendRightMdData(PID_VEL_CMD, nArray);
                        }

                        byCntInitStep++;
                        break;

                    case 4: //Motor POS reset
                        nArray[0] = 0;
                        PutMdData(PID_POSI_RESET, Com.nIDMDT, Motor.ID, nArray);
                        if (right_enabled_) {
                            SendRightMdData(PID_POSI_RESET, nArray);
                        }
                        encoder_odom_initialized_ = false;
                        byCntInitStep++;
                        break;

                    case 5:
                        printf("========================================================\n\n");
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MOTOR INIT END\n");
                        fgInitsetting = ON;

                        break;

                    }
                }
            }
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
