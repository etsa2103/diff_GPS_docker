#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rtcm_msgs/msg/message.hpp>

// Serial
#include <libserial/SerialPort.h>

#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

/* ------------------ Utilities ------------------ */
static std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) out.push_back(item);
    return out;
}

static bool parse_double(const std::string &s, double &out)
{
    try { if (s.empty()) return false; out = std::stod(s); return true; }
    catch (...) { return false; }
}

/* ------------------ Node ------------------ */
class DiffGpsNode : public rclcpp::Node
{
public:
    DiffGpsNode() : Node("diff_gps_node")
    {
        port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
        baud_ = declare_parameter<int>("baudrate", 460800);

        fix_pub_        = create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
        heading_pub_    = create_publisher<geometry_msgs::msg::QuaternionStamped>("gps/heading", 10);
        cov_pub_        = create_publisher<geometry_msgs::msg::Vector3>("gps/stddev", 10);
        heading_acc_pub_= create_publisher<std_msgs::msg::Float64>("gps/heading_accuracy_deg", 10);

        rtcm_sub_ = create_subscription<rtcm_msgs::msg::Message>(
            "/rtcm", 10,
            std::bind(&DiffGpsNode::rtcm_callback, this, std::placeholders::_1));

        open_serial();
        timer_ = create_wall_timer(50ms, std::bind(&DiffGpsNode::read_serial, this));
    }

private:
    /* ------------------ Serial Setup ------------------ */
    void open_serial()
    {
        serial_.Open(port_);

        // Map common baudrates
        if (baud_ == 115200) serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        else if (baud_ == 230400) serial_.SetBaudRate(LibSerial::BaudRate::BAUD_230400);
        else serial_.SetBaudRate(LibSerial::BaudRate::BAUD_460800);

        serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    }

    /* ------------------ Serial Read ------------------ */
    void read_serial()
    {
        if (!serial_.IsOpen()) return;

        std::string line;
        try { serial_.ReadLine(line, '\n', 1000); }
        catch (...) { return; }

        if (line.empty() || line[0] != '$') return;

        if (line.rfind("$GNGGA",0)==0 || line.rfind("$GPGGA",0)==0) parse_gga(line);
        else if (line.rfind("$GNGST",0)==0 || line.rfind("$GPGST",0)==0) parse_gst(line);
        else if (line.rfind("$PQTMTAR",0)==0) parse_pqtmtar(line);
    }

    /* ------------------ GGA: Position ------------------ */
    void parse_gga(const std::string &nmea)
    {
        auto f = split(nmea, ',');
        if (f.size() < 10 || f[2].empty()) return;

        double lat = nmea_to_deg(f[2], f[3]);
        double lon = nmea_to_deg(f[4], f[5]);
        double alt = 0.0;
        int fix_quality = 0;

        parse_double(f[9], alt);
        try { if(!f[6].empty()) fix_quality = std::stoi(f[6]); }
        catch (...) { fix_quality = 0; }

        sensor_msgs::msg::NavSatFix msg;
        msg.header.stamp = now();
        msg.header.frame_id = "gps";
        msg.latitude = lat;
        msg.longitude = lon;
        msg.altitude = alt;

        msg.status.status = (fix_quality > 0) ?
            sensor_msgs::msg::NavSatStatus::STATUS_FIX :
            sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

        if (last_stddev_[0] >= 0.0) {
            msg.position_covariance[0] = last_stddev_[0]*last_stddev_[0];
            msg.position_covariance[4] = last_stddev_[1]*last_stddev_[1];
            msg.position_covariance[8] = last_stddev_[2]*last_stddev_[2];
            msg.position_covariance_type =
                sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        } else {
            msg.position_covariance_type =
                sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        }

        fix_pub_->publish(msg);

        // -------- Clean Status Print --------
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "POS %.7f %.7f alt=%.2f | StdDev[m]: %.3f %.3f %.3f | "
            "Heading=%.2f deg | HeadAcc=%.3f deg | Baseline=%.3f m",
            lat, lon, alt,
            last_stddev_[0], last_stddev_[1], last_stddev_[2],
            last_heading_, last_heading_acc_deg_, last_baseline_m_);
    }

    /* ------------------ GST: Accuracy ------------------ */
    void parse_gst(const std::string &nmea)
    {
        auto f = split(nmea, ',');
        if (f.size() < 9) return;

        if (!parse_double(f[6], last_stddev_[0])) return;
        if (!parse_double(f[7], last_stddev_[1])) return;
        if (!parse_double(f[8], last_stddev_[2])) return;

        geometry_msgs::msg::Vector3 msg;
        msg.x = last_stddev_[0];
        msg.y = last_stddev_[1];
        msg.z = last_stddev_[2];
        cov_pub_->publish(msg);
    }

    /* ------------------ PQTMTAR: Dual Antenna Heading ------------------ */
    void parse_pqtmtar(const std::string &nmea)
    {
        auto f = split(nmea, ',');
        if (f.size() < 11) return;

        parse_double(f[5],  last_baseline_m_);
        parse_double(f[8],  last_heading_);
        parse_double(f[11], last_heading_acc_deg_);

        // Publish heading accuracy
        std_msgs::msg::Float64 acc;
        acc.data = last_heading_acc_deg_;
        heading_acc_pub_->publish(acc);

        // Convert GNSS heading (0°=North CW) → ROS ENU yaw
        double yaw = (90.0 - last_heading_) * M_PI / 180.0;

        geometry_msgs::msg::QuaternionStamped q;
        q.header.stamp = now();
        q.header.frame_id = "gps";
        q.quaternion.z = std::sin(yaw*0.5);
        q.quaternion.w = std::cos(yaw*0.5);
        heading_pub_->publish(q);
    }

    /* ------------------ NMEA Conversion ------------------ */
    static double nmea_to_deg(const std::string &val, const std::string &dir)
    {
        if (val.size() < 6) return 0.0;
        int deg_len = (dir=="N"||dir=="S") ? 2 : 3;
        double deg = std::stod(val.substr(0,deg_len));
        double min = std::stod(val.substr(deg_len));
        double out = deg + min/60.0;
        if (dir=="S"||dir=="W") out *= -1.0;
        return out;
    }

    /* ------------------ RTCM Forwarding ------------------ */
    void rtcm_callback(const rtcm_msgs::msg::Message::SharedPtr msg)
    {
        if (!serial_.IsOpen() || msg->message.empty()) return;
        try {
            LibSerial::DataBuffer buf(msg->message.begin(), msg->message.end());
            serial_.Write(buf);
        } catch (...) {}
    }

    /* ------------------ Members ------------------ */
    std::string port_;
    int baud_;
    LibSerial::SerialPort serial_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr cov_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_acc_pub_;
    rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;

    double last_stddev_[3] = {-1.0,-1.0,-1.0};
    double last_heading_ = -1.0;
    double last_heading_acc_deg_ = -1.0;
    double last_baseline_m_ = -1.0;
};

/* ------------------ Main ------------------ */
int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<DiffGpsNode>());
    rclcpp::shutdown();
    return 0;
}
