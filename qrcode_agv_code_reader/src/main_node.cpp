#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/math/constants/constants.hpp>

#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "angles/angles.h"
#include <tf2/LinearMath/Quaternion.h>
#include "qrcode_reader_interfaces/msg/qr_detect_result.hpp"

using boost::asio::ip::tcp;
using boost::asio::deadline_timer;
using namespace std::chrono_literals;

class QRCodeAGVCodeReaderNode : public rclcpp::Node
{
    public:
        QRCodeAGVCodeReaderNode(): Node("agv_code_reader_node")
        {
            this->declare_parameter<std::string>("ip", "127.0.0.1");
            this->declare_parameter<int>("port", 3000);

            ip_addr_ = this->get_parameter("ip").get_parameter_value().get<std::string>();
            port_num_ = this->get_parameter("port").get_parameter_value().get<long int>();
            RCLCPP_INFO(this->get_logger(), "Reader IP: [%s] and port %ld", ip_addr_.c_str(), port_num_);

            rclcpp::sleep_for(std::chrono::milliseconds(100));

            pub_qr_detect_result_ = this->create_publisher<qrcode_reader_interfaces::msg::QRDetectResult>("qrcode_scan_result", 10);
            thread_recv_data_ = std::make_shared<std::thread>(std::bind(&QRCodeAGVCodeReaderNode::thread_recv_data_func, this));

            RCLCPP_INFO(this->get_logger(), "Initialized...");
        }
        ~QRCodeAGVCodeReaderNode() {}

    private:
        void thread_recv_data_func(void)
        {
            boost::asio::io_service io_service;
            tcp::socket socket(io_service);
            socket.connect(tcp::endpoint(boost::asio::ip::address::from_string(ip_addr_), port_num_));

            RCLCPP_INFO(this->get_logger(), "Start thread...");

            while(rclcpp::ok())
            {
                boost::system::error_code error;
                boost::asio::streambuf recv_buf;

                boost::asio::read_until(socket, recv_buf, ")", error);

                std::istream is(&recv_buf);
                std::string recv_data;
                std::getline(is, recv_data);

                recv_data.erase(0, 1);
                recv_data.erase(recv_data.size() - 1);

                std::vector<std::string> split_str;
                boost::split(split_str, recv_data, boost::is_any_of(";"));

                qrcode_reader_interfaces::msg::QRDetectResult result;
                result.header.stamp = this->now();

                if(split_str.size() == 4)
                {
                    double x_offset = std::stoi(split_str[0]) / 10000.0;
                    double y_offset = std::stoi(split_str[1]) / 10000.0;
                    double theta = angles::normalize_angle(angles::from_degrees((std::stoi(split_str[2]) + 900)/ 10.0));

                    result.code_id = split_str[3];
                    result.pose.position.x = y_offset;
                    result.pose.position.y = x_offset;

                    tf2::Quaternion q;
                    q.setRPY(0, 0, theta);

                    result.pose.orientation.x = q[0];
                    result.pose.orientation.y = q[1];
                    result.pose.orientation.z = q[2];
                    result.pose.orientation.w = q[3];

                    result.is_detected = true;
                }
                else
                {
                    result.is_detected = false;
                }

                pub_qr_detect_result_->publish(result);
            }
        }

    private:
        std::string ip_addr_;
        long int port_num_;

        rclcpp::Publisher<qrcode_reader_interfaces::msg::QRDetectResult>::SharedPtr pub_qr_detect_result_;
        std::shared_ptr<std::thread> thread_recv_data_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QRCodeAGVCodeReaderNode>());
    rclcpp::shutdown();
    return 0;
}