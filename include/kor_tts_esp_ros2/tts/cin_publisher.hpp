#ifndef CIN_PUBLISHER_HPP
#define CIN_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread> // For std::thread

class CinPublisher : public rclcpp::Node {
public:
    CinPublisher();
    ~CinPublisher();
    void cinThread();

private:
    std::thread cin_thread;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_kor_tts;

};

#endif