#include "kor_tts_esp_ros2/tts/cin_publisher.hpp"

CinPublisher::CinPublisher() : Node("cin_publisher"), 
                                cin_thread(&CinPublisher::cinThread, this) {
    //
    this->pub_kor_tts = this->create_publisher<std_msgs::msg::String>("kor_tts", 1);
}

CinPublisher::~CinPublisher() {
    if(this->cin_thread.joinable()) {
        this->cin_thread.join(); // Wait for the initialization thread to complete
    }
}

void CinPublisher::cinThread() {
    // rclcpp:Rate lr(1);
    while(rclcpp::ok()) {
        std::string input_str;
        std::string whole_str;
        for(;;) {
            std::cout << "Please type. last word q\n";
            std::cin >> input_str;
            if(input_str == "q") {
                break;
            }
            whole_str = whole_str + " " + input_str;
        }

        std_msgs::msg::String msg;
        msg.data = whole_str;
        this->pub_kor_tts->publish(msg);
        std::cout << "msg sent!" << std::endl;
    }
}