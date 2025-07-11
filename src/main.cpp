#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <portaudio.h>
#include <chrono>
#include <thread>
#include "kor_tts_esp_ros2/tts/sp.hpp"
#include "kor_tts_esp_ros2/tts/cin_publisher.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Instantiate the node, which starts the Python initialization in a separate thread
    auto korTTs = std::make_shared<MyKoreanTTSNode>();
    auto cinPub = std::make_shared<CinPublisher>();

    std::cout << "Main thread: Continuing execution while Python TTS initializes..." << std::endl;
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(korTTs);
    executor.add_node(cinPub);
    executor.spin();

    /// for single
    // rclcpp::spin(korTTs);
    rclcpp::shutdown();

    std::cout << "bye" << std::endl;
    return 0;
}

