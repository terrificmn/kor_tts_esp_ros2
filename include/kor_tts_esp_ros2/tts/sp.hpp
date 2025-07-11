#ifndef SP_HPP
#define SP_HPP

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <memory>
#include <thread> // For std::thread
#include <atomic> // For std::atomic<bool>
#include <chrono>
#include <mutex>
#include <portaudio.h>
// Headers for Python C API
#include <Python.h>


#define PA_CHECK(err) if ((err) != paNoError) throw std::runtime_error(Pa_GetErrorText(err));

// WAV header structure (simplified for common 16-bit PCM mono/stereo)
struct WavHeader {
    char        riff_header[4]; // "RIFF"
    uint32_t    wav_size;       // Size of the wav portion of the file, which follows this number
    char        wave_header[4]; // "WAVE"
    char        fmt_header[4];  // "fmt "
    uint32_t    fmt_chunk_size; // Size of the fmt chunk (16 for PCM)
    uint16_t    audio_format;   // Audio format 1=PCM, 6=mulaw, 7=alaw, etc.
    uint16_t    num_channels;   // Number of channels 1=mono, 2=stereo
    uint32_t    sample_rate;    // Sample rate
    uint32_t    byte_rate;      // SampleRate * NumChannels * BitsPerSample/8
    uint16_t    block_align;    // NumChannels * BitsPerSample/8
    uint16_t    bits_per_sample;// Number of bits per sample
    char        data_header[4]; // "data"
    uint32_t    data_size;      // Size of the data section
};


class MyKoreanTTSNode : public rclcpp::Node {
public:
    // Forward declare PyObject pointers
    PyObject* pModule = nullptr;
    PyObject* pKoreanTTSServiceClass = nullptr;
    PyObject* pServiceInstance = nullptr;
    PyObject* pSynthesizeMethod = nullptr;

    // rclcpp::TimerBase::SharedPtr timer_;
    std::string shared_app_path = ament_index_cpp::get_package_share_directory("kor_tts_esp_ros2");
    /// only dir path is needed.
    std::string python_script_dir = shared_app_path + "/script-py";

public:
    MyKoreanTTSNode();
    ~MyKoreanTTSNode();
    void ttsCb(const std_msgs::msg::String& msg);
    void timeoutCb();
    bool isTTSInitialized();
    void queueAudioData(const std::string& text_to_speak);
    void queueAudioStr(const std::string& cb_str);

    void audioPlaybackLoop();
    void startPlaybackStream();


private:
    void initialize_python_tts_service_threaded();
    void audioPlaybackThreadLoop();
    std::vector<char> synthesize_korean_text(const std::string& text);
    void finalize_python_tts_service();
    void playWavAudio(const std::vector<char>& wav_data);
    void endAudio();

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_tts;
    rclcpp::TimerBase::SharedPtr timeout_timer;
    std::chrono::milliseconds timeout_sec {2000};
    std::thread python_init_thread; // Thread for Python initialization
    std::thread audio_playback_thread;
    std::condition_variable audio_cv;
    std::mutex audio_que_mutex;
    std::atomic<bool> python_tts_initialized{false}; // Flag to check if Python TTS is ready
    std::atomic<bool> is_playback_active {false};
    std::atomic<bool> stop_playback_thread {false};
    std::queue<std::vector<char>> audio_data_que;
    std::queue<std::string> q_received_str;
    // Buffer for the current audio being played
    std::vector<char> current_playback_buffer;
    size_t current_playback_pos = 0;
    PaStream* audio_stream;

};

#endif