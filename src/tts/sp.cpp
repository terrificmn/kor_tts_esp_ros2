#include "kor_tts_esp_ros2/tts/sp.hpp"

// Define your PortAudio callback function
// This function must be static or a free function if you're not using C++17 lambda capture with PortAudio
// For simplicity, we'll use a static method and pass 'this' pointer via userData
static int paCallback(const void *inputBuffer, void *outputBuffer,
                        unsigned long framesPerBuffer,
                        const PaStreamCallbackTimeInfo* timeInfo,
                        PaStreamCallbackFlags statusFlags,
                        void *userData);


MyKoreanTTSNode::MyKoreanTTSNode() : Node("kor_tts_node") {
    std::cout << "MyKoreanTTSNode started. Launching Python TTS initialization thread..." << std::endl;
    // 1. Initialize PortAudio
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        RCLCPP_ERROR(this->get_logger(), "PortAudio initialization failed: %s", Pa_GetErrorText(err));
        // node_running_ = false;
        return;
    }
    RCLCPP_INFO(this->get_logger(), "PortAudio initialization succeed");

    // Launch Python initialization in a separate thread
    this->python_init_thread = std::thread(&MyKoreanTTSNode::initialize_python_tts_service_threaded, this);
    this->audio_playback_thread = std::thread(&MyKoreanTTSNode::audioPlaybackThreadLoop, this);

    this->sub_tts =  this->create_subscription<std_msgs::msg::String>(
                                "kor_tts", 10,
                                std::bind(&MyKoreanTTSNode::ttsCb, this,
                                std::placeholders::_1));

     // Create timer but don't start it yet
    this->timeout_timer = this->create_wall_timer(
                                this->timeout_sec, std::bind(&MyKoreanTTSNode::timeoutCb, this));
}

MyKoreanTTSNode::~MyKoreanTTSNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down TTS Player Node.");
    // Signal the playback thread to stop
    {
        std::lock_guard<std::mutex> lock(this->audio_que_mutex);
        this->stop_playback_thread = true;
        audio_cv.notify_all(); // Wake up the thread
    }

    if (this->python_init_thread.joinable()) {
        this->python_init_thread.join(); // Wait for the initialization thread to complete
    }
    if (this->audio_playback_thread.joinable()) {
        this->audio_playback_thread.join(); // Wait for the thread to finish
    }

    // Close and terminate PortAudio
    if (this->audio_stream != nullptr) {
        Pa_CloseStream(this->audio_stream);
    }
    this->finalize_python_tts_service(); // Clean up Python resources
    
    // Stop and terminate PortAudio
    if (this->audio_stream) {
        Pa_StopStream(this->audio_stream);
        Pa_CloseStream(this->audio_stream);
    }
    PaError pa_term_err = Pa_Terminate();
    if (pa_term_err != paNoError) {
        std::cerr << "PortAudio termination failed: " << Pa_GetErrorText(pa_term_err) << std::endl;
    }
    std::cout << "MyKoreanTTSNode shutdown complete." << std::endl;
}

void MyKoreanTTSNode::ttsCb(const std_msgs::msg::String& msg) {
    if(!this->isTTSInitialized()) {
        std::cerr << "Not initialized yet." << std::endl;
        return;
    }

    if(msg.data.empty()) {
        std::cerr << "empty not allowed." << std::endl;
        return;
    }
    // this->queueAudioData(msg.data);
    // std::lock_guard<std::mutex> lock(this->audio_que_mutex);
    // if (q_received_str.size()>=2) return;
    this->queueAudioStr(msg.data);
    this->timeout_timer->reset(); // reset
}

void MyKoreanTTSNode::timeoutCb() {
    RCLCPP_WARN(this->get_logger(), "No messages received for %ld ms - assuming stream ended", 
                    this->timeout_sec.count());
    this->endAudio();
}

bool MyKoreanTTSNode::isTTSInitialized() {
    return this->python_tts_initialized.load();
}


void MyKoreanTTSNode::queueAudioStr(const std::string& cb_str) {
    std::lock_guard<std::mutex> lock(this->audio_que_mutex);
    this->q_received_str.push(cb_str);
    std::cout << "\tqueueAudioStr pushed!" << std::endl;
    ///FYI: if audio_data_que has been pushed, then audioPlaybackThreadLoop() wakes up and works
    this->audio_cv.notify_one();
}

void MyKoreanTTSNode::endAudio() {
    std::cout << "Stopped checking\n";
    this->timeout_timer->cancel(); // Stop checking
    while(!this->q_received_str.empty()) {\
        if(this->q_received_str.size() > 2) {
            /// Leave the last 2
            ///TODO: 다른 플래그를 받아서 ex. mandatory 등, 이 있다면 남겨준다.
            this->q_received_str.pop();
        } else {
            // std::cout << "wait for popping q_received_str" << std::endl;
        }
    }
    while(!this->audio_data_que.empty()) {
        this->audio_data_que.pop();
    }
    std::cout << "All queue cleared." << std::endl;
}


void MyKoreanTTSNode::audioPlaybackThreadLoop() {
    std::cout << "audioPlaybackThreadLoop start" << std::endl;
    while(rclcpp::ok() && !stop_playback_thread) {
        {
            std::unique_lock<std::mutex> lock(this->audio_que_mutex);
            audio_cv.wait(lock, [this] {
                return this->stop_playback_thread || !this->q_received_str.empty();
            });
        }

        if(this->stop_playback_thread) {
            std::cout << "break the audioPlaybackLoop" << std::endl;
            break;
        }
        std::cout << "audioPlaybackThreadLoop" << std::endl;

        if(!this->q_received_str.empty()) {
            // lock.unlock(); // If unique_lock is not scoped, then unlock is needed
            std::string q_str;
            {
                std::lock_guard<std::mutex> lock_(this->audio_que_mutex);
                std::cout << "***q_received_str size: " << this->q_received_str.size() << std::endl;
                q_str = this->q_received_str.front();
                this->q_received_str.pop();
            }
            // while(!this->q_received_str.empty()) {
            //     this->q_received_str.pop();
            // }
            this->queueAudioData(q_str);
            
            // Start or restart the PortAudio stream
            this->startPlaybackStream();
        }
        // lock.lock();
    }
}

void MyKoreanTTSNode::queueAudioData(const std::string& text_to_speak) {
    if (!python_tts_initialized.load()) { // Check the atomic flag
        std::cout << "TTS service not yet initialized. Skipping synthesis for: '" << text_to_speak << "'" << std::endl;
        return;
    }

    std::cout << "Requesting synthesis for: '" << text_to_speak << "'" << std::endl;
    std::vector<char> audio_data = this->synthesize_korean_text(text_to_speak);

    if (!audio_data.empty()) {
        std::cout << "Received " << audio_data.size() << " bytes of audio data." << std::endl;
        {
            std::lock_guard<std::mutex> lock(this->audio_que_mutex);
            // while(!this->audio_data_que.empty()) {
            //     this->audio_data_que.pop();
            // }
            std::cout << "audio_data push into queue" << std::endl;
            this->audio_data_que.push(audio_data);
            std::cout << "audio_data push into queue done" << std::endl;

        }
        // this->playWavAudio(audio_data); // origin
    } else {
        std::cout << "No audio data received or synthesis failed for: '" << text_to_speak << "'" << std::endl;
        /// pop all of them
        while(!this->audio_data_que.empty()) {
            this->audio_data_que.pop();
        }
    }
}

void MyKoreanTTSNode::startPlaybackStream() {
    std::cout << "startPlaybackStream() starts" << std::endl;

    if(this->audio_data_que.empty()) {
        std::cout << "audio_data_que empty. return" << std::endl;
        return;
    }
    this->current_playback_buffer = this->audio_data_que.front();
    this->audio_data_que.pop();
    this->current_playback_pos = 0;

    if(this->audio_stream != nullptr && Pa_IsStreamActive(audio_stream) == 1) {
        Pa_AbortStream(audio_stream);
        Pa_CloseStream(audio_stream);
        this->audio_stream = nullptr;
    }

    const WavHeader* header = reinterpret_cast<const WavHeader*>(this->current_playback_buffer.data());

    // Basic validation of WAV header fields
    if (std::string(header->riff_header, 4) != "RIFF" ||
        std::string(header->wave_header, 4) != "WAVE" ||
        std::string(header->fmt_header, 4) != "fmt " ||
        std::string(header->data_header, 4) != "data" ||
        header->audio_format != 1 // Must be PCM
    ) {
        std::cerr << "Error: Invalid or unsupported WAV format." << std::endl;
        return;
    }

    int sample_rate = header->sample_rate;
    // int sample_rate = 18000; // 22050
    int num_channels = header->num_channels;
    int bits_per_sample = header->bits_per_sample;
    size_t data_offset = sizeof(WavHeader);
    size_t num_bytes = header->data_size;

    std::cout << "#####sample_rate: " << sample_rate << std::endl;

    if (this->current_playback_buffer.size() < data_offset + num_bytes) {
        std::cerr << "Error: WAV data truncated (size mismatch with header)." << std::endl;
        return;
    }

    const char* raw_audio_bytes = this->current_playback_buffer.data() + data_offset;
    // PaStream *stream = nullptr;

    try {
        PaStreamParameters outputParameters;
        outputParameters.device = Pa_GetDefaultOutputDevice();
        if (outputParameters.device == paNoDevice) {
            throw std::runtime_error("No default output device found.");
        }
        outputParameters.channelCount = num_channels;

        PaSampleFormat pa_format;
        if (bits_per_sample == 16) {
            pa_format = paInt16;
        } else if (bits_per_sample == 32 && header->audio_format == 3) { // float PCM
            pa_format = paFloat32;
        } else {
            throw std::runtime_error("Unsupported bits per sample or audio format: " +
                                        std::to_string(bits_per_sample) + " bits, format " + std::to_string(header->audio_format));
        }
        outputParameters.sampleFormat = pa_format;

        outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
        outputParameters.hostApiSpecificStreamInfo = nullptr;

        // PaError err = Pa_OpenStream(
        //     &audio_stream,
        //     nullptr, // No input
        //     &outputParameters,
        //     (double)sample_rate,
        //     paFramesPerBufferUnspecified,
        //     paNoFlag,
        //     &paCallback, // static callback function
        //     this // pass 'this' pointer
        // );

        PaError err = Pa_OpenStream(
            &audio_stream,
            nullptr, // No input
            &outputParameters,
            (double)sample_rate,
            paFramesPerBufferUnspecified,
            paNoFlag,
            nullptr,
            nullptr
        );
        PA_CHECK(err);

        err = Pa_StartStream(this->audio_stream);
        PA_CHECK(err);
        this->is_playback_active = true;

        std::cout << "Playing audio (" << num_channels << "ch, "
                    << sample_rate << "Hz, " << bits_per_sample << "bit)..." << std::endl;

        long num_frames = num_bytes / (num_channels * (bits_per_sample / 8));

        err = Pa_WriteStream(this->audio_stream, raw_audio_bytes, num_frames);
        PA_CHECK(err);

        err = Pa_StopStream(this->audio_stream);
        PA_CHECK(err);

        err = Pa_CloseStream(this->audio_stream);
        PA_CHECK(err);

        std::cout << "Audio playback finished." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "PortAudio Playback Error: " << e.what() << std::endl;
        if (this->audio_stream) {
            Pa_AbortStream(this->audio_stream);
            Pa_CloseStream(this->audio_stream);
        }
    }
}


void MyKoreanTTSNode::initialize_python_tts_service_threaded() {
    // Declare gstate here so it's in scope for finally-like cleanup
    PyGILState_STATE gstate; 
    bool gil_acquired_in_this_thread = false; // Track if we successfully called PyGILState_Ensure

    try {
        std::cout << "Python TTS thread: Initializing Python interpreter..." << std::endl;

        // Step 1: Initialize the Python interpreter.
        // This thread becomes the "main" Python thread and implicitly holds the GIL.
        Py_Initialize();
        if (!Py_IsInitialized()) {
            std::cerr << "Python TTS thread: Failed to initialize Python interpreter!" << std::endl;
            return; // Cannot proceed without initialized interpreter
        }

        // Step 2: Initialize Python's threading support.
        // This sets up the GIL for multi-threading and then *releases* the GIL
        // from the current thread (the one that called Py_Initialize()).
        // This allows other C++ threads to acquire the GIL.
        PyEval_InitThreads();
        
        // Step 3: Re-acquire the GIL for *this* thread (the initialization thread)
        // to continue interacting with Python. This call will now truly acquire it
        // from the newly set up threading system.
        gstate = PyGILState_Ensure(); 
        gil_acquired_in_this_thread = true; // Mark that GIL was successfully acquired

        std::cout << "Python TTS thread: Python interpreter and threading initialized. Acquiring GIL for task..." << std::endl;
        
        PyGILState_Release(gstate); // And immediately release it
        std::cout << "Python: Main thread explicitly released GIL after init." << std::endl;
        // --- All Python C API interactions go here ---

        std::cout << "python_script_dir: " << this->python_script_dir << std::endl;

        // Add script_dir to Python path
        PyObject* sysPath = PySys_GetObject("path");
        if (sysPath == nullptr) {
            PyErr_Print();
            throw std::runtime_error("Failed to get sys.path"); // Throw to enter catch block
        }
        PyObject* path = PyUnicode_DecodeFSDefault(python_script_dir.c_str());
        if (PyList_Append(sysPath, path) == -1) {
            PyErr_Print();
            Py_XDECREF(path);
            throw std::runtime_error("Failed to append script directory to sys.path");
        }
        Py_XDECREF(path); // Release path object

        // Import the Python module
        pModule = PyImport_ImportModule("kor_tts_service");
        if (pModule == nullptr) {
            PyErr_Print();
            throw std::runtime_error("Failed to load 'kor_tts_service' Python module.");
        }

        // Get the class object from the module
        pKoreanTTSServiceClass = PyObject_GetAttrString(pModule, "KoreanTTSService");
        if (pKoreanTTSServiceClass == nullptr || !PyCallable_Check(pKoreanTTSServiceClass)) {
            PyErr_Print();
            throw std::runtime_error("Failed to get callable 'KoreanTTSService' class from module.");
        }

        // Instantiate the class
        pServiceInstance = PyObject_CallObject(pKoreanTTSServiceClass, nullptr);
        if (pServiceInstance == nullptr) {
            PyErr_Print();
            throw std::runtime_error("Failed to instantiate KoreanTTSService.");
        }

        // Get the synthesize_speech method
        pSynthesizeMethod = PyObject_GetAttrString(pServiceInstance, "synthesize_speech");
        if (pSynthesizeMethod == nullptr || !PyCallable_Check(pSynthesizeMethod)) {
            PyErr_Print();
            throw std::runtime_error("Failed to get callable 'synthesize_speech' method.");
        }

        // --- End of Python C API interactions ---

        this->python_tts_initialized.store(true); // Atomically set flag to true
        std::cout << "Python TTS thread: Service initialized and ready for synthesis." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Python TTS thread ERROR: " << e.what() << std::endl;
        // Ensure atomic flag is false on error
        this->python_tts_initialized.store(false);
        // Clean up PyObject pointers on error path to prevent memory leaks
        // Only dereference if they might have been assigned.
        if (pSynthesizeMethod) { Py_XDECREF(pSynthesizeMethod); pSynthesizeMethod = nullptr; }
        if (pServiceInstance) { Py_XDECREF(pServiceInstance); pServiceInstance = nullptr; }
        if (pKoreanTTSServiceClass) { Py_XDECREF(pKoreanTTSServiceClass); pKoreanTTSServiceClass = nullptr; }
        if (pModule) { Py_XDECREF(pModule); pModule = nullptr; }
    }

    // --- Crucial: Ensure GIL is released in all exit paths ---
    // Only release GIL if it was successfully acquired by this thread
    if (Py_IsInitialized() && gil_acquired_in_this_thread) {
        PyGILState_Release(gstate);
        std::cout << "Python TTS thread: GIL released." << std::endl;
    } else {
            std::cout << "Python TTS thread: GIL not acquired or not initialized, so not released." << std::endl;
    }
}



    // Function to call the Python TTS service
std::vector<char> MyKoreanTTSNode::synthesize_korean_text(const std::string& text) {
    std::vector<char> audio_data;

    if (!pSynthesizeMethod) {
        std::cerr << "Python TTS service not initialized or method not found." << std::endl;
        return audio_data;
    }

    // Acquire the GIL before interacting with Python
    PyGILState_STATE gstate = PyGILState_Ensure(); // <-- THIS IS THE LIKELY HANG POINT
    std::cout << "TTS: synthesize_korean_text: GIL successfully acquired for synthesis. (Line after PyGILState_Ensure)" << std::endl; 


    // Convert C++ string to Python string
    PyObject* pArgs = Py_BuildValue("(s)", text.c_str());
    if (pArgs == nullptr) {
        PyErr_Print();
        std::cerr << "Failed to build arguments for Python function." << std::endl;
        PyGILState_Release(gstate);
        return audio_data;
    }

    // Call the Python method
    PyObject* pResult = PyObject_CallObject(pSynthesizeMethod, pArgs);
    Py_DECREF(pArgs);

    if (pResult == nullptr) {
        PyErr_Print();
        std::cerr << "Python 'synthesize_speech' method failed." << std::endl;
        PyGILState_Release(gstate);
        return audio_data;
    }

    // Process the result (assuming it's a bytes object)
    if (PyBytes_Check(pResult)) {
        Py_ssize_t size = PyBytes_Size(pResult);
        const char* buffer = PyBytes_AsString(pResult);
        if (buffer) {
            audio_data.assign(buffer, buffer + size);
        } else {
            PyErr_Print();
            std::cerr << "Failed to get string from Python bytes object." << std::endl;
        }
    } else {
        std::cerr << "Python 'synthesize_speech' did not return bytes object." << std::endl;
    }

    Py_DECREF(pResult);
    PyGILState_Release(gstate); // Release the GIL

    return audio_data;
}

// Function to deinitialize Python interpreter and release resources
void MyKoreanTTSNode::finalize_python_tts_service() {
    // Acquire GIL before finalizing Python interpreter
    PyGILState_STATE gstate = PyGILState_Ensure();

    // Release Python objects (in reverse order of creation, good practice)
    if (pSynthesizeMethod) { Py_DECREF(pSynthesizeMethod); pSynthesizeMethod = nullptr; }
    if (pServiceInstance) { Py_DECREF(pServiceInstance); pServiceInstance = nullptr; }
    if (pKoreanTTSServiceClass) { Py_DECREF(pKoreanTTSServiceClass); pKoreanTTSServiceClass = nullptr; }
    if (pModule) { Py_DECREF(pModule); pModule = nullptr; }

    // Finalize the Python interpreter
    if (Py_IsInitialized()) {
        Py_Finalize();
        std::cout << "Python interpreter finalized." << std::endl;
    } else {
        std::cout << "***ELSE Python interpreter finalized." << std::endl;
    }
    // PyGILState_Release(gstate); // Release the GIL after finalization
}

/// Function to play raw audio data (WAV format assumed)
void MyKoreanTTSNode::playWavAudio(const std::vector<char>& wav_data) {
    if (wav_data.empty()) {
        std::cerr << "No audio data to play." << std::endl;
        return;
    }

    if (wav_data.size() < sizeof(WavHeader)) {
        std::cerr << "Error: WAV data too small to contain a header." << std::endl;
        return;
    }

    const WavHeader* header = reinterpret_cast<const WavHeader*>(wav_data.data());

    // Basic validation of WAV header fields
    if (std::string(header->riff_header, 4) != "RIFF" ||
        std::string(header->wave_header, 4) != "WAVE" ||
        std::string(header->fmt_header, 4) != "fmt " ||
        std::string(header->data_header, 4) != "data" ||
        header->audio_format != 1 // Must be PCM
    ) {
        std::cerr << "Error: Invalid or unsupported WAV format." << std::endl;
        return;
    }

    int sample_rate = header->sample_rate;
    int num_channels = header->num_channels;
    int bits_per_sample = header->bits_per_sample;
    size_t data_offset = sizeof(WavHeader);
    size_t num_bytes = header->data_size;

    std::cout << "#####sample_rate: " << sample_rate << std::endl;

    if (wav_data.size() < data_offset + num_bytes) {
        std::cerr << "Error: WAV data truncated (size mismatch with header)." << std::endl;
        return;
    }

    const char* raw_audio_bytes = wav_data.data() + data_offset;
    PaStream *stream = nullptr;

    try {
        PaStreamParameters outputParameters;
        outputParameters.device = Pa_GetDefaultOutputDevice();
        if (outputParameters.device == paNoDevice) {
            throw std::runtime_error("No default output device found.");
        }
        outputParameters.channelCount = num_channels;

        PaSampleFormat pa_format;
        if (bits_per_sample == 16) {
            pa_format = paInt16;
        } else if (bits_per_sample == 32 && header->audio_format == 3) { // float PCM
            pa_format = paFloat32;
        } else {
            throw std::runtime_error("Unsupported bits per sample or audio format: " +
                                        std::to_string(bits_per_sample) + " bits, format " + std::to_string(header->audio_format));
        }
        outputParameters.sampleFormat = pa_format;

        outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
        outputParameters.hostApiSpecificStreamInfo = nullptr;

        PaError err = Pa_OpenStream(
            &stream,
            nullptr, // No input
            &outputParameters,
            (double)sample_rate,
            paFramesPerBufferUnspecified,
            paNoFlag,
            nullptr,
            nullptr
        );
        PA_CHECK(err);

        err = Pa_StartStream(stream);
        PA_CHECK(err);

        std::cout << "Playing audio (" << num_channels << "ch, "
                    << sample_rate << "Hz, " << bits_per_sample << "bit)..." << std::endl;

        long num_frames = num_bytes / (num_channels * (bits_per_sample / 8));

        err = Pa_WriteStream(stream, raw_audio_bytes, num_frames);
        PA_CHECK(err);

        err = Pa_StopStream(stream);
        PA_CHECK(err);

        err = Pa_CloseStream(stream);
        PA_CHECK(err);

        std::cout << "Audio playback finished." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "PortAudio Playback Error: " << e.what() << std::endl;
        if (stream) {
            Pa_AbortStream(stream);
            Pa_CloseStream(stream);
        }
    }
}

