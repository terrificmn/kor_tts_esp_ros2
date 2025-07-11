# MELO KOR-TTS 

mportant Notes:

    Python Script Directory: Make sure python_script_dir in main.cpp points to the correct location of your kor_tts_service.py file inside the Docker container.

    initialize_tts_model() in Python: Ensure your kor_tts_service.py's KoreanTTSService class has an initialize_tts_model() method (as discussed in the previous Python code block) which performs the actual, potentially long-running, model loading. The C++ code now calls this method.

    GIL Management:

        PyGILState_Ensure(): Acquires the GIL (Global Interpreter Lock). You must call this before any Python C API functions if you are calling Python from a C++ thread that isn't the main thread that initialized Python, or if you're making a sequence of Python calls that could be interrupted by other C++ threads.

        PyGILState_Release(): Releases the GIL. You must call this after you're done with Python C API calls in a section of code, to allow other Python threads (or C++ threads that want to acquire the GIL) to run.

    Error Handling: The C++ code includes basic error handling for Python API calls. If you encounter issues, check the std::cerr output and PyErr_Print() for Python-specific error messages.

    rclcpp Re-enablement: If you plan to re-enable ROS 2 functionality (commented out in the provided code), the threading approach will integrate smoothly with rclcpp::spin(). The background thread for Python initialization will run concurrently while rclcpp::spin() processes ROS messages.

This setup provides a responsive main application while handling the potentially slow Python model loading in the background.



///필수
////////////////////////////////////////////////////////////////////////
// sudo apt install python3-pip

// pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
// pip3 install espnet
// pip3 install soundfile
// sudo apt update
// sudo apt install libportaudio2 libportaudiocpp0 portaudio19-dev
// sudo apt install libasound-dev # ALSA development files, often a dependency for PortAudio on Linux

////////////////////////////////////////////////////////////////////////



// sudo apt install libsndfile1

// pip3 install pybind11

// .asoundrc 파일을 home 디렉토리 이하에 만들어준 후에 아래와 같은 형식으로 만들어 준다.
// ```
// pcm.!default {
//   type plug
//   slave {
//     pcm "hw:1,0"
//   }
// }
// ctl.!default {
//   type hw
//   card 1
// }
// ```

/// Melo tts 사용시 깃허브로 설치해야함
// 테스트: 압축파일에 풀면 cache 디렉토리 이하를  ~/.cache/ 에 넣어주기 (checkpoint.phn 관련)
// https://github.com/myshell-ai/MeloTTS/blob/main/docs/install.md#linux-and-macos-install
// 설치
// git clone https://github.com/myshell-ai/MeloTTS.git
// cd MeloTTS
// pip install -e .
// python3S -m unidic download

/// *unidic 은 여기에 Downloaded UniDic v3.1.0+2021-08-31 to /home/docker_humble/.local/lib/python3.10/site-packages/unidic/dicdir


// espnet 과 충돌이 발생하지만 그냥 무시할 수 있다.
// espnet 202412 requires librosa==0.9.2, but you have librosa 0.9.1 which is incompatible.
// espnet 202412 requires pypinyin<=0.44.0, but you have pypinyin 0.50.0 which is incompatible.

// 최초 실행 시 
// Collecting python-mecab-ko
//   Downloading python_mecab_ko-1.3.7-cp310-cp310-manylinux_2_17_x86_64.manylinux2014_x86_64.whl (577 kB)
//      ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 577.1/577.1 KB 14.9 MB/s eta 0:00:00
// Collecting python-mecab-ko-dic
//   Downloading python_mecab_ko_dic-2.1.1.post2-py3-none-any.whl (34.5 MB)
//      ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 34.5/34.5 MB 56.3 MB/s eta 0:00:00
// 2개의 패키지를 설치한다. 

// 일단 ~/.cache/ 이하의 huggingface 에는 복사로 가능한 듯 하다. 
