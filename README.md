# KOR-TTS 
한국어 TTS , ESPNET

## 의존성 
파이썬 관련 패키지 설치
```shell
sudo apt update
sudo apt install python3-pip

pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip3 install espnet
pip3 install soundfile
sudo apt install libportaudio2 libportaudiocpp0 portaudio19-dev
sudo apt install libasound-dev # ALSA development files, often a dependency for PortAudio on Linux
```
> torch 는 cpu 버전


## sound 관련 설정
현재 사운드 관련 장치 확인
```
aplay -l
```

여기에서 봐야할 부분은  card 와 , device 의 번호  

자신의 home 디렉토리 이하에 ~/ .asoundrc 파일 생성한 후 아래 처럼 만들어 준다.   
`vi ~/.asoundrc`
```
pcm.!default {
    type plug
    slave {
        pcm "hw:1,0"
    }
}
ctl.!default {
    type hw
    card 1
}
```

## pre-trained 모델 
모델 파일을 다운로드 후에 script-py/songhee 디렉토리 이하에 복사  

[huggingface 에서 다운로드](https://huggingface.co/songhee/kss_tts_train_full_band_vits_raw_phn_korean_cleaner_korean_jaso/tree/main)   

## 빌드 
`colcon build` 또는 
```
colcon build --symlink-install --packages-select kor_tts_esp_ros2
```

## 실행
```
ros2 run kor_tts_esp_ros2 kor_tts_esp_ros2_node 
```

아래 메세지까지 나오게 되면 성공적으로 실행이 되었고, 바로 입력을 할 수가 있다. 
```
PythonTTS: initialize_tts_model FINISHED. (Inside Python)
Python TTS thread: Service initialized and ready for synthesis.
```

터미널에 한글로 입력을 해준다. 띄어쓰기를 하게 되면 한 단어씩 끊어지므로 마지막에 q 눌러준다. 
예)
```
안녕하세요. q
```
또는 한 단어씩 입력하게 되면 아래 처럼 나오게 되는데 최종적으로 q 입력해야 완료가 된다.   
예)
```
안녕하세요?
Please type. last word q
날씨가 참 좋아요
Please type. last word q
Please type. last word q
Please type. last word q
q
```

> Please type. last word q 는 출력으로 나오는 로그

ros 토픽으로 보내기, 한번만
```
ros2 topic pub /kor_tts std_msgs/msg/String "data: '안녕하세요. 날씨가, 참 좋아요'" --once
```

rate 지정,
```
ros2 topic pub /kor_tts std_msgs/msg/String "data: '안녕하세요. 날씨가, 참 좋아요'" --rate 0.5
```

> ros2 는 YAML 파싱을 사용하므로 shell escape 를 사용하면 에러가 발생하므로 data 자체를 쌍따옴표로 묶고  
실제 스트링에는 따옴표로 묶어 준다.   


## 라이센스
[CC-BY-NC-SA-4.0](https://spdx.org/licenses/CC-BY-NC-SA-4.0)  
Creative Commons Attribution Non Commercial Share Alike 4.0    
