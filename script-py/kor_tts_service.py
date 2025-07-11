# korean_tts_service.py
import io
import numpy as np
import soundfile as sf
import torch, os, time

# print(os.environ.get('CUDA_PATH'))

# Import the Text2Speech class from ESPnet
from espnet2.bin.tts_inference import Text2Speech
from espnet2.utils.types import str_or_none # Good practice for argument types

# --- Configuration for your ESPnet Model ---
# You'll need to know the exact paths to your downloaded files inside the Docker container.
# Assuming you've copied them into your ROS 2 package's python directory:
# e.g., your_ros2_pkg/python/model_name/model.pth
#       your_ros2_pkg/python/model_name/config.yaml

# Set these paths relative to where your korean_tts_service.py script will run
# (which is usually the root of your Python module, e.g., in install/your_ros2_pkg/lib/python3.X/site-packages)
# It's better to pass these paths as arguments during initialization if possible,
# or ensure they are properly accessible relative to the script.

# For simplicity, let's assume your model files are directly in the same directory as this script
# OR you pass them as constructor arguments to KoreanTTSService.
# For demonstration, let's hardcode for now (you should make this configurable):
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(CURRENT_DIR, "songhee") # korean_tts_model Create this directory
MODEL_FILE = os.path.join(MODEL_DIR, "1000epoch.pth") # Renamed your .pth file?
CONFIG_FILE = os.path.join(MODEL_DIR, "config.yaml")

# Check if files exist (good for debugging paths)
if not os.path.exists(MODEL_FILE):
    print(f"ERROR: Model file not found at {MODEL_FILE}")
if not os.path.exists(CONFIG_FILE):
    print(f"ERROR: Config file not found at {CONFIG_FILE}")


# Korean G2P and Tokenizer: For ESPnet models, the Text2Speech class usually handles
# this internally based on the model's configuration. You might not need a separate
# `korean_g2p_and_tokenize` function unless you need custom pre-processing.
# The `text2speech` object will have its own text processing pipeline.
# We'll keep a dummy for now, but in many cases, it's not needed for ESPnet.
def korean_g2p_and_tokenize(text: str, hps=None) -> torch.Tensor:
    # This function is likely NOT needed if Text2Speech handles it directly.
    # We'll just pass the raw text to text2speech(input_text).
    print(f"Python: (Dummy) G2P/Tokenization for: '{text}' - ESPnet's Text2Speech handles this.")
    return text # Return the text directly for Text2Speech

class KoreanTTSService:
    def __init__(self, model_file_path=MODEL_FILE, config_file_path=CONFIG_FILE, speed_factor=1.0):
        print("Python: KoreanTTSService initializing (loading ESPnet VITS model)...")
        
        # Construct the args_override list for the speed factor
        # The exact parameter name might be '--duration_factor', '--alpha', etc.
        # '--duration_factor' is a very common one for VITS/FastSpeech models in ESPnet.
        # You might need to confirm this from the specific ESPnet model's documentation or its config.yaml.

        # Load the Text2Speech model
        # 'vocoder_tag' can often be None for end-to-end models like VITS
        # 'model_file' and 'train_config' should be paths to your downloaded files
        try:
            self.text2speech = Text2Speech.from_pretrained(
                # vocoder_tag=str_or_none(""), # Or "" if None is not allowed
                model_file=str_or_none(model_file_path),
                train_config=str_or_none(config_file_path),
                # If your model requires a specific device (e.g., CUDA), add it here
                # device="cuda" if torch.cuda.is_available() else "cpu"
                speed_control_alpha=speed_factor, 
            )
            # self.text2speech.freeze_parameters(
            #     p=True # Freeze parameters for inference mode
            # )
            # Get the sample rate from the loaded model
            self.sample_rate = self.text2speech.fs
            
            print(f"Python: ESPnet Text2Speech model loaded successfully. Sample Rate: {self.sample_rate} Hz")
        except Exception as e:
            print(f"ERROR: Failed to load ESPnet Text2Speech model: {e}")
            raise # Re-raise to ensure C++ catches initialization failure
        
        finally:
            # Make sure this print always runs
            print("PythonTTS: initialize_tts_model FINISHED. (Inside Python)")

        # The vocab_map and dummy model are no longer needed
        # self.vocab_map = {...}
        # self.model = DummyVITSModel()
        # self.text2speech.eval() # Set to evaluation mode

    def synthesize_speech(self, text: str) -> bytes:
        """
        Synthesizes speech from Korean text and returns raw audio bytes (WAV format).
        """
        with torch.no_grad():
            print(f"Python: Synthesizing text: '{text}' using ESPnet model.")
            
            # The ESPnet example directly passes the input_text
            # It will handle G2P and tokenization internally.
            # No need for korean_g2p_and_tokenize here unless you have custom preprocessing.
            # input_ids = korean_g2p_and_tokenize(text, self.hps) # This line is likely removed
            
            start_time = time.time()
            # Perform inference. The result is a dictionary, and 'wav' is the audio.
            # The 'wav' tensor is already a numpy array internally or easily convertible.
            speech_result = self.text2speech(text)
            audio_waveform = speech_result["wav"].view(-1).cpu().numpy() # Ensure it's 1D numpy array on CPU
            
            inference_duration = time.time() - start_time
            audio_duration = len(audio_waveform) / self.sample_rate
            rtf = inference_duration / audio_duration
            print(f"Python: Synthesis RTF = {rtf:.5f}")


            # Convert to bytes (WAV in memory)
            buffer = io.BytesIO()
            # sf.write expects float data and will convert to PCM_16 (int16)
            sf.write(buffer, audio_waveform, self.sample_rate, format='WAV', subtype='PCM_16')
            buffer.seek(0)
            
            # --- IMPORTANT FOR DEBUGGING ---
            # Save the WAV to disk here temporarily for verification
            # This is critical to ensure the Python side is working correctly
            ########### If you want to make a wav file, then uncomment the delow
            # with open("debug_tts_output_espnet.wav", "wb") as f:
            #      f.write(buffer.getvalue())
            # print("Python: Debug WAV saved to debug_tts_output_espnet.wav")
            # --- END DEBUGGING ---

            return buffer.read()


# ros2 topic pub /kor_tts std_msgs/msg/String data:\ 이동중,입니다.\  --rate 2
# ros2 topic pub /kor_tts std_msgs/msg/String data:\ 이동중.\  --rate 2
# ros2 topic pub /kor_tts std_msgs/msg/String data:\ 이동중,입니다.\'\ --rate 2
# ros2 topic pub /kor_tts std_msgs/msg/String data:\ 이동합니다.\'\ 
# ros2 topic pub /kor_tts std_msgs/msg/String data:\ 무빙.\'\ 
# ros2 topic pub /kor_tts std_msgs/msg/String data:\ 이동할까요?\'\ 
# ros2 topic pub /kor_tts std_msgs/msg/String data:\ 목적지에,도착합니다.파킹을 하고 있습니다.\'\ 
# ros2 topic pub /kor_tts std_msgs/msg/String data:\ 이제,곧,,도착합니다..파킹을,하고있습니다.\'\ 
#### dummy sound
# import sys
# import time

# class KoreanTTSService:
#     def __init__(self):

#     def initialize_tts_model(self):
#         # Simulate a short delay without loading complex models
#         print("Python: Initializing dummy TTS model...")
#         time.sleep(1) # Simulate some work
#         print("Python: Dummy TTS model initialized successfully.")
#         # DO NOT import espnet, torch, etc., here for this test

#     def synthesize_speech(self, text):
#         print(f"Python: Dummy synthesis requested for '{text}'")
#         # Simulate audio data without actual synthesis
#         dummy_audio = b'\x00\x01\x02\x03\x04\x05' * 1000 # Just some dummy bytes
#         print(f"Python: Dummy synthesis finished for '{text}'")
#         return dummy_audio