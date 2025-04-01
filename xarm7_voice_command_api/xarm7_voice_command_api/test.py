from vosk import Model, KaldiRecognizer
import pyaudio

model = Model("/home/shared_folder/xarm7_ws/src/xarm_ros2/xarm7_voice_command_api/models/vosk-model-small-en-us-0.15")
# model = Model("/home/shared_folder/xarm7_ws/src/xarm_ros2/xarm7_voice_command_api/models/vosk-model-en-us-0.22")
# model = Model("/home/shared_folder/xarm7_ws/src/xarm_ros2/xarm7_voice_command_api/models/vosk-model-en-us-0.22-lgraph")

recognizer = KaldiRecognizer(model, 16000)

mic = pyaudio.PyAudio()

stream = mic.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)

stream.start_stream()

while True:
    data = stream.read(4096)

    if recognizer.AcceptWaveform(data):
        text = recognizer.Result()
        print(f"Recognized: {text}")