""" 
Copyright 2024 Virtual Reality Labs DKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================
 """

#! usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading
import json
import pyaudio
from vosk import Model, KaldiRecognizer

from xarm_msgs.action import VoiceCommand

threading.Thread(target=lambda: rclpy.init()).start()
action_client = ActionClient(Node("vosk_interface_node"), VoiceCommand, "voice_command_server")

# Initialize Vosk model
# model = Model("/home/newton/Downloads/Compressed/vosk-model-small-en-us-0.15)
model = Model("/home/newton/Downloads/Compressed/vosk-model-en-us-0.22")
# model = Model("/home/newton/Downloads/Compressed/vosk-model-en-us-0.22-lgraph")

recognizer = KaldiRecognizer(model, 16000)

# Setup PyAudio
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=4096)
stream.start_stream()

def listen_and_process():
    while True:
        data = stream.read(4096, exception_on_overflow=False)
        if recognizer.AcceptWaveform(data):
            result = json.loads(recognizer.Result())
            command_text = result['text']
            process_command(command_text)

def process_command(command_text):
    if "home" in command_text.lower():
        goal = VoiceCommand.Goal()
        goal.task_code = 1
        action_client.send_goal_async(goal)
    elif "pick" in command_text.lower():
        goal = VoiceCommand.Goal()
        goal.task_code = 2
        action_client.send_goal_async(goal)
    elif "place" in command_text.lower():
        goal = VoiceCommand.Goal()
        goal.task_code = 3
        action_client.send_goal_async(goal)
    elif "open gripper" in command_text.lower():
        goal = VoiceCommand.Goal()
        goal.task_code = 4
        action_client.send_goal_async(goal)
    elif "close gripper" in command_text.lower():
        goal = VoiceCommand.Goal()
        goal.task_code = 5
        action_client.send_goal_async(goal)
    else:
        print("Unrecognized command:", command_text)
        # Add more error handling as needed
    # Add more commands as needed

if __name__ == "__main__":
    listen_and_process()
