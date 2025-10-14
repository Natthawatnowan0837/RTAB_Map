#!/usr/bin/env python3
import rclpy
import torch
import soundfile as sf
import numpy as np
import sounddevice as sd
import speech_recognition as sr
from rclpy.node import Node
from my_command_pkg.srv import Command
from my_command_pkg.srv import SendPosition
from gtts import gTTS
from transformers import WhisperProcessor, WhisperForConditionalGeneration

model_text = "biodatlab/whisper-th-small-combined"
processor = WhisperProcessor.from_pretrained(model_text)
whisper_model = WhisperForConditionalGeneration.from_pretrained(model_text)
device = "cuda" if torch.cuda.is_available() else "cpu"
whisper_model.to(device)


class RobotClient(Node):
    def __init__(self):
        super().__init__('node_client')
        self.client_voice_speech = self.create_client(Command, 'voice_speech')
        # self.client_nevigetion = self.create_client(SendPosition, 'nevigetion')

        while not self.client_voice_speech.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to voice_speech...')
        # while not self.client_nevigetion.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for service to nevigetion...')

        self.get_logger().info('Service is available.')
        self.timer = self.create_timer(5.0, self.timer_callback)

    def record_audio(self, filename="user.wav", fs=16000):
        input("\nPress Enter for recording 5 second...")
        print("Recording...")
        recording = sd.rec(int(fs * 5), samplerate=fs, channels=1, dtype='float32')
        sd.wait()  #
        sf.write(filename, recording, fs)

    def listen_command(self):
        self.record_audio()
        speech, sample_rate = sf.read("user.wav")
        speech = speech.astype(np.float32)
        inputs = processor(speech, sampling_rate=sample_rate, return_tensors="pt").to(device)

        with torch.no_grad():
            predicted_ids = whisper_model.generate(inputs.input_features)

        transcription = processor.batch_decode(predicted_ids, skip_special_tokens=True)[0]
        print(f"\nYou say: {transcription}")
        return transcription.lower()

    def timer_callback(self):
        if hasattr(self, 'future') and not self.future.done():
            return 
        command_text = self.listen_command()
        request = Command.Request()
        request.command = command_text  

        self.get_logger().info(f"Sending request: command='{request.command}'")
        self.future = self.client_voice_speech.call_async(request)  # แก้ชื่อ client
        self.future.add_done_callback(self.response_callback)


    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Response received: x={response.pos_x}, y={response.pos_y}, z={response.pos_z}"
            )
            
            req_pos = SendPosition.Request()
            req_pos.send_x = response.pos_x
            req_pos.send_y = response.pos_y
            req_pos.send_z = response.pos_z

            self.future_pos = self.client_nevigetion.call_async(req_pos)
            self.future_pos.add_done_callback(self.navigation_callback)

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def navigation_callback(self, future):
        try:
            res = future.result()
            self.get_logger().info(f"Navigation service responded: {res}")
        except Exception as e:
            self.get_logger().error(f"Navigation call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
