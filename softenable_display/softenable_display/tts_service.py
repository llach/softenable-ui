import os
import time
import rclpy
from rclpy.node import Node

from softenable_display_msgs.srv import TTS
import sounddevice as sd
from piper import PiperVoice


class TTSService(Node):
    def __init__(self):
        super().__init__('tts_service')

        # Declare and read model path parameter
        default_model_path = f"{os.environ['HOME']}/en_US-lessac-high.onnx"
        self.declare_parameter('model_path', default_model_path)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        self.get_logger().info(f"Loading Piper voice model from: {model_path}")
        try:
            self.voice = PiperVoice.load(model_path)
            self.get_logger().info("Voice model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load voice model: {e}")
            raise

        # Create service
        self.srv = self.create_service(TTS, 'tts', self.handle_tts_request)
        self.get_logger().info("TTS service ready at /tts")

    def handle_tts_request(self, request, response):
        text = request.text.strip()
        if not text:
            self.get_logger().warn("Received empty text.")
            return response

        self.get_logger().info(f"Synthesizing: \"{text}\"")

        start_time = time.time()
        try:
            chunks = self.voice.synthesize(text)
            duration = time.time() - start_time
            self.get_logger().info(f"Synthesis took {duration:.3f}s")

            for chunk in chunks:
                sd.play(chunk.audio_float_array, samplerate=chunk.sample_rate)
                sd.wait()
            
            self.get_logger().info(f"Done talking.")

        except Exception as e:
            self.get_logger().error(f"Error during synthesis: {e}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TTSService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
