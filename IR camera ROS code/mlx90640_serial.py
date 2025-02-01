import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

class MLX90640Publisher(Node):
    def __init__(self):
        super().__init__('mlx90640_publisher')
        self.ser = serial.Serial('/dev/ttyACM0', 2000000, timeout=0.01)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'thermal_image', 10)
        self.buffer = bytearray()

    def timer_callback(self):
        try:
            data = self.ser.read(3072)  # 768 floats × 4 bytes
            if data:
                self.buffer.extend(data)
                while len(self.buffer) >= 3072:
                    chunk = self.buffer[:3072]
                    self.buffer = self.buffer[3072:]
                    arr = np.frombuffer(chunk, dtype=np.float32)
                    msg = Float32MultiArray(data=arr.tolist())
                    self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = MLX90640Publisher()
    node.create_timer(0.001, node.timer_callback)  # 1ms poll
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()