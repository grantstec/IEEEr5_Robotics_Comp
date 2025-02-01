#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray

class ThermalVisualizer(Node):
    def __init__(self):
        super().__init__('thermal_visualizer')
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'thermal_image',
            self.image_callback,
            10  # Keep small queue to process latest frame
        )
        
        # Pre-allocate arrays
        self.thermal_array = np.zeros((24, 32), dtype=np.float32)
        self.last_display = 0

    def image_callback(self, msg):
        try:
            # Fast reshape without copy
            self.thermal_array = np.asarray(msg.data, dtype=np.float32).reshape(24, 32)
            
            # Throttle display to 16Hz
            now = self.get_clock().now().nanoseconds
            if now - self.last_display < 62_500_000:  # 1/16 second in nanoseconds
                return
            self.last_display = now
            
            # Normalize and display
            normalized = cv2.normalize(self.thermal_array, None, 0, 255, cv2.NORM_MINMAX)
            colored = cv2.applyColorMap(np.uint8(normalized), cv2.COLORMAP_INFERNO)
            resized = cv2.resize(colored, (640, 480), interpolation=cv2.INTER_NEAREST)
            
            cv2.imshow('Thermal Image', resized)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ThermalVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()