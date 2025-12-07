#!/bin/bash
# Test script for object detection node
# Downloads a real test image and publishes it to simulate camera feed

echo "=========================================="
echo "Object Detection Test"
echo "=========================================="
echo "This script will:"
echo "1. Download a real test image (if needed)"
echo "2. Publish it as a camera feed"
echo "3. Monitor for YOLO detections"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Create a temporary Python script to publish test images
python3 << 'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import sys
import os
from pathlib import Path

class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        self.publisher = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.detection_sub = self.create_subscription(String, 'turtlebot3/perception/labels', self.detection_callback, 10)
        self.timer = self.create_timer(2.0, self.publish_image)  # 0.5 Hz - slower to see detections
        self.get_logger().info('='*60)
        self.get_logger().info('Camera simulator started')
        self.get_logger().info('Publishing to: /camera/color/image_raw')
        self.get_logger().info('Monitoring: turtlebot3/perception/labels')
        self.get_logger().info('='*60)
        self.frame_count = 0
        self.detection_count = 0
        self.last_detection = "None yet"
        
        # Try to download a real test image
        self.test_image = self.get_test_image()
        
    def detection_callback(self, msg):
        """Monitor detections from YOLO"""
        self.detection_count += 1
        self.last_detection = msg.data
        self.get_logger().info(f'🎯 DETECTION #{self.detection_count}: {msg.data}', throttle_duration_sec=1.0)
    
    def get_test_image(self):
        """Try to load or create a test image with actual objects"""
        try:
            import urllib.request
            import cv2
            
            test_img_path = Path.home() / '.cache' / 'test_image.jpg'
            test_img_path.parent.mkdir(parents=True, exist_ok=True)
            
            if not test_img_path.exists():
                self.get_logger().info('Downloading test image with real objects...')
                # Download a sample image from Ultralytics (people in a street)
                url = 'https://ultralytics.com/images/bus.jpg'
                urllib.request.urlretrieve(url, str(test_img_path))
                self.get_logger().info(f'Test image saved to: {test_img_path}')
            
            img = cv2.imread(str(test_img_path))
            if img is not None:
                # Resize to 640x480
                img = cv2.resize(img, (640, 480))
                self.get_logger().info('✓ Using real test image (contains: bus, people, vehicles)')
                return img
        except Exception as e:
            self.get_logger().warn(f'Could not load real image: {e}. Using synthetic patterns.')
        
        return None
        
    def publish_image(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height = 480
        msg.width = 640
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = 640 * 3
        
        if self.test_image is not None:
            # Use the real test image
            img = self.test_image.copy()
            # Add some variation by adjusting brightness
            brightness = np.random.randint(-30, 30)
            img = np.clip(img.astype(np.int16) + brightness, 0, 255).astype(np.uint8)
        else:
            # Fallback: Create synthetic pattern
            img = self.create_synthetic_pattern()
        
        msg.data = img.tobytes()
        self.publisher.publish(msg)
        self.frame_count += 1
        
        # Status update every 5 frames
        if self.frame_count % 5 == 0:
            self.get_logger().info(
                f'Published {self.frame_count} frames | '
                f'Detections: {self.detection_count} | '
                f'Last: {self.last_detection}'
            )
    
    def create_synthetic_pattern(self):
        """Fallback: Create synthetic test pattern"""
        img = np.ones((480, 640, 3), dtype=np.uint8) * 128
        
        # Create structured objects with realistic colors
        # Person-like figure (vertical with head)
        img[100:150, 300:340] = [210, 180, 140]  # skin tone head
        img[150:300, 280:360] = [50, 50, 150]    # blue shirt
        img[300:400, 280:320] = [40, 40, 80]     # dark pants left leg
        img[300:400, 320:360] = [40, 40, 80]     # dark pants right leg
        
        # Car-like shape
        img[250:350, 100:250] = [180, 50, 50]    # red car body
        img[310:330, 115:145] = [20, 20, 20]     # black wheel
        img[310:330, 205:235] = [20, 20, 20]     # black wheel
        
        # Add noise for realism
        noise = np.random.randint(-15, 15, img.shape, dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        return img

def main():
    rclpy.init()
    node = CameraSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('')
        node.get_logger().info('='*60)
        node.get_logger().info(f'Camera simulator stopped')
        node.get_logger().info(f'Total frames published: {node.frame_count}')
        node.get_logger().info(f'Total detections: {node.detection_count}')
        node.get_logger().info('='*60)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
