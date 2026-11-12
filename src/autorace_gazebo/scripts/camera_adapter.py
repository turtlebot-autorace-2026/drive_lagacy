#!/usr/bin/env python3
"""Expose simulated JPEGs on the bag's /camera and /camera2 interfaces.

No image transform or driving logic runs here. CameraInfo uses the same
intrinsics as Gazebo's custom projection (the stock plugin assumes fx == fy).
"""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage


class CameraAdapter(Node):
    def __init__(self):
        super().__init__('sim_camera_adapter')
        default = str(Path(get_package_share_directory('autorace_gazebo')) / 'config/camera_rig.json')
        path = self.declare_parameter('rig_config', default).value
        rig = json.loads(Path(path).read_text())
        self.subscriptions_ = []
        for c in rig['cameras']:
            name = c['name']
            image_pub = self.create_publisher(CompressedImage, '/' + name, 5)
            info_pub = self.create_publisher(CameraInfo, '/' + name + '/camera_info', 5)
            info = CameraInfo()
            info.width, info.height = c['width'], c['height']
            info.distortion_model = 'plumb_bob'
            info.d = [0.0] * 5
            fx, fy, cx, cy = (float(c[k]) for k in ('fx', 'fy', 'cx', 'cy'))
            info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]

            def callback(msg, image_pub=image_pub, info_pub=info_pub, info=info):
                info.header = msg.header
                image_pub.publish(msg)
                info_pub.publish(info)

            self.subscriptions_.append(self.create_subscription(
                CompressedImage, '/' + name + '/image_raw/compressed',
                callback, qos_profile_sensor_data))
            self.get_logger().info(f'{name}: {c["width"]}x{c["height"]}, fx={fx:.3f}, fy={fy:.3f}')


def main():
    rclpy.init()
    node = CameraAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
