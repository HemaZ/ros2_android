import json
import urllib.request
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from sensor_msgs.msg import Image, NavSatFix
from android_node_interfaces.srv import Torch, Zoom


class Android(Node):
    def __init__(self):
        super().__init__("android_node")
        self.declare_parameter('ip',
                               '192.168.1.122')
        self.declare_parameter('port', 8080)
        self.declare_parameter('fps', 15)
        self.declare_parameter('gps_freq', 2)
        self.ip_ = self.get_parameter(
            'ip').get_parameter_value().string_value
        self.port_ = self.get_parameter(
            'port').get_parameter_value().integer_value
        self.url_ = 'http://' + self.ip_ + ':' + str(self.port_) + '/'
        self.fps_ = self.get_parameter(
            'fps').get_parameter_value().integer_value
        self.gps_freq_ = self.get_parameter(
            'gps_freq').get_parameter_value().integer_value
        try:
            self.update_status_()
        except urllib.error.URLError:
            self.get_logger().error('Can not Connect to the url {}'.format(self.url_))
            return

        self.model_ = self.status['model'].replace('-', '_')
        self.camera_pub_ = self.create_publisher(
            Image, self.model_ + '/camera', 1)
        self.gps_pub_ = self.create_publisher(
            NavSatFix, self.model_ + '/gps', 1)
        self.cv_bridge_ = CvBridge()
        self.timer_camera = self.create_timer(
            1/self.fps_, self.timer_camera_clk_)
        self.timer_gps = self.create_timer(
            1/self.gps_freq_, self.timer_gps_clk_)
        self.torch_srv_ = self.create_service(
            Torch, '/{}/toggle_torch'.format(self.model_), self.torch_srv_clk_)
        self.zoom_srv_ = self.create_service(
            Zoom, '/{}/camera_zoom'.format(self.model_), self.zoom_srv_clk_)

    def update_status_(self):
        with urllib.request.urlopen(self.url_ + 'status') as url:
            self.status = json.loads(url.read().decode())

    def update_sensors_(self):
        with urllib.request.urlopen(self.url_ + 'sensors') as url:
            self.sensors = json.loads(url.read().decode())

    def timer_camera_clk_(self):
        try:
            resp = urllib.request.urlopen(self.url_ + 'jpeg')
            image = np.asarray(bytearray(resp.read()), dtype='uint8')
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)
            img_msg = self.cv_bridge_.cv2_to_imgmsg(image)
            img_msg.header.frame_id = self.model_
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.camera_pub_.publish(img_msg)
        except BaseException:
            self.get_logger().error('Can not Connect to the device {}'.format(self.model_))
            self.destroy_node()

    def timer_gps_clk_(self):
        self.update_status_()
        msg = NavSatFix()
        msg.latitude = self.status['location']['latitude']
        msg.longitude = self.status['location']['longitude']
        msg.position_covariance_type = 0
        msg.header.frame_id = 'gps'
        self.gps_pub_.publish(msg)

    def torch_srv_clk_(self, request, response):
        self.get_logger().info('Got torch toggling request {}'.format(request.state))
        if request.state:
            resp = urllib.request.urlopen(self.url_ + 'control?torch=on')
        else:
            resp = urllib.request.urlopen(self.url_ + 'control?torch=off')
        self.update_sensors_()
        if self.sensors['torch'] == 'false':
            response.response = False
        else:
            response.response = True
        return response

    def zoom_srv_clk_(self, request, response):
        self.get_logger().info('Got Zoom request {}'.format(request.delta))
        try:
            if int(request.delta) > 0:
                _ = urllib.request.urlopen(
                    self.url_ + 'control?zoom=+{}'.format(int(request.delta)))
                print('control?zoom=+{}'.format(int(request.delta)))
            else:
                _ = urllib.request.urlopen(
                    self.url_ + 'control?zoom={}'.format(int(request.delta)))
                print('control?zoom={}'.format(int(request.delta)))
            status = True
        except BaseException:
            status = False
        response.response = status
        return response


def main(args=None):
    rclpy.init(args=args)
    device = Android()
    rclpy.spin(device)
    device.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
