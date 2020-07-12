import rclpy
from rclpy.node import Node
import urllib.request
import json
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
import numpy as np
import cv2
from android_node_interfaces.srv import Torch, Zoom


class Android(Node):
    def __init__(self, ip, port=8080):
        self.ip_ = ip
        self.port_ = port
        self.url_ = "http://"+ip+":"+str(port)+"/"
        self.update_status_()
        self.model_ = self.status['model'].replace("-", "_")
        super().__init__(self.model_)
        self.camera_pub_ = self.create_publisher(
            Image, self.model_+"/camera", 1)
        self.gps_pub_ = self.create_publisher(
            NavSatFix, self.model_+"/gps", 1)
        self.cv_bridge_ = CvBridge()
        self.timer_camera = self.create_timer(1/15, self.timer_camera_clk_)
        self.timer_gps = self.create_timer(0.5, self.timer_gps_clk_)
        self.torch_srv_ = self.create_service(
            Torch, "/{}/toggle_torch".format(self.model_), self.torch_srv_clk_)
        self.zoom_srv_ = self.create_service(
            Zoom, "/{}/camera_zoom".format(self.model_), self.zoom_srv_clk_)

    def update_status_(self):
        with urllib.request.urlopen(self.url_+"status") as url:
            self.status = json.loads(url.read().decode())

    def update_sensors_(self):
        with urllib.request.urlopen(self.url_+"sensors") as url:
            self.sensors = json.loads(url.read().decode())

    def timer_camera_clk_(self):
        try:
            resp = urllib.request.urlopen(self.url_+"jpeg")
            image = np.asarray(bytearray(resp.read()), dtype="uint8")
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)
            img_msg = self.cv_bridge_.cv2_to_imgmsg(image)
            img_msg.header.frame_id = self.model_
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.camera_pub_.publish(img_msg)
        except:
            self.get_logger().error("Can't Connect to the device {}".format(self.model_))
            self.destroy_node()

    def timer_gps_clk_(self):
        self.update_status_()
        msg = NavSatFix()
        msg.latitude = self.status['location']['latitude']
        msg.longitude = self.status['location']['longitude']
        msg.position_covariance_type = 0
        msg.header.frame_id = "gps"
        self.gps_pub_.publish(msg)

    def torch_srv_clk_(self, request, response):
        self.get_logger().info("Got torch toggling request {}".format(request.state))
        if request.state:
            resp = urllib.request.urlopen(self.url_+"control?torch=on")
        else:
            resp = urllib.request.urlopen(self.url_+"control?torch=off")
        self.update_sensors_()
        if self.sensors['torch'] == "false":
            response.response = False
        else:
            response.response = True
        return response

    def zoom_srv_clk_(self, request, response):
        self.get_logger().info("Got Zoom request {}".format(request.delta))
        try:
            if int(request.delta) > 0:
                resp = urllib.request.urlopen(
                    self.url_+"control?zoom=+{}".format(int(request.delta)))
                print("control?zoom=+{}".format(int(request.delta)))
            else:
                resp = urllib.request.urlopen(
                    self.url_+"control?zoom={}".format(int(request.delta)))
                print("control?zoom={}".format(int(request.delta)))
            status = True
        except:
            status = False
        response.response = status
        return response


def main(args=None):
    rclpy.init(args=args)
    device = Android("192.168.1.122")
    rclpy.spin(device)
    device.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
