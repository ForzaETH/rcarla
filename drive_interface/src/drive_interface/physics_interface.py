#!/bin/python3
import math

import carla
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from transforms3d.euler import euler2quat




class PhysicsInterface(CompatibleNode):
    def __init__(self):
        super(PhysicsInterface, self).__init__("physics_interface")
        
        self.x = self.get_param('x_init', 0.0)
        self.y = self.get_param('y_init', 0.0)
        self.vx = self.get_param('vx_init', 0.0)
        self.yaw = self.get_param('yaw_init', 0.0)
        self.wheelbase = 1.5
                
        self.desired_acceleration = None
        self.desired_steering = None
        
        self.last_joy_msg = self.get_time()


        self.pose_pub = self.new_publisher(
            PoseStamped,
            "/rcarla/drive_interface/pose",
            10
        )

        self.drive_sub = self.new_subscription(
            AckermannDriveStamped,
            "/rcarla/drive_interface/drive",
            self.drive_callback,
            10
        )

        self.joy_sub = self.new_subscription(
            Joy,
            "/joy",
            self.joy_cb,
            10
        )   

        self.world = carla.Client('localhost', 2000).get_world()
        self.on_tick_id = self.world.on_tick(self.run)

    def drive_callback(self, msg):
        if self.get_time() - self.last_joy_msg < 0.5:
            self.logwarn("This hy")
            return
        self.desired_acceleration = msg.acceleration
        self.desired_steering = msg.steering
        
    def joy_cb(self, msg):
        if msg.axes[5] == 0.0 and msg.axes[2] == 0.0:
            return

        if msg.buttons[0] == 1:
            self.desired_acceleration = (msg.axes[2] - 1) * 2.0
            self.desired_acceleration = (msg.axes[5] - 1) * -2.0
            self.desired_acceleration = max(-2.0, min(2.0, self.desired_acceleration))
            self.desired_steering = msg.axes[0]
            self.desired_steering = max(-1.0, min(1.0, self.desired_steering))
            self.last_joy_msg = self.get_time()

    def run(self, snapshot):
        if self.desired_acceleration == None or self.desired_steering == None:
            return

        dt = snapshot.timestamp.delta_seconds

        #TODO: Implement real physics
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt
        self.vx *= 0.99
        self.vx += self.desired_acceleration * dt
        self.yaw += self.vx / self.wheelbase * math.tan(self.desired_steering) * dt
        if self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        elif self.yaw < -math.pi:
            self.yaw += 2 * math.pi
        self.loginfo(f"x: {self.x:.2f}, y: {self.y:.2f}, vx: {self.vx:.2f}, yaw: {self.yaw:.2f}")
        self.publish_pose()


    def publish_pose(self):
        quat = euler2quat(0.0, 0.0, self.yaw * 180 / math.pi , axes='sxyz')
        pose_msg = PoseStamped()
        pose_msg.header.stamp = roscomp.ros_timestamp(sec=self.get_time(), from_sec=True)
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = quat[0]
        pose_msg.pose.orientation.x = quat[1]
        pose_msg.pose.orientation.y = quat[2]
        pose_msg.pose.orientation.z = quat[3]
        self.pose_pub.publish(pose_msg)


    def destroy(self):
        self.world.remove_on_tick(self.on_tick_id)

        

def main():
    roscomp.init("physics_interface", args=None)
    executor = roscomp.executors.SingleThreadedExecutor()
    pi = PhysicsInterface()
    executor.add_node(pi)
    roscomp.on_shutdown(pi.destroy)
    pi.spin()


if __name__ == '__main__':
    main()

