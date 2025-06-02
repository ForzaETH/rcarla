#!/bin/python3
import math
import threading

import carla
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from geometry_msgs.msg import Pose, Twist


from rcarla_msgs.srv import PhysicsInit, PhysicsStep, ImuUpdate, OdomUpdate
from transforms3d.euler import euler2quat


class PhysicsInterface(CompatibleNode):
    def __init__(self):
        super(PhysicsInterface, self).__init__("physics_interface")
        
        
        self.wheelbase = self.get_param('vehicle/wheelbase', 1.5)
        self.max_speed = self.get_param('vehicle/max_speed', 30.0)

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.vx = 0

        try:
            self.imu_client = self.new_client(
                ImuUpdate,
                "/rcarla/sensors_interface/imu_update",
                timeout_sec=0.001 # 1 second
            )
        except Exception as e:
            self.imu_client = None

        try:
            self.odom_client = self.new_client(
                OdomUpdate,
                "/rcarla/sensors_interface/odom_update",
                timeout_sec=0.001 # 1 second
            )
        except Exception as e:
            self.odom_client = None
            self.logwarn(f"Failed to create odometry client: {e}")
        
        self.step_service = self.new_service(
            PhysicsStep,
            "/rcarla/physics_interface/step",
            self.step_callback,
        )

        self.init_service = self.new_service(
            PhysicsInit,
            "/rcarla/physics_interface/init",
            self.init_callback,
        )

        self.loginfo("Physics interface ready")

    
    def init_callback(self, req, response=None):
        self.x = req.x_init
        self.y = req.y_init
        self.yaw = req.yaw_init
        response = roscomp.get_service_response(PhysicsInit)
        response.success = True
        self.loginfo(f"Initializing physics with x: {self.x}, y: {self.y}, yaw: {self.yaw}")
        return response
    
    def step_callback(self, req, response=None):
        self.physics_step(req.dt, req.desired_acceleration, req.desired_steering)
        response = roscomp.get_service_response(PhysicsStep)
        response.x = self.x
        response.y = self.y
        response.yaw = self.yaw
        return response


    def update_odometry(self, x, y, yaw, vx, yaw_rate):
        if self.odom_client is None:
            return

        req = roscomp.get_service_request(OdomUpdate)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        orientation = euler2quat(0.0, 0.0, yaw)
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = yaw_rate

        req.pose = pose
        req.twist = twist        
        
        try:
            thread = threading.Thread(target=self._call_service, args=(self.odom_client, req))
            thread.start()
        except roscomp.exceptions.ServiceException as e:
            self.logwarn(f"Failed to update odometry: {e}") 


    def update_imu(self, ax, ay, yaw_rate, yaw):
        if self.imu_client is None:
            return
        
        orientation = euler2quat(0.0, 0.0, yaw)
        req = roscomp.get_service_request(ImuUpdate)
        req.orientation.x = orientation[0]
        req.orientation.y = orientation[1]
        req.orientation.z = orientation[2]
        req.orientation.w = orientation[3]
        req.ax = ax
        req.ay = ay
        req.yaw_rate = yaw_rate
        
        try:
            thread = threading.Thread(target=self._call_service, args=(self.imu_client, req))
            thread.start()
        except roscomp.ServiceException as e:
            self.logwarn(f"Failed to update IMU: {e}")

    def _call_service(self, client, req):
        resp = self.call_service(client, req, spin_until_response_received=False)
        return resp

    def physics_step(self, dt, desired_acceleration, desired_steering):
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt
        old_vx = self.vx
        if desired_acceleration == 0.0:
            self.vx *= 0.999 # fake friction
        else:
            self.vx += desired_acceleration * dt
        self.vx = max(0.0, self.vx)
        self.vx = min(self.max_speed, self.vx)
        yaw_rate = self.vx / self.wheelbase * math.tan(desired_steering)
        self.yaw += yaw_rate * dt
        if self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        elif self.yaw < -math.pi:
            self.yaw += 2 * math.pi
        self.update_imu(
            ax=(self.vx - old_vx) / dt,
            ay=0.0,
            yaw_rate=yaw_rate,
            yaw=self.yaw
        )

        self.update_odometry(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            vx=self.vx,
            yaw_rate=yaw_rate
        )


    def destroy(self):
        self.destroy_service(self.init_service)
        self.destroy_service(self.step_service)

        
def main():
    roscomp.init("physics_interface", args=None)
    pi = PhysicsInterface()
    executor = roscomp.executors.MultiThreadedExecutor()
    executor.add_node(pi)
    roscomp.on_shutdown(pi.destroy)
    pi.spin()


if __name__ == '__main__':
    main()

