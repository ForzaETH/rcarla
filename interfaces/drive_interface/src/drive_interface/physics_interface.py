#!/bin/python3
import math
import threading

import carla
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from geometry_msgs.msg import Pose, Twist


from rcarla_msgs.srv import PhysicsInit, PhysicsStep, ImuUpdate, OdomUpdate
from transforms3d.euler import euler2quat

ROS_VERSION = roscomp.get_ros_version()
if ROS_VERSION == 1:
    from dynamics_model import vehicle_dynamics_st_update_pacejka
elif ROS_VERSION == 2:
    from drive_interface.dynamics_model import vehicle_dynamics_st_update_pacejka


class PhysicsInterface(CompatibleNode):
    def __init__(self):
        super(PhysicsInterface, self).__init__("physics_interface")
        
        
        self.wheelbase = self.get_param('vehicle/wheelbase', 1.5)
        self.max_speed = self.get_param('vehicle/max_speed', 30.0)
        self.max_sv = self.get_param('vehicle/max_steering_velocity', 1.0)


        self.state = {
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,
            "velocity": 0.0,
            "steer_angle": 0.0,
            "angular_velocity": 0.0,
            "slip_angle": 0.0
        }

        self.params = {
            "Bf": self.get_param("tire_params/Bf", 10.0),
            "Br": self.get_param("tire_params/Br", 10.0),
            "Cf": self.get_param("tire_params/Cf", 10.0),
            "Cr": self.get_param("tire_params/Cr", 10.0),
            "Df": self.get_param("tire_params/Df", 10.0),
            "Dr": self.get_param("tire_params/Dr", 10.0),
            "Ef": self.get_param("tire_params/Ef", 10.0),
            "Er": self.get_param("tire_params/Er", 10.0),
            "mu":  self.get_param("tire_params/mu", 1.0),
            "l_f": self.get_param("vehicle/cog_to_front", 1.0),
            "l_r": self.get_param("vehicle/cog_to_rear", 1.0),
            "wheelbase": self.wheelbase,
            "h_cg": self.get_param("vehicle/cog_height", 0.5),
            "m": self.get_param("vehicle/mass", 1000.0),
            "I_z": self.get_param("vehicle/moment_of_inertia", 1000.0) 
        }

        try:
            self.imu_client = self.new_client(
                ImuUpdate,
                "/rcarla/sensors_interface/imu_update",
                timeout_sec=0.001 # 1 second
            )
        except Exception as e:
            self.imu_client = None
            self.logwarn(f"Failed to create imu client: {e}")


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
        self.state["x"] = req.x_init
        self.state["y"] = req.y_init
        self.state["theta"] = req.yaw_init
        response = roscomp.get_service_response(PhysicsInit)
        response.success = True
        self.loginfo(f"Initializing physics with x: {req.x_init}, y: {req.y_init}, yaw: {req.yaw_init}")
        return response
    
    def step_callback(self, req, response=None):  
        self.state, [ax, ay] = vehicle_dynamics_st_update_pacejka(
            self.state,
            req.desired_acceleration,
            self.get_sv(req.desired_steering),
            self.params,
            req.dt
        )        
        
        self.update_imu(ax=ax, ay=ay)
        self.update_odometry()
        
        response = roscomp.get_service_response(PhysicsStep)
        response.x = self.state["x"]
        response.y = self.state["y"]
        response.yaw = self.state["theta"]
        return response


    def get_sv(self, desired_steering):
        steer_diff = desired_steering - self.state["steer_angle"]
        if abs(steer_diff) < 1e-3:
            return 0.0
        else:
            return self.max_sv if steer_diff > 0 else -self.max_sv 

    def update_odometry(self):
        if self.odom_client is None:
            return

        req = roscomp.get_service_request(OdomUpdate)
        pose = Pose()
        pose.position.x = self.state["x"]
        pose.position.y = self.state["y"]
        pose.position.z = 0.0
        orientation = euler2quat(0.0, 0.0, self.state["theta"])
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        twist = Twist()
        twist.linear.x = self.state["velocity"] * math.cos(self.state["slip_angle"])
        twist.linear.y = self.state["velocity"] * math.sin(self.state["slip_angle"])
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.state["angular_velocity"]

        req.pose = pose
        req.twist = twist        
        
        try:
            thread = threading.Thread(target=self._call_service, args=(self.odom_client, req))
            thread.start()
        except roscomp.exceptions.ServiceException as e:
            self.logwarn(f"Failed to update odometry: {e}") 

    def update_imu(self, ax, ay):
        if self.imu_client is None:
            return
        
        orientation = euler2quat(0.0, 0.0, self.state["theta"])
        req = roscomp.get_service_request(ImuUpdate)
        req.orientation.x = orientation[0]
        req.orientation.y = orientation[1]
        req.orientation.z = orientation[2]
        req.orientation.w = orientation[3]
        req.ax = ax
        req.ay = ay
        req.yaw_rate = self.state["angular_velocity"]
        
        try:
            thread = threading.Thread(target=self._call_service, args=(self.imu_client, req))
            thread.start()
        except roscomp.ServiceException as e:
            self.logwarn(f"Failed to update IMU: {e}")

    def _call_service(self, client, req):
        resp = self.call_service(client, req, spin_until_response_received=False)
        return resp


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

