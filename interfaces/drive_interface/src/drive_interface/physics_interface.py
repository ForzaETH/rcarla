#!/bin/python3
import math

import carla
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from rcarla_msgs.srv import PhysicsInit, PhysicsStep
from transforms3d.euler import euler2quat




class PhysicsInterface(CompatibleNode):
    def __init__(self):
        super(PhysicsInterface, self).__init__("physics_interface")
        
        
        self.wheelbase = self.get_param('vehicle/wheelbase', 1.5)
        self.max_speed = self.get_param('vehicle/max_speed', 30.0)  # m/s

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

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.vx = 0
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

    def physics_step(self, dt, desired_acceleration, desired_steering):
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt
        if desired_acceleration == 0.0:
            self.vx *= 0.999 # fake friction
        else:
            self.vx += desired_acceleration * dt
        self.vx = max(0.0, self.vx)
        self.vx = min(self.max_speed, self.vx)
        self.yaw += self.vx / self.wheelbase * math.tan(desired_steering) * dt
        if self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        elif self.yaw < -math.pi:
            self.yaw += 2 * math.pi


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

