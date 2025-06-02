#!/bin/python3
import math

import carla
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from transforms3d.euler import quat2euler, euler2quat
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

from rcarla_msgs.srv import PhysicsInit, PhysicsStep


class DriveInterface(CompatibleNode):
    def __init__(self):
        super(DriveInterface, self).__init__("drive_interface")
        self.follow_cam = self.get_param('cam/use_follow_cam', True)
        self.delta_viewing_angle = self.get_param('cam/delta_viewing_angle', 0.0)
        self.viewing_distance = self.get_param('cam/viewing_distance', 10.0)
        self.viewing_height = self.get_param('cam/viewing_height', 5.0)

        x = self.get_param('physics/x_init', 0.0)
        y = self.get_param('physics/y_init', 0.0)
        yaw = self.get_param('physics/yaw_init', 0.0)

        self.transform = carla.Transform(
            carla.Location(x, -y, 0.0),  # CARLA uses a different coordinate system
            carla.Rotation(
                pitch=0.0,
                roll=0.0,
                yaw=-yaw  # CARLA uses a different coordinate system
            )
        )
        self.new_transform = False
        self.ego_vehicle = None
        self.initialized = False
                
        self.desired_acceleration = 0.0
        self.desired_steering = 0.0

        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        self.world = client.get_world()
        self.spectacor = self.world.get_spectator()
        self.get_ego_vehicle()

        self.joy_sub = self.new_subscription(
            Joy,
            "/joy",
            self.joy_cb,
            10
        )   
    
        self.init_client = self.new_client(
            PhysicsInit,
            "/rcarla/physics_interface/init",
            timeout_sec=10.0
        )


        self.step_client = self.new_client(
            PhysicsStep,
            "/rcarla/physics_interface/step",
            timeout_sec=10.0
        )

        self.pose_pub = self.new_publisher(
            PoseStamped,
            "/rcarla/ego_vehicle/pose",
            10
        )
        self.cb_id = self.world.on_tick(self.run)


    def update_spectactor(self):
        if not self.follow_cam or self.ego_vehicle is None or self.spectacor is None:
            return

        transform = self.ego_vehicle.get_transform()
        viewing_angle = math.radians(transform.rotation.yaw + self.delta_viewing_angle)
        transform.location.z += self.viewing_height
        transform.location.x -= math.cos(viewing_angle) * self.viewing_distance
        transform.location.y -= math.sin(viewing_angle) * self.viewing_distance
        transform.rotation.pitch = min(0,134 / math.pow(self.viewing_height, 0.5) - 90)
        transform.rotation.yaw = math.degrees(viewing_angle)
        self.spectacor.set_transform(transform)


    def get_ego_vehicle(self):
        vehicle_list = self.world.get_actors()
        vehicle_list = [x for x in vehicle_list if 'vehicle' in x.type_id and not 'npc' in x.type_id]
        if len(vehicle_list) == 1:
            self.ego_vehicle = vehicle_list[0]
            self.ego_vehicle.set_simulate_physics(False)
            self.loginfo(f"Found ego vehicle and disabled CARLA physics: {self.ego_vehicle.type_id}")
            return True
        else:
            self.logerr("No ego vehicle found. Please ensure a vehicle is spawned in the CARLA world.")
            self.world.remove_on_tick(self.cb_id)


    def initialize_physics(self):
        init_request = roscomp.get_service_request(PhysicsInit)
        init_request.x_init = self.transform.location.x
        init_request.y_init = -self.transform.location.y 
        yaw = -math.degrees(self.transform.rotation.yaw)
        self.init_client.call(init_request)
        self.initialized = True


    def run(self, snapshot):
        if not self.initialized:
            self.initialize_physics()
            return
        request = roscomp.get_service_request(PhysicsStep)
        request.dt  = float(snapshot.timestamp.delta_seconds)
        request.desired_acceleration = float(self.desired_acceleration)
        request.desired_steering = float(self.desired_steering)
        resp = self.step_client.call(request)
        if resp is None:
            self.logwarn("Failed to call physics step service. Skipping physics step.")
            return
        self.transform = carla.Transform(
            carla.Location(resp.x, -resp.y, 0.0),  # CARLA uses a different coordinate system
            carla.Rotation(
                pitch=0.0,
                roll=0.0,
                yaw=-math.degrees(resp.yaw)  # CARLA uses a different coordinate system
            )
        )
        self.ego_vehicle.set_transform(self.transform)
        self.update_spectactor()
        self.publish_pose(roscomp.ros_timestamp(snapshot.timestamp.elapsed_seconds, from_sec=True))

    def publish_pose(self, time):
        msg = PoseStamped()
        msg.header.stamp = time
        msg.header.frame_id = "map"
        msg.pose.position.x = self.transform.location.x
        msg.pose.position.y = -self.transform.location.y 
        msg.pose.position.z = self.transform.location.z
        quat = euler2quat(
            math.radians(self.transform.rotation.roll),
            math.radians(self.transform.rotation.pitch),
            - math.radians(self.transform.rotation.yaw)
        )
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        if self.pose_pub is not None:
            self.pose_pub.publish(msg)
       
    def joy_cb(self, msg):
        if msg.axes[5] == 0.0 and msg.axes[2] == 0.0:
            return

        if msg.buttons[0] == 1:
            self.desired_acceleration = ((1 - msg.axes[5]) + (msg.axes[2] - 1))  * 2.0 # R2 and L2
            self.desired_acceleration = max(-4.0, min(4.0, self.desired_acceleration))
            # if msg.buttons[4] == 1: # L1
            #     self.desired_acceleration *= -1
            self.desired_steering = msg.axes[0] * 0.3 # Left stick horizontal
            self.desired_steering = max(-1.0, min(1.0, self.desired_steering))
        else:
            self.desired_acceleration = 0.0
            self.desired_steering = 0.0
    
    def destroy(self):
        self.world.remove_on_tick(self.cb_id)
        

def main():
    roscomp.init("drive_interface", args=None)
    executor = roscomp.executors.SingleThreadedExecutor()
    di = DriveInterface()
    executor.add_node(di)
    roscomp.on_shutdown(di.destroy)
    di.spin()


if __name__ == '__main__':
    main()

