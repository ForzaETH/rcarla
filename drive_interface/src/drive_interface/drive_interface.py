#!/bin/python3
import math

import carla
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from transforms3d.euler import quat2euler
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy



class DriveInterface(CompatibleNode):
    def __init__(self):
        super(DriveInterface, self).__init__("drive_interface")
        self.follow_cam = self.get_param('follow_cam', True)
        self.delta_viewing_angle = self.get_param('delta_viewing_angle', 0.0)
        self.viewing_distance = self.get_param('viewing_distance', 10.0)
        self.viewing_height = self.get_param('viewing_height', 5.0)

        x = self.get_param('x_init', 0.0)
        y = self.get_param('y_init', 0.0)
        self.vx = self.get_param('vx_init', 0.0)
        yaw = self.get_param('yaw_init', 0.0)
        self.wheelbase = 1.5

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
                
        self.desired_acceleration = None
        self.desired_steering = None

        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        self.world = client.get_world()

        self.spectacor = self.world.get_spectator()

        self.joy_sub = self.new_subscription(
            Joy,
            "/joy",
            self.joy_cb,
            10
        )   
        
        self.pose_sub = self.new_subscription(
            PoseStamped,
            "/rcarla/drive_interface/pose",
            self.pose_callback,
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
            self.logwarn("No ego vehicle found. Please ensure a vehicle is spawned in the CARLA world.")
            return False
        
    def run(self, carla_snapshot):
        if self.ego_vehicle is None:
            if not self.get_ego_vehicle():
                self.logwarn("Ego vehicle not found. Skipping physics step.")
                return

        self.update_spectactor()
        self.physics_step(carla_snapshot.timestamp.delta_seconds)

        if self.new_transform:
            self.new_transform = False
            self.ego_vehicle.set_transform(self.transform)
            self.update_spectactor()

    def physics_step(self, dt):
        if self.desired_acceleration == None or self.desired_steering == None:
            return
        
        x = self.transform.location.x
        y = -self.transform.location.y  # CARLA uses a different coordinate system
        yaw = math.radians(-self.transform.rotation.yaw)  # CARLA uses a different coordinate system

        x += self.vx * math.cos(yaw) * dt
        y += self.vx * math.sin(yaw) * dt
        self.vx *= 0.999
        self.vx += self.desired_acceleration * dt
        yaw += self.vx / self.wheelbase * math.tan(self.desired_steering) * dt
        if yaw > math.pi:
            yaw -= 2 * math.pi
        elif yaw < -math.pi:
            yaw += 2 * math.pi

        self.transform = carla.Transform(
            carla.Location(x, -y, 0.0),
            carla.Rotation(
                pitch=0.0,
                roll=0.0,
                yaw= - math.degrees(yaw)
            )
        )
        self.new_transform = True
        
    
    def joy_cb(self, msg):
        if msg.axes[5] == 0.0 and msg.axes[2] == 0.0:
            return

        if msg.buttons[0] == 1:
            self.desired_acceleration = ((1 - msg.axes[5]) + (msg.axes[2] - 1))  * 2.0 # R2 and L2
            self.desired_acceleration = max(0.0, min(4.0, self.desired_acceleration))
            if self.buttons[4] == 1: # L1
                self.desired_acceleration *= -1
            self.desired_steering = msg.axes[0] * 0.3 # Left stick horizontal
            self.desired_steering = max(-1.0, min(1.0, self.desired_steering))
            self.last_joy_msg = self.get_time()
        else:
            self.desired_acceleration = 0.0
            self.desired_steering = 0.0
    
    def pose_callback(self, msg):
        quaternion = (
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        )
        roll, pitch, yaw = quat2euler(quaternion, 'sxyz')
        yaw = math.degrees(yaw)
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)

        self.loginfo(f"Setting yaw to {-yaw}")
        self.transform = carla.Transform(
            carla.Location(msg.pose.position.x, -msg.pose.position.y, msg.pose.position.z),
            carla.Rotation(
                pitch=pitch,
                roll=roll,
                yaw=-yaw
            )
        )
        self.new_transform = True

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

