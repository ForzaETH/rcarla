#!/bin/python3
import random
import math
import json

import carla
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from geometry_msgs.msg import PoseStamped


class PositionController:
    def __init__(self, id, wp, vehicle, world, logwarn=print, loginfo=print, pose_publisher=None):
        self.callback_id = None
        self.logwarn = logwarn
        self.loginfo = loginfo
        self.pose_publisher = pose_publisher
        self.id = id
        self.world = world
        self.vehicle = vehicle
        self.waypoints = wp
        self.vel_scaler = random.uniform(0.3, 0.7)
        self.waypoint_index = random.randint(0, len(self.waypoints) - 1)
        
        last_waypoint_index = self.waypoint_index - 1 if self.waypoint_index > 0 else len(self.waypoints) - 1
        self.current_x = self.waypoints[last_waypoint_index]["x"]
        self.current_y = self.waypoints[last_waypoint_index]["y"]
        self.current_yaw = self.waypoints[last_waypoint_index]["psi"] * 180 / math.pi

        rounded_x = round(self.current_x, 2)
        rounded_y = round(self.current_y, 2)
        rounded_scale = round(self.vel_scaler, 2)
        self.loginfo(f"NPC spawned at {rounded_x}, {rounded_y} and velocity scaler {rounded_scale}")
        self.loginfo(f"NPC spawned at {rounded_x}, {rounded_y} and velocity scaler {rounded_scale}")
    
    def set_callback_id(self, callback_id):
        if self.callback_id is not None:
            self.logwarn(f"Callback ID already set to {self.callback_id}, cannot set to {callback_id}")
            return
        self.callback_id = callback_id
        self.loginfo(f"Callback ID set to {self.callback_id} for vehicle {self.id}")

    def remove_cb(self):
        if self.callback_id is None:
            self.logwarn(f"Callback ID is None, cannot remove callback for vehicle {self.id}")
            return
        if self.world is None:
            self.logwarn(f"World is None, cannot remove callback for vehicle {self.id}")
            return
        self.world.remove_on_tick(self.callback_id)

    def publish_pose(self, time_stamp):
        if self.pose_publisher is None:
            return
        msg = PoseStamped()
        msg.header.stamp = time_stamp
        msg.header.frame_id = "map"
        msg.pose.position.x = self.current_x
        msg.pose.position.y = self.current_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(math.radians(self.current_yaw / 2))
        msg.pose.orientation.w = math.cos(math.radians(self.current_yaw / 2))
        self.pose_publisher.publish(msg)
        
    
    def update_pose(self, carla_snapshot):
        dt = carla_snapshot.timestamp.delta_seconds
        wp_in_range =self.waypoint_index < len(self.waypoints)

        if not wp_in_range or not self.vehicle.is_alive:
            self.logwarn(f"Vehicle {self.id} is not alive or waypoint index {self.waypoint_index} is out of bounds ({len(self.waypoints)})")
            self.remove_cb()
            return

        waypoint = self.waypoints[self.waypoint_index]
        distance = waypoint["vx"] * dt * self.vel_scaler
        direction_x = waypoint["x"] - self.current_x
        direction_y = waypoint["y"] - self.current_y
        direction_norm = math.sqrt(direction_x ** 2 + direction_y ** 2)
        if direction_norm > 0:
            direction_x /= direction_norm
            direction_y /= direction_norm
            self.current_x += direction_x * distance
            self.current_y += direction_y * distance
            self.current_yaw = waypoint["psi"] * 180 / math.pi
        else:
            self.logwarn(f"Zero direction norm for waypoint index {self.waypoint_index} with coordinates ({waypoint['x_m']}, {waypoint['y_m']})")
            self.remove_cb()

        self.vehicle.set_transform(carla.Transform(carla.Location(self.current_x, -self.current_y, 0.0), carla.Rotation(0, -self.current_yaw, 0))) 
        self.publish_pose(roscomp.ros_timestamp(carla_snapshot.timestamp.elapsed_seconds, from_sec=True))
            
        if direction_norm <= distance:
            if self.waypoint_index == len(self.waypoints) - 1:
                self.waypoint_index = 0
            self.waypoint_index += 1
    
       


class TrafficManager(CompatibleNode):
    def __init__(self):
        super(TrafficManager, self).__init__("traffic_interface")

        self.map_name = self.get_param("map_name", "")

        if not self.map_name:
            self.logwarn("No map name provided, exiting...")
            return

        self.waypoints = self.load_waypoints()
        carla_client = carla.Client(
            host='localhost',
            port=2000)
        carla_client.set_timeout(10.0)
        self.carla_world = carla_client.get_world()
        self.vehicles = []
        self.on_tick_ids = []
        self.remove_old_npc()        
        num_npc =  self.get_param('num_npc', 2)
        for num in range(num_npc):
            self.generate_npc(num)

    def remove_old_npc(self):
        old_vehicles = self.carla_world.get_actors().filter('vehicle.f1tenth.npc')
        for vehicle in old_vehicles:
            self.loginfo(f"Removing old NPC: {vehicle.id}")
            vehicle.destroy()
    
    
    def load_waypoints(self):
        file_path = f"/opt/ws/src/traffic_interface/maps/{self.map_name}/global_waypoints.json"
        with open(file_path, 'r') as f:
            data = json.load(f)
        
        if not data:
            self.logwarn(f"No waypoints found in {file_path}")
            return []
        
        return data["waypoints"]

    def generate_npc(self, npc_id):
        npc_bp = self.carla_world.get_blueprint_library().filter("npc")[0]
        npc_bp.set_attribute('role_name', f"npc_{npc_id}")
        carla_actor = self.carla_world.spawn_actor(npc_bp, self.get_random_spawn_point())
        carla_actor.set_simulate_physics(False)
        self.vehicles.append(carla_actor)
        pose_publisher = self.new_publisher(
            PoseStamped,
            f"/npc_{npc_id}/pose",
            10
        )
        controller = PositionController(
            npc_id, 
            self.waypoints, 
            carla_actor, 
            self.carla_world, 
            logwarn=self.logwarn, 
            loginfo=self.loginfo,
            pose_publisher=pose_publisher
            )
        
        new_id = self.carla_world.on_tick(controller.update_pose)
        controller.set_callback_id(new_id)
        self.on_tick_ids.append(new_id)
        

    def get_random_spawn_point(self):
        transform = carla.Transform()
        transform.location.x = random.uniform(-6, 3)
        transform.location.y = random.uniform(-5, 10)
        transform.location.z = 0.5  
        transform.rotation.yaw = 0
        return transform

    def destroy(self):
        for vehicle in self.vehicles:
            if vehicle.is_alive: 
                vehicle.destroy()
        for on_tick_id in self.on_tick_ids:
            self.carla_world.remove_on_tick(on_tick_id)
        self.loginfo('Traffic Manager destroyed')
        

def main():
    roscomp.init("traffic_interface", args=None)
    executor = roscomp.executors.MultiThreadedExecutor()
    ti = TrafficManager()
    executor.add_node(ti)
    roscomp.on_shutdown(ti.destroy)
    ti.spin()


if __name__ == '__main__':
    main()
