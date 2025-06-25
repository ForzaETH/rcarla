#!/bin/python3
import math

import carla
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from transforms3d.euler import quat2euler, euler2quat
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

from rcarla_msgs.srv import ImuUpdate


class ImuInterface(CompatibleNode):
    def __init__(self):
        super(ImuInterface, self).__init__("imu_interface")

        self.simulate_imu = self.get_param("simulate_imu", True)
        if not self.simulate_imu:
            self.logwarn("IMU simulation is disabled. No IMU data will be published.")
            return
        self.loginfo("IMU simulation is enabled. IMU data will be published.")
        self.rate = self.get_param("rate", 100.0)
        self.loginfo(f"IMU rate set to {self.rate} Hz")
        self.from_sim = self.get_param("from_simulator", True)
        self.last_time = None
        self.last_transforms = []


        self.ax = 0.0
        self.ay = 0.0
        self.yaw_rate = 0.0
        self.orientation = Quaternion()

        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        self.world = client.get_world()


        if self.from_sim:
            self.update_acc_service = self.new_service(
            ImuUpdate,
            "/rcarla/sensors_interface/imu_update",
            self.imu_update_callback,
            )
        else:
            self.get_ego_vehicle()


        self.imu_pub = self.new_publisher(
            Imu,
            "/imu",
            10
        )
        self.cb_id = self.world.on_tick(self.run)


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


    def calculate_acceleration(self, dt, xt, xt_1, xt_2, xt_3, xt_4):
        return (-xt + 16*xt_1 - 30*xt_2 + 16*xt_3 - xt_4) / (12 * dt ** 2)


    def calculate_velocity(self, dt, xt, xt_1):
        xt = math.radians(xt)
        xt_1 = math.radians(xt_1)

        delta = xt - xt_1
        if delta > math.pi:
            delta -= 2 * math.pi
        elif delta < -math.pi:
            delta += 2 * math.pi
        
        return delta / dt


    def imu_update_callback(self, req, response=None):
        self.ax = req.ax
        self.ay = req.ay
        self.yaw_rate = req.yaw_rate
        self.orientation = req.orientation
        response = roscomp.get_service_response(ImuUpdate)
        response.success = True
        return response

    def update_from_world(self, dt):
        transform = self.ego_vehicle.get_transform()

        if len(self.last_transforms) < 5:
            self.last_transforms.append(transform)
            return


        acc_x_world = self.calculate_acceleration(
            dt,
            transform.location.x,
            self.last_transforms[-1].location.x,
            self.last_transforms[-2].location.x,
            self.last_transforms[-3].location.x,
            self.last_transforms[-4].location.x
        )
        acc_y_world = self.calculate_acceleration(
            dt,
            transform.location.y,
            self.last_transforms[-1].location.y,
            self.last_transforms[-2].location.y,
            self.last_transforms[-3].location.y,
            self.last_transforms[-4].location.y
        )
        yaw = -math.radians(transform.rotation.yaw)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        self.ax = acc_x_world * cos_yaw - acc_y_world * sin_yaw
        self.ay = acc_x_world * sin_yaw + acc_y_world * cos_yaw
        self.yaw_rate = self.calculate_velocity(
            dt,
            - transform.rotation.yaw,
            - self.last_transforms[-1].rotation.yaw
        )

        quat = euler2quat(
            math.radians(transform.rotation.roll),
            math.radians(transform.rotation.pitch),
            - math.radians(transform.rotation.yaw)
        )

        self.orientation.x = quat[0]
        self.orientation.y = quat[1]
        self.orientation.z = quat[2]
        self.orientation.w = quat[3]

        self.last_transforms.pop(0)
        self.last_transforms.append(transform)

    def run(self, snapshot):
        if self.last_time is None:
            self.last_time = snapshot.timestamp.elapsed_seconds
            return

        current_time = snapshot.timestamp.elapsed_seconds
        dt = current_time - self.last_time

        if dt < 1.0 / self.rate:
            return

        if not self.from_sim:
            self.update_from_world(dt)

        self.last_time = current_time
        self.publish_imu(roscomp.ros_timestamp(current_time, from_sec=True))

        

    def publish_imu(self, time):
        msg = Imu()
        msg.header.stamp = time
        msg.header.frame_id = "base_link"
        
        msg.linear_acceleration.x = self.ax
        msg.linear_acceleration.y = self.ay
        msg.linear_acceleration.z = 9.81 
        msg.angular_velocity.z = self.yaw_rate
        msg.orientation = self.orientation
        self.imu_pub.publish(msg)
       
    
    def destroy(self):
        self.world.remove_on_tick(self.cb_id)
        if self.from_sim:
            self.destroy_service(self.update_acc_service)
        

def main():
    roscomp.init("imu_interface", args=None)
    executor = roscomp.executors.SingleThreadedExecutor()
    ii = ImuInterface()
    executor.add_node(ii)
    roscomp.on_shutdown(ii.destroy)
    ii.spin()


if __name__ == '__main__':
    main()

