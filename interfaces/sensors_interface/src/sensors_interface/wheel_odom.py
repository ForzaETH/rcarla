#!/bin/python3

import carla
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from nav_msgs.msg import Odometry
from rcarla_msgs.srv import OdomUpdate


class OdomInterface(CompatibleNode):
    def __init__(self):
        super(OdomInterface, self).__init__("odometry_interface")

        self.simulate_odom = self.get_param("simulate_odom", True)
        self.from_sim = self.get_param("from_simulator", True)
        self.rate = self.get_param("rate", 100.0)
        
        if not self.simulate_odom:
            self.logwarn("Odometry simulation is disabled. No odometry data will be published.")
            return
        
        if self.from_sim:
            self.update_acc_service = self.new_service(
                OdomUpdate,
                "/rcarla/sensors_interface/odom_update",
                self.odom_update_cb,
                )
        else:
            self.odom_sub = self.new_subscription(
                Odometry,
                "/carla/ego_vehicle/odometry",
                self.odom_cb,
                10
            )

        self.msg = None
        self.last_time = 0.0

        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        self.world = client.get_world()

        self.odom_pub = self.new_publisher(
            Odometry,
            "/rcarla/ego_vehicle/wheel_odom",
            10
        )

        self.cb_id = self.world.on_tick(self.run)

    def run(self, snapshot):
        if self.msg is not None and snapshot.timestamp.elapsed_seconds - self.last_time > 1.0 / self.rate:
            self.last_time = snapshot.timestamp.elapsed_seconds
            self.msg.header.stamp = roscomp.ros_timestamp(snapshot.timestamp.elapsed_seconds, from_sec=True)
            self.odom_pub.publish(self.msg)



    def odom_update_cb(self, req, response=None):
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.twist.twist = req.twist
        msg.pose.pose = req.pose
        self.msg = msg
        response = roscomp.get_service_response(OdomUpdate)
        response.success = True
        return response

    def odom_cb(self, msg):
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.twist.twist.linear.y = 0.0
        self.msg = msg
       
    
    def destroy(self):
        if self.from_sim:
            self.destroy_service(self.update_acc_service)
        else:
            self.destroy_subscription(self.odom_sub)
        self.destroy_publisher(self.odom_pub)
        self.world.remove_on_tick(self.cb_id)
        

def main():
    roscomp.init("odometry_interface", args=None)
    executor = roscomp.executors.SingleThreadedExecutor()
    oi = OdomInterface()
   
    executor.add_node(oi)
    roscomp.on_shutdown(oi.destroy)
    oi.spin()


if __name__ == '__main__':
    main()

