import rclpy
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from fsd_path_planning import ConeTypes, MissionTypes, PathPlanner
import fsd_path_planning
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Pose as RosPose
from eufs_msgs.msg import (ConeArrayWithCovariance, WaypointArrayStamped,
                           Waypoint, PointArray)
from slam_tools.slam_node_tools import quaternion_to_angle

from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
import numpy as np
from numba import jit, config
# config.DISABLE_JIT = True

def cone_array_with_covariance_to_internal_cone_format(cones):
    cone_array = [[], [], [], [], []]
    #for cone in cones.unknown_color_cones:
    #    cone_array[0].append([cone.point.x, cone.point.y])
    #for cone in cones.yellow_cones:
    #    cone_array[1].append([cone.point.x, cone.point.y])
    #for cone in cones.blue_cones:
    #    cone_array[2].append([cone.point.x, cone.point.y])
    #for cone in cones.orange_cones:
    #    cone_array[3].append([cone.point.x, cone.point.y])
    #for cone in cones.big_orange_cones:
    #    cone_array[4].append([cone.point.x, cone.point.y])
    for cone in cones.unknown_color_cones:
        cone_array[0].append([cone.point.x, cone.point.y])
    for cone in cones.yellow_cones:
        cone_array[0].append([cone.point.x, cone.point.y])
    for cone in cones.blue_cones:
        cone_array[0].append([cone.point.x, cone.point.y])
    for cone in cones.orange_cones:
        cone_array[0].append([cone.point.x, cone.point.y])
    for cone in cones.big_orange_cones:
        cone_array[0].append([cone.point.x, cone.point.y])
    cone_array = [np.array(cones) for cones in cone_array]
    return cone_array

class Fsd(Node):
    def __init__(self):
        super().__init__("ft_fsd")
        self.declare_parameter('use_slam', True)
        if self.get_parameter('use_slam').get_parameter_value().bool_value:
            self.cone_sub = self.create_subscription(ConeArrayWithCovariance, '/map', self.cb, 1)
            self.pose_sub = self.create_subscription(PoseStamped, '/pose', self.pose_cb, 1)
            self.frame_id = "odom"
        else:
            self.cone_sub = self.create_subscription(ConeArrayWithCovariance, '/cones', self.cb, 1)
            self.frame_id = "base_footprint"
        self.raceline_publisher = self.create_publisher(PointArray, "/raceline", 1)
        self.waypoint_publisher = self.create_publisher(WaypointArrayStamped, "/trajectory", 1)
        # self.car_rotation = np.array([1.0, 0.0])
        self.car_position = np.array([0.0, 0.0])
        self.angle = 0.0
        # self.fig, self.axis = plt.subplots()
        # self.axis.set_aspect('equal', adjustable='box')
        # [self.cones_plot] = plt.plot([], [], "g*")
        # [self.path_plot]  = plt.plot([], [], "r+")
        # self.plots = [self.cones_plot, self.path_plot]
        # self.anim = anim.FuncAnimation(self.fig, lambda _: self.plots, init_func=lambda: self.plots)
        # self.create_timer(0.05, self.render)
        # plt.show(block=False)
        # self.cones_plot_data = None
        # self.path_plot_data = None

    # def render(self):
    #     if self.cones_plot_data and self.path_plot_data:
    #         self.cones_plot.set_data(self.cones_plot_data)
    #         self.path_plot.set_data(self.path_plot_data)
    #         self.fig.canvas.start_event_loop(0.01)

    def pose_cb(self, pose: PoseStamped):
        from numpy import array, float64
        self.angle = quaternion_to_angle(pose.pose.orientation)
        self.car_position = np.array([pose.pose.position.x, pose.pose.position.y])
        # self.get_logger().info(f"Angle is {self.angle} and position is {self.car_position}")

    def cb(self, cone_array: ConeArrayWithCovariance):
        inp = cone_array
        cone_array = cone_array_with_covariance_to_internal_cone_format(cone_array)
        if len(cone_array[0]) == 0:
            self.get_logger().warn("Received empty cone frame, skipping")
            return
        # diff = cone_array[0] - self.car_position
        # cone_array[0] = diff[np.linalg.norm(diff, axis=1) < 15]
        # self.get_logger().info(f"Going to processs {len(cone_array[0])} cones")
        try:
            out = PathPlanner(MissionTypes.trackdrive).calculate_path_in_global_frame(
                cone_array,
                np.array(self.car_position),
                self.angle,
            )
        except Exception as e:
            self.get_logger().error(f"Error in ft_fsd: {e}")
            return
        # cones_data = [*cone_array[0].T]
        # path_data = [*out[:, 1:3].T]
        # pose_data = self.car_position
        # def z():
        #     plt.scatter(*cones_data)
        #     plt.scatter(*path_data)
        #     plt.scatter(*pose_data)
        #     plt.gcf().canvas.start_event_loop(0.01)
        #     # plt.show(block=False)
        # breakpoint()
        points = [Point(x=x, y=y, z=0.0) for (x,y) in out[:, 1:3]]
        self.raceline_publisher.publish(PointArray(points=points))
        
        msg = WaypointArrayStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = inp.header.stamp
        msg.waypoints = [Waypoint(position=point) for point in points]
        self.waypoint_publisher.publish(msg)
        self.get_logger().info("path planner alive and publishing", throttle_duration_sec=10.0)

def main(args=None):
    rclpy.init(args=args)
    fsd = Fsd()
    rclpy.spin(fsd)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
