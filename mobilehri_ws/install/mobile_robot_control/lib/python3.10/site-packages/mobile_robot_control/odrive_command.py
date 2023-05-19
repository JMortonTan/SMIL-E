import math
import rclpy
import odrive
import tf2_ros
# import std_srvs.srv
from rclpy.node import Node
from odrive.enums import *
from std_msgs.msg import *
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, TransformStamped

class odrive_command(Node):
    def __init__(self):
        super().__init__("command_lisener")
        self.odrv = odrive.find_any()
        self.axis0=self.odrv.axis0
        self.axis1=self.odrv.axis1
        self.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.axis0.config.enable_watchdog = False
        self.axis1.config.enable_watchdog = False

        self.odrv.clear_errors()
        
        self.axis0.config.enable_watchdog = True
        self.axis1.config.enable_watchdog = True

        # self.axis0.config.watchdog_timeout = 4
        # self.axis1.config.watchdog_timeout = 4

        self.axis0.watchdog_feed()
        self.axis1.watchdog_feed()

        self.odrv.clear_errors()

        self.get_logger().info("odrv initialization done")

        self.old_pos_l = 0
        self.old_pos_r = 0
        timer_period = 0.1 # seconds
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.odom_timer = self.create_timer(timer_period, self.odom_timer_callback)

        # setup message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"
        self.encoder_cpr = self.axis0.encoder.config.cpr
        self.odom_calc_hz = 10
        # rospy.Service('reset_odometry',    std_srvs.srv.Trigger, self.reset_odometry)

        # store current location to be updated.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # TODO: Robot Wheel Specifications -------------------------------
        # self.R is wheel radius (in meters)
        self.R = 0.07
        self.tyre_circumference = 2*math.pi*self.R
        # Adjust the wheel track to account for your robot
        self.wheel_track = 0.51
        # ----------------------------------------------------------------

        self.tf_publisher = tf2_ros.TransformBroadcaster(self)
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = "odom"
        self.tf_msg.child_frame_id  = "base_link"

        self.twist_sub = self.create_subscription(Twist, "/cmd_vel", self.diff_drive_callback, 10)
        self.get_logger().info("ros initialization done")


    def diff_drive_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        # TODO: Your Code Here -------------------------------------------
        Vl = (v - self.wheel_track * w / 2) / self.R
        Vr = (v + self.wheel_track * w / 2) / self.R
        # raise NotImplementedError("Differential Drive Controller, remove this line once implemented.")
        # ----------------------------------------------------------------

        # convert to turns per sec (ODrive Unit)
        Vl = Vl / 2*math.pi
        Vr = Vr / 2*math.pi

        self.feed_watchdog()

        if abs(Vl) <= 0.08:
            self.axis0.requested_state = AXIS_STATE_IDLE
        else:
            self.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.axis0.controller.input_vel = Vl / 10

        if abs(Vr) <= 0.08:
            self.axis1.requested_state = AXIS_STATE_IDLE
        else:
            self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.axis1.controller.input_vel = -1 * Vr / 10
        return

    def feed_watchdog(self):
        self.axis0.watchdog_feed()
        self.axis1.watchdog_feed()
        return

######################################### ODOMETRY #################################
    # def reset_odometry(self, request):
    #     self.x = 0.0
    #     self.y = 0.0
    #     self.theta = 0.0
    #     return(True, "Odometry reset.")

    def odom_timer_callback(self):
        time_now = self.get_clock().now()
        self.vel_l, self.vel_r = 0, 0
        self.new_pos_l, self.new_pos_r = 0, 0
        self.current_l, self.current_r = 0, 0
        self.temp_v_l, self.temp_v_r = 0., 0
        self.m_s_to_value = self.encoder_cpr/self.tyre_circumference # calculated
        try:
            self.vel_l = self.axis0.encoder.vel_estimate    # units: encoder counts/s
            self.vel_r = -self.axis1.encoder.vel_estimate # neg is forward for right
            self.new_pos_l = self.axis0.encoder.pos_cpr_counts
            self.new_pos_r = -self.axis1.encoder.pos_cpr_counts        # units: encoder counts
            self.pub_odometry(time_now)
        except Exception as e:
            self.get_logger().info(e)
            pass


    def pub_odometry(self, time_now):
        now_stamp = time_now.to_msg()
        self.odom_msg.header.stamp = now_stamp
        self.tf_msg.header.stamp = now_stamp
        # Twist/velocity: calculated from motor values only
        s = self.tyre_circumference * (self.vel_l+self.vel_r) / (2.0*self.encoder_cpr)
        w = self.tyre_circumference * (self.vel_r-self.vel_l) / (self.wheel_track * self.encoder_cpr) # angle: vel_r*tyre_radius - vel_l*tyre_radius
        self.odom_msg.twist.twist.linear.x = s
        self.odom_msg.twist.twist.angular.z = w
        
        # Position
        delta_pos_l = self.new_pos_l - self.old_pos_l
        delta_pos_r = self.new_pos_r - self.old_pos_r

        self.old_pos_l = self.new_pos_l
        self.old_pos_r = self.new_pos_r

        # Check for overflow. Assume we can't move more than half a circumference in a single timestep.
        half_cpr = self.encoder_cpr/2.0
        if   delta_pos_l >  half_cpr: delta_pos_l = delta_pos_l - self.encoder_cpr
        elif delta_pos_l < -half_cpr: delta_pos_l = delta_pos_l + self.encoder_cpr
        if   delta_pos_r >  half_cpr: delta_pos_r = delta_pos_r - self.encoder_cpr
        elif delta_pos_r < -half_cpr: delta_pos_r = delta_pos_r + self.encoder_cpr

        # counts to metres
        delta_pos_l_m = delta_pos_l / self.m_s_to_value
        delta_pos_r_m = delta_pos_r / self.m_s_to_value

        # Distance travelled
        d = (delta_pos_l_m+delta_pos_r_m)/2.0  # delta_ps
        th = (delta_pos_r_m-delta_pos_l_m)/self.wheel_track # works for small angles

        xd = math.cos(th)*d
        yd = -math.sin(th)*d

        # Pose: updated from previous pose + position delta
        self.x += math.cos(self.theta)*xd - math.sin(self.theta)*yd
        self.y += math.sin(self.theta)*xd + math.cos(self.theta)*yd
        self.theta = (self.theta + th) % (2*math.pi)

        # fill odom message and publish

        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        self.odom_msg.pose.pose.orientation.z = q[2] # math.sin(self.theta)/2
        self.odom_msg.pose.pose.orientation.w = q[3] # math.cos(self.theta)/2

        self.tf_msg.transform.translation.x = self.x
        self.tf_msg.transform.translation.y = self.y
        self.tf_msg.transform.rotation.z = q[2]
        self.tf_msg.transform.rotation.w = q[3]

        self.odom_publisher.publish(self.odom_msg)
        self.tf_publisher.sendTransform(self.tf_msg)

    def shutdown(self):
        self.axis0.requested_state = AXIS_STATE_IDLE
        self.axis1.requested_state = AXIS_STATE_IDLE
