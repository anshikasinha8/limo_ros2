#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import matplotlib.pyplot as plt

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def ang_wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class FourCorners(Node):
    def __init__(self):
        super().__init__('four_corners')

        # Tune
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('w_max', 1.2)
        self.declare_parameter('k_rho', 0.8)   # distance gain
        self.declare_parameter('k_alpha', 2.0) # heading gain
        self.declare_parameter('pos_tol', 0.08)
        self.declare_parameter('yaw_tol', 0.12)

        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.k_rho = self.get_parameter('k_rho').value
        self.k_alpha = self.get_parameter('k_alpha').value
        self.pos_tol = self.get_parameter('pos_tol').value
        self.yaw_tol = self.get_parameter('yaw_tol').value

        # Define corners in odom frame (2m x 2m)
        #time = math.linspace(0, 2*math.pi)
        self.goals = [(0.0, 0.0),
                      (2.0, 0.0),
                      (3.0, 1.5),
                      (2.0, 2.0),
                      (1.5, 3.0),
                      (0.0, 2.0)]
        self.goal_idx = 0

        self.x = None
        self.y = None
        self.yaw = None

        self.x_data = []
        self.y_data = []

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("FourCorners started. Waiting for /odom...")

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.x0 is None:
            self.x0, self.y0, self.yaw0 = self.x, self.y, self.yaw
            self.get_logger().info(f"Latched initial pose: ({self.x:.2f}, {self.y:.2f}), yaw = {math.degrees(self.yaw):.1f} deg")

        # Set local frame
        self.x -= self.x0
        self.y -= self.y0
        self.yaw = ang_wrap(self.yaw - self.yaw0)

    def plot_xy(self):
        self.x_data.append(self.x)
        self.y_data.append(self.y)
        plt.clf()
        plt.plot(self.x_data, self.y_data, 'b-')
        plt.plot([g[0] for g in self.goals], [g[1] for g in self.goals], 'ro')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)') 
        plt.pause(0.001)

        

    def control_loop(self):
        if self.x is None:
            return

        gx, gy = self.goals[self.goal_idx]
        dx = gx - self.x
        dy = gy - self.y
        rho = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        alpha = ang_wrap(desired - self.yaw)

        cmd = Twist()

        # If close enough to goal position, rotate to face the next segment
        if rho < self.pos_tol:
            next_idx = (self.goal_idx + 1) % len(self.goals)
            ngx, ngy = self.goals[next_idx]
            ndx = ngx - gx
            ndy = ngy - gy
            next_heading = math.atan2(ndy, ndx)
            yaw_err = ang_wrap(next_heading - self.yaw)

            if abs(yaw_err) < self.yaw_tol:
                self.goal_idx = next_idx
                self.get_logger().info(f"Reached corner {self.goal_idx}: ({gx:.2f},{gy:.2f})")
            else:
                cmd.angular.z = clamp(self.k_alpha * yaw_err, -self.w_max, self.w_max)
                cmd.linear.x = 0.0
        else:
            # Drive toward current goal
            cmd.angular.z = clamp(self.k_alpha * alpha, -self.w_max, self.w_max)
            # slow down
            facing = max(0.0, math.cos(alpha))
            cmd.linear.x = clamp(self.k_rho * rho * facing, 0.0, self.v_max)

        self.plot_xy()

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = FourCorners()
    plt.ion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    stop = Twist()
    node.cmd_pub.publish(stop)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
