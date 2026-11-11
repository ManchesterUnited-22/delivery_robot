import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry

from .dwa import DWAPlanner


class DWANode(Node):

    def __init__(self):
        super().__init__("dwa_node")

        # =========================
        # DWA PLANNER
        # =========================

        self.planner = DWAPlanner()

        # =========================
        # LIDAR
        # =========================

        self.scan_data = None
        self.min_obstacle_distance = float("inf")

        # =========================
        # ROBOT POSE
        # =========================

        self.robot_pose = (
            0.0,
            -1.0,
            0.0
        )

        # =========================
        # GLOBAL PATH
        # =========================

        self.global_path = []
        self.current_waypoint_index = 0
        self.goal = None

        # Khoảng cách cho phép để xem
        # như đã tới waypoint
        self.waypoint_tolerance = 0.5

        # =========================
        # SPEED
        # =========================

        self.current_speed = 0.0

        # =========================
        # THỐNG KÊ CHUYẾN GIAO HÀNG
        # =========================

        self.arrived = False
        self.start_time = None
        self.total_distance = 0.0
        self.prev_xy = None
        self.min_clearance = float("inf")

        # Nếu odom bắt đầu từ (0, 0) thay vì tọa độ world,
        # đổi thành (-14.0, -1.0).
        # Kiểm tra bằng:
        # ros2 topic echo /model/delivery_robot/odometry --once
        self.odom_offset = (0.0, 0.0)

        # =========================
        # SUBSCRIBER - LIDAR
        # =========================

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        # =========================
        # SUBSCRIBER - A*
        # =========================

        self.path_subscriber = self.create_subscription(
            PoseArray,
            "/global_path",
            self.path_callback,
            10
        )

        # =========================
        # SUBSCRIBER - ODOMETRY
        # =========================

        self.odom_subscriber = self.create_subscription(
            Odometry,
            "/model/delivery_robot/odometry",
            self.odom_callback,
            10
        )

        # =========================
        # PUBLISHER - CMD_VEL
        # =========================

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # =========================
        # CONTROL LOOP
        # =========================

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            "DWA Node da khoi dong"
        )

    # ==================================================
    # LIDAR CALLBACK
    # ==================================================

    def scan_callback(self, msg):

        self.scan_data = msg

        front_ranges = []

        for i, r in enumerate(msg.ranges):

            if not math.isfinite(r):
                continue

            if r < msg.range_min or r > msg.range_max:
                continue

            angle = (
                msg.angle_min
                + i * msg.angle_increment
            )

            # Vùng phía trước robot
            # -45 độ đến +45 độ
            if -math.pi / 4 <= angle <= math.pi / 4:
                front_ranges.append(r)

        if front_ranges:

            front_ranges.sort()

            # Ghi nhận khoảng cách gần vật cản nhất
            if self.start_time is not None and not self.arrived:
                self.min_clearance = min(
                    self.min_clearance,
                    front_ranges[0]
                )

            middle = len(front_ranges) // 2

            if len(front_ranges) % 2 == 0:

                self.min_obstacle_distance = (
                    front_ranges[middle - 1]
                    + front_ranges[middle]
                ) / 2.0

            else:

                self.min_obstacle_distance = (
                    front_ranges[middle]
                )

        else:

            self.min_obstacle_distance = float("inf")

    # ==================================================
    # ODOMETRY CALLBACK
    # ==================================================

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Quaternion -> Yaw
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        x += self.odom_offset[0]
        y += self.odom_offset[1]

        self.robot_pose = (
            x,
            y,
            yaw
        )

        # Cộng dồn quãng đường thực tế
        if self.start_time is not None and not self.arrived:

            if self.prev_xy is not None:
                self.total_distance += math.dist(
                    self.prev_xy,
                    (x, y)
                )

            self.prev_xy = (x, y)

    # ==================================================
    # GLOBAL PATH CALLBACK
    # ==================================================

    def path_callback(self, msg):

        new_path = []

        for pose in msg.poses:

            new_path.append(
                (
                    pose.position.x,
                    pose.position.y
                )
            )

        # ==================================================
        # CHỈ RESET KHI PATH THỰC SỰ THAY ĐỔI
        # ==================================================

        if new_path != self.global_path:

            self.global_path = new_path

            self.current_waypoint_index = 0

            if self.global_path:

                self.goal = self.global_path[0]

                self.get_logger().info(
                    f"Da nhan Global Path: "
                    f"{len(self.global_path)} diem"
                )

                self.get_logger().info(
                    f"Waypoint bat dau: "
                    f"({self.goal[0]:.1f}, "
                    f"{self.goal[1]:.1f})"
                )

    # ==================================================
    # CONTROL LOOP
    # ==================================================

    def control_loop(self):

        # Chưa có LiDAR
        if self.scan_data is None:
            return

        # Chưa có Global Path
        if not self.global_path:
            return

        # Đã tới đích: giữ xe đứng yên
        if self.arrived:
            self.stop_robot()
            return

        # Bắt đầu bấm giờ khi xe bắt đầu chạy
        if self.start_time is None:
            self.start_time = (
                self.get_clock().now().nanoseconds * 1e-9
            )

        # ==================================================
        # KIỂM TRA ĐÃ ĐI HẾT WAYPOINT CHƯA
        # ==================================================

        if (
            self.current_waypoint_index
            >= len(self.global_path)
        ):

            self.stop_robot()

            return

        # ==================================================
        # LẤY WAYPOINT HIỆN TẠI
        # ==================================================

        self.goal = self.global_path[
            self.current_waypoint_index
        ]

        goal_x = self.goal[0]
        goal_y = self.goal[1]

        robot_x = self.robot_pose[0]
        robot_y = self.robot_pose[1]

        # ==================================================
        # TÍNH KHOẢNG CÁCH
        # ==================================================

        distance = math.sqrt(
            (goal_x - robot_x) ** 2
            + (goal_y - robot_y) ** 2
        )

        # ==================================================
        # ĐÃ TỚI WAYPOINT
        # ==================================================

        if distance < self.waypoint_tolerance:

            self.get_logger().info(
                f"Da toi waypoint "
                f"{self.current_waypoint_index + 1}/"
                f"{len(self.global_path)}: "
                f"({goal_x:.1f}, {goal_y:.1f})"
            )

            self.current_waypoint_index += 1

            # ==================================================
            # ĐÃ TỚI ĐÍCH CUỐI
            # ==================================================

            if (
                self.current_waypoint_index
                >= len(self.global_path)
            ):

                self.arrived = True

                self.stop_robot()

                self.report_arrival()

                return

            # Chuyển sang waypoint tiếp theo
            self.goal = self.global_path[
                self.current_waypoint_index
            ]

            return

        # ==================================================
        # DWA
        # ==================================================

        speed, steering = self.planner.plan(
            self.robot_pose,
            self.current_speed,
            self.goal,
            self.min_obstacle_distance
        )

        # ==================================================
        # VẬT CẢN QUÁ GẦN
        # ==================================================

        if self.min_obstacle_distance < 0.5:

            speed = 0.0

            self.get_logger().warn(
                f"Vat can qua gan: "
                f"{self.min_obstacle_distance:.2f} m"
            )

        # ==================================================
        # PUBLISH CMD_VEL
        # ==================================================

        cmd = Twist()

        cmd.linear.x = speed
        wheel_base = 0.70
        cmd.angular.z = speed / wheel_base * math.tan(steering)

        self.cmd_vel_publisher.publish(cmd)

        self.current_speed = speed

    # ==================================================
    # BÁO CÁO KẾT QUẢ KHI TỚI ĐÍCH
    # ==================================================

    def report_arrival(self):

        now = self.get_clock().now().nanoseconds * 1e-9

        elapsed = now - self.start_time

        gx, gy = self.global_path[-1]

        rx, ry, ryaw = self.robot_pose

        error = math.dist((rx, ry), (gx, gy))

        straight = math.dist(self.global_path[0], (gx, gy))

        planned = sum(
            math.dist(
                self.global_path[i],
                self.global_path[i + 1]
            )
            for i in range(len(self.global_path) - 1)
        )

        if elapsed > 0:
            avg_speed = self.total_distance / elapsed
        else:
            avg_speed = 0.0

        if self.min_clearance == float("inf"):
            clearance = "N/A"
        else:
            clearance = f"{self.min_clearance:.2f} m"

        log = self.get_logger().info

        log("=" * 46)
        log("   DA TOI DIEM GIAO HANG - KET QUA CHUYEN DI")
        log("=" * 46)
        log(f"Diem giao hang       : ({gx:.1f}, {gy:.1f})")
        log(f"Vi tri xe luc dung   : ({rx:.2f}, {ry:.2f})")
        log(f"Sai so toi diem dich : {error:.2f} m")
        log(f"Thoi gian di chuyen  : {elapsed:.1f} s")
        log(f"Quang duong thuc te  : {self.total_distance:.1f} m")
        log(f"Quang duong A*       : {planned:.1f} m")
        log(f"Khoang cach thang    : {straight:.1f} m")
        log(f"Van toc trung binh   : {avg_speed:.2f} m/s")
        log(f"Cach vat can gan nhat: {clearance}")
        log("=" * 46)

    # ==================================================
    # STOP ROBOT
    # ==================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)

        self.current_speed = 0.0


# ======================================================
# MAIN
# ======================================================

def main(args=None):

    rclpy.init(args=args)

    node = DWANode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.stop_robot()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()