import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose

from .grid_map import GridMap
from .a_star import AStarPlanner


class AStarNode(Node):

    def __init__(self):
        super().__init__("a_star_node")

        # Tạo GridMap
        self.grid_map = GridMap(resolution=1.0)

        # Tạo A* planner
        self.planner = AStarPlanner(self.grid_map)

        # Publisher xuất Global Path
        self.path_publisher = self.create_publisher(
            PoseArray,
            "/global_path",
            10
        )

        # Tạo timer chạy A* mỗi 2 giây
        self.timer = self.create_timer(
            2.0,
            self.plan_path
        )

        # Start và Goal
        self.start = (-14.0, -1.0)
        self.goal = (7.0, 5.8)   # delivery_point_1 (xanh lá)
        # DP2 (xanh dương): (-7.0, -5.8)
        # DP3 (cam):        (12.0, 5.8)

        self.logged = False

        self.get_logger().info(
            "A* Node da khoi dong"
        )

    def plan_path(self):

        # Tìm đường bằng A*
        t0 = time.perf_counter()

        path = self.planner.plan(
            self.start,
            self.goal
        )

        plan_ms = (time.perf_counter() - t0) * 1000.0

        if not path:
            self.get_logger().error(
                "Khong tim thay duong!"
            )
            return

        # Thêm điểm đích chính xác vào cuối path
        # (A* làm tròn về ô lưới 1m nên điểm cuối bị lệch)
        path = list(path)

        if math.dist(path[-1], self.goal) > 0.05:
            path.append(self.goal)

        # Tạo message PoseArray
        path_msg = PoseArray()

        path_msg.header.stamp = self.get_clock().now().to_msg()

        path_msg.header.frame_id = "map"

        # Chuyển từng điểm A* thành Pose
        for x, y in path:

            pose = Pose()

            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0

            # Không cần orientation ở bước này
            pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        # Publish Global Path
        self.path_publisher.publish(path_msg)

        # Chỉ log thông số 1 lần
        if not self.logged:

            length = sum(
                math.dist(path[i], path[i + 1])
                for i in range(len(path) - 1)
            )

            self.get_logger().info(
                f"A*: {len(path)} diem | dai {length:.1f} m | "
                f"tinh toan {plan_ms:.1f} ms | "
                f"start={self.start} goal={self.goal}"
            )

            self.logged = True


def main(args=None):

    rclpy.init(args=args)

    node = AStarNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()