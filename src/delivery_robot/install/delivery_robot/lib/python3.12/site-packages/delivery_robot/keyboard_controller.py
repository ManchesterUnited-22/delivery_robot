import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AutoController(Node):

    def __init__(self):
        super().__init__('auto_controller')

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Thời gian bắt đầu chu trình
        self.start_time = self.get_clock().now()

        # Cập nhật lệnh điều khiển 10 lần/giây
        self.timer = self.create_timer(
            0.1,
            self.control
        )

        self.get_logger().info('Auto Controller started')
        self.get_logger().info(
            'Xe tu dong: tien -> trai -> tien -> lui -> phai -> tien'
        )

    def control(self):

        # Tính thời gian đã chạy
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        # Một chu kỳ dài 20 giây
        t = elapsed % 20.0

        msg = Twist()

        # ==========================================
        # 0 - 4 giây: ĐI TIẾN
        # ==========================================
        if 0 <= t < 4:
            msg.linear.x = 0.6
            msg.angular.z = 0.0

        # ==========================================
        # 4 - 6 giây: RẼ TRÁI
        # ==========================================
        elif 4 <= t < 6:
            msg.linear.x = 0.6
            msg.angular.z = 0.7

        # ==========================================
        # 6 - 10 giây: ĐI TIẾN
        # ==========================================
        elif 6 <= t < 10:
            msg.linear.x = 0.6
            msg.angular.z = 0.0

        # ==========================================
        # 10 - 14 giây: ĐI LÙI
        # ==========================================
        elif 10 <= t < 14:
            msg.linear.x = -0.6
            msg.angular.z = 0.0

        # ==========================================
        # 14 - 16 giây: RẼ PHẢI
        # ==========================================
        elif 14 <= t < 16:
            msg.linear.x = 0.6
            msg.angular.z = -0.7

        # ==========================================
        # 16 - 20 giây: ĐI TIẾN
        # ==========================================
        else:
            msg.linear.x = 0.6
            msg.angular.z = 0.0

        # Gửi lệnh điều khiển
        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    controller = AutoController()

    try:
        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

    finally:
        # Khi dừng node thì gửi lệnh dừng xe
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        controller.publisher.publish(msg)

        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()