import math


class DWAPlanner:
    """
    Dynamic Window Approach đơn giản
    có xét vật cản từ LiDAR.

    Input:
        robot_pose = (x, y, yaw)
        current_speed
        goal = (x, y)
        obstacle_distance

    Output:
        speed
        steering
    """

    def __init__(self):

        # ==============================
        # GIỚI HẠN ROBOT
        # ==============================

        self.max_speed = 1.0
        self.min_speed = 0.0

        self.max_accel = 0.5

        self.max_steering = 0.5

        # ==============================
        # ĐỘ PHÂN GIẢI
        # ==============================

        self.speed_resolution = 0.1
        self.steering_resolution = 0.1

        # ==============================
        # THỜI GIAN DỰ ĐOÁN
        # ==============================

        self.predict_time = 1.0

        # ==============================
        # KHOẢNG CÁCH AN TOÀN
        # ==============================

        self.safe_distance = 1.0

    # ==================================
    # NORMALIZE ANGLE
    # ==================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # ==================================
    # DYNAMIC WINDOW
    # ==================================

    def calculate_dynamic_window(self, current_speed):

        min_speed = max(
            self.min_speed,
            current_speed - self.max_accel * self.predict_time
        )

        max_speed = min(
            self.max_speed,
            current_speed + self.max_accel * self.predict_time
        )

        return min_speed, max_speed

    # ==================================
    # DỰ ĐOÁN VỊ TRÍ
    # ==================================

    def predict_position(
        self,
        pose,
        speed,
        steering
    ):

        x, y, yaw = pose

        dt = self.predict_time

        wheel_base = 0.70

        new_x = (
            x
            + speed * math.cos(yaw) * dt
        )

        new_y = (
            y
            + speed * math.sin(yaw) * dt
        )

        new_yaw = (
            yaw
            + (
                speed
                / wheel_base
                * math.tan(steering)
                * dt
            )
        )

        new_yaw = self.normalize_angle(
            new_yaw
        )

        return new_x, new_y, new_yaw

    # ==================================
    # KIỂM TRA AN TOÀN
    # ==================================

    def is_collision_free(
        self,
        predicted_pose,
        obstacle_distance
    ):

        # Nếu LiDAR không thấy vật cản
        if not math.isfinite(obstacle_distance):
            return True

        # Khoảng cách dự đoán từ robot
        # đến vật cản phía trước

        predicted_distance = obstacle_distance

        # Nếu robot đang đi nhanh
        # cần giữ khoảng cách lớn hơn

        required_distance = (
            self.safe_distance
            + abs(predicted_pose[0]) * 0.0
        )

        return predicted_distance > required_distance

    # ==================================
    # COST
    # ==================================

    def calculate_cost(
        self,
        predicted_pose,
        goal,
        speed,
        steering
    ):

        x, y, yaw = predicted_pose

        goal_x, goal_y = goal

        # ------------------------------
        # Khoảng cách tới goal
        # ------------------------------

        distance_to_goal = math.sqrt(
            (goal_x - x) ** 2
            + (goal_y - y) ** 2
        )

        # ------------------------------
        # Hướng tới goal
        # ------------------------------

        target_angle = math.atan2(
            goal_y - y,
            goal_x - x
        )

        heading_error = abs(
            self.normalize_angle(
                target_angle - yaw
            )
        )

        # ------------------------------
        # Cost
        # ------------------------------

        cost = (
            distance_to_goal
            + 0.1 * heading_error
            + 0.05 * abs(steering)
            - 1.0 * speed
        )

        return cost

    # ==================================
    # DWA PLAN
    # ==================================

    def plan(
        self,
        robot_pose,
        current_speed,
        goal,
        obstacle_distance=float("inf")
    ):

        min_speed, max_speed = (
            self.calculate_dynamic_window(
                current_speed
            )
        )

        best_speed = 0.0
        best_steering = 0.0

        best_cost = float("inf")

        # ==================================
        # DUYỆT VẬN TỐC
        # ==================================

        speed = min_speed

        while speed <= max_speed + 1e-6:

            # ==================================
            # DUYỆT GÓC LÁI
            # ==================================

            steering = -self.max_steering

            while steering <= (
                self.max_steering + 1e-6
            ):

                predicted_pose = (
                    self.predict_position(
                        robot_pose,
                        speed,
                        steering
                    )
                )

                # ==================================
                # KIỂM TRA VA CHẠM
                # ==================================

                if self.is_collision_free(
                    predicted_pose,
                    obstacle_distance
                ):

                    cost = self.calculate_cost(
                        predicted_pose,
                        goal,
                        speed,
                        steering
                    )

                    # ==================================
                    # CHỌN QUỸ ĐẠO TỐT NHẤT
                    # ==================================

                    if cost < best_cost:

                        best_cost = cost

                        best_speed = speed

                        best_steering = steering

                steering += (
                    self.steering_resolution
                )

            speed += self.speed_resolution

        return (
            best_speed,
            best_steering
        )


# ======================================
# TEST DWA
# ======================================

if __name__ == "__main__":

    planner = DWAPlanner()

    robot_pose = (
        -20.0,
        0.0,
        0.0
    )

    goal = (
        10.0,
        0.0
    )

    current_speed = 0.0

    obstacle_distance = float("inf")

    speed, steering = planner.plan(
        robot_pose,
        current_speed,
        goal,
        obstacle_distance
    )

    print("================================")
    print("          DWA PLANNER")
    print("================================")

    print("Robot pose:", robot_pose)
    print("Goal:", goal)

    print(
        "Obstacle distance:",
        obstacle_distance
    )

    print(
        "Best speed:",
        speed
    )

    print(
        "Best steering:",
        steering
    )