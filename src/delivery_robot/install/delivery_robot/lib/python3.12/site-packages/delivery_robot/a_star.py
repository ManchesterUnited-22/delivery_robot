import heapq
from .grid_map import GridMap


class AStarPlanner:
    """
    Thuật toán A* tìm đường đi ngắn nhất trên GridMap.
    """

    def __init__(self, grid_map):
        self.map = grid_map

    def heuristic(self, a, b):
        """
        Hàm heuristic Manhattan.
        Dùng cho bản đồ chỉ di chuyển 4 hướng.
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node):
        """
        Lấy 4 ô xung quanh:
        phải, trái, lên, xuống.
        """
        x, y = node

        directions = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1)
        ]

        for dx, dy in directions:
            nx = x + dx
            ny = y + dy

            if self.map.is_free(nx, ny):
                yield nx, ny

    def plan(self, start_world, goal_world):
        """
        Tìm đường từ start đến goal.

        Input:
            start_world = (x, y)
            goal_world  = (x, y)

        Output:
            danh sách các điểm (x, y) trong world.
        """

        start = self.map.world_to_grid(*start_world)
        goal = self.map.world_to_grid(*goal_world)

        # Kiểm tra điểm bắt đầu
        if not self.map.is_free(*start):
            raise ValueError(
                f"Start {start_world} không nằm trên đường đi."
            )

        # Kiểm tra điểm đích
        if not self.map.is_free(*goal):
            raise ValueError(
                f"Goal {goal_world} không nằm trên đường đi."
            )

        # Priority Queue
        open_set = []

        heapq.heappush(
            open_set,
            (0, start)
        )

        # Lưu node trước đó
        came_from = {}

        # Chi phí từ start đến node
        g_score = {
            start: 0.0
        }

        # f(n) = g(n) + h(n)
        f_score = {
            start: self.heuristic(start, goal)
        }

        while open_set:

            _, current = heapq.heappop(open_set)

            # Đã tới đích
            if current == goal:
                return self.reconstruct_path(
                    came_from,
                    current
                )

            # Xét các node lân cận
            for neighbor in self.get_neighbors(current):

                tentative_g = (
                    g_score[current] + 1.0
                )

                # Nếu tìm được đường tốt hơn
                if tentative_g < g_score.get(
                    neighbor,
                    float("inf")
                ):

                    came_from[neighbor] = current

                    g_score[neighbor] = tentative_g

                    f_score[neighbor] = (
                        tentative_g
                        + self.heuristic(
                            neighbor,
                            goal
                        )
                    )

                    heapq.heappush(
                        open_set,
                        (
                            f_score[neighbor],
                            neighbor
                        )
                    )

        # Không tìm thấy đường
        return []

    def reconstruct_path(self, came_from, current):
        """
        Truy ngược từ goal về start
        để lấy toàn bộ đường đi.
        """

        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        # Đảo lại thành start -> goal
        path.reverse()

        # Đổi grid coordinate -> world coordinate
        world_path = []

        for gx, gy in path:
            world_path.append(
                self.map.grid_to_world(gx, gy)
            )

        return world_path


# ==============================
# TEST A*
# ==============================

if __name__ == "__main__":

    # Tạo bản đồ
    grid = GridMap(resolution=1.0)

    # Tạo A*
    planner = AStarPlanner(grid)

    # Điểm bắt đầu
    start = (-20.0, 0.0)

    # Điểm giao hàng
    goal = (10.0, 10.0)

    # Tìm đường
    path = planner.plan(
        start,
        goal
    )

    print("================================")
    print("       A* GLOBAL PLANNER")
    print("================================")

    print("Start:", start)
    print("Goal :", goal)

    print("So diem tren path:", len(path))

    print("\nPath:")

    for point in path:
        print(point)