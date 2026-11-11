import matplotlib.pyplot as plt

from grid_map import GridMap
from a_star import AStarPlanner


# Tạo bản đồ
grid = GridMap(resolution=1.0)

# Tạo A*
planner = AStarPlanner(grid)

# Điểm bắt đầu
start = (-20.0, 0.0)

# Điểm giao hàng
goal = (10.0, 10.0)

# Tìm đường
path = planner.plan(start, goal)


# ==============================
# VẼ GRID MAP
# ==============================

plt.figure(figsize=(12, 6))

# Vẽ các ô đường
for gx in range(grid.width):
    for gy in range(grid.height):

        if grid.is_free(gx, gy):

            x, y = grid.grid_to_world(gx, gy)

            plt.scatter(
                x,
                y,
                s=8
            )


# ==============================
# VẼ PATH A*
# ==============================

path_x = [point[0] for point in path]
path_y = [point[1] for point in path]

plt.plot(
    path_x,
    path_y,
    linewidth=3,
    label="A* Path"
)


# Điểm Start
plt.scatter(
    start[0],
    start[1],
    s=100,
    marker="o",
    label="Start"
)


# Điểm Goal
plt.scatter(
    goal[0],
    goal[1],
    s=100,
    marker="X",
    label="Goal"
)


# ==============================
# CẤU HÌNH
# ==============================

plt.xlabel("X (m)")
plt.ylabel("Y (m)")

plt.title("A* Global Planner")

plt.grid(True)

plt.axis("equal")

plt.legend()

plt.show()