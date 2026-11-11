class GridMap:
    """
    Bản đồ lưới cho campus.

    0 = không đi được
    1 = đường robot có thể đi
    """

    def __init__(self, resolution=1.0):
        self.resolution = resolution

        # Phạm vi campus
        self.x_min = -15
        self.x_max = 15
        self.y_min = -10
        self.y_max = 10

        self.width = int(
            (self.x_max - self.x_min) / self.resolution
        ) + 1

        self.height = int(
            (self.y_max - self.y_min) / self.resolution
        ) + 1

        self.grid = [
            [0 for _ in range(self.height)]
            for _ in range(self.width)
        ]

        self.create_roads()
        self.create_obstacles()

    # --------------------------------------------------
    # World -> Grid
    # --------------------------------------------------

    def world_to_grid(self, x, y):
        gx = int((x - self.x_min) / self.resolution)
        gy = int((y - self.y_min) / self.resolution)

        return gx, gy

    # --------------------------------------------------
    # Grid -> World
    # --------------------------------------------------

    def grid_to_world(self, gx, gy):
        x = self.x_min + gx * self.resolution
        y = self.y_min + gy * self.resolution

        return x, y

    # --------------------------------------------------
    # Kiểm tra ô
    # --------------------------------------------------

    def is_valid(self, gx, gy):
        return (
            0 <= gx < self.width
            and
            0 <= gy < self.height
        )

    def is_free(self, gx, gy):
        if not self.is_valid(gx, gy):
            return False

        return self.grid[gx][gy] == 1

    # --------------------------------------------------
    # Tạo các đoạn đường
    # --------------------------------------------------

    def create_roads(self):

        # ==============================
        # ĐƯỜNG CHÍNH
        # x: -15 -> 15
        # y: -2 -> 2
        # ==============================

        for x in range(-15, 16):
            for y in range(-2, 3):

                gx, gy = self.world_to_grid(x, y)

                if self.is_valid(gx, gy):
                    self.grid[gx][gy] = 1

        # ==============================
        # NHÁNH GIẢNG ĐƯỜNG
        # x ≈ 7
        # y: 2 -> 6  (tòa nhà bắt đầu từ y = 6.5)
        # ==============================

        for x in range(6, 9):
            for y in range(2, 7):

                gx, gy = self.world_to_grid(x, y)

                if self.is_valid(gx, gy):
                    self.grid[gx][gy] = 1

        # ==============================
        # NHÁNH THƯ VIỆN
        # x ≈ -7
        # y: -7 -> -1  (tòa nhà kết thúc ở y = -7)
        # ==============================

        for x in range(-8, -5):
            for y in range(-7, 0):

                gx, gy = self.world_to_grid(x, y)

                if self.is_valid(gx, gy):
                    self.grid[gx][gy] = 1

        # ==============================
        # NHÁNH SÂN TRƯỜNG
        # x ≈ 12
        # y: 2 -> 6
        # ==============================

        for x in range(11, 14):
            for y in range(2, 7):

                gx, gy = self.world_to_grid(x, y)

                if self.is_valid(gx, gy):
                    self.grid[gx][gy] = 1

    # --------------------------------------------------
    # Vật cản nằm trên đường (khớp với file world)
    # --------------------------------------------------

    def create_obstacles(self):

        # (danh sách x, danh sách y) theo tọa độ world nguyên
        blocks = [
            ([0], [1, 2]),             # planter_box  (0, 1.5)
            ([5], [-2, -1]),           # traffic_cone (5, -1.5)
            ([-4, -3, -2], [-2, -1]),  # bench_1      (-3, -1.5)
        ]

        for xs, ys in blocks:
            for x in xs:
                for y in ys:

                    gx, gy = self.world_to_grid(x, y)

                    if self.is_valid(gx, gy):
                        self.grid[gx][gy] = 0