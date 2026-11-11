class GridMap:
    """
    Bản đồ lưới cho campus (map lớn = COLS x ROWS ô map gốc ghép lại).

    0 = không đi được
    1 = đường robot có thể đi

    Phải khớp với worlds/delivery_world_10x.sdf
    (sinh bằng tools/make_big_world.py).
    """

    # Kích thước 1 ô map gốc (m) và số ô ghép
    TILE_W = 42
    TILE_H = 30
    COLS = 5
    ROWS = 2

    def __init__(self, resolution=1.0, cols=COLS, rows=ROWS):
        self.resolution = resolution
        self.cols = cols
        self.rows = rows

        # Phạm vi campus: ô (0,0) nằm quanh gốc toạ độ như map gốc
        self.x_min = -21
        self.x_max = 21 + self.TILE_W * (cols - 1)
        self.y_min = -15
        self.y_max = 15 + self.TILE_H * (rows - 1)

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
    # Tiện ích tô ô đường
    # --------------------------------------------------

    def _fill(self, xs, ys, value=1):
        for x in xs:
            for y in ys:
                gx, gy = self.world_to_grid(x, y)

                if self.is_valid(gx, gy):
                    self.grid[gx][gy] = value

    # --------------------------------------------------
    # Tạo các đoạn đường
    # --------------------------------------------------

    def create_roads(self):

        # ==============================
        # ĐƯỜNG CHÍNH: liền mạch xuyên cả COLS ô, mỗi hàng 1 đường
        # y: -2 -> 2 (cộng offset của hàng)
        # ==============================

        for r in range(self.rows):
            dy = r * self.TILE_H
            self._fill(
                range(self.x_min, self.x_max + 1),
                range(-2 + dy, 3 + dy),
            )

        # ==============================
        # ĐẠI LỘ DỌC: mép trái mỗi ô (x = -21 -> -18),
        # chạy suốt chiều cao map, nối các hàng với nhau
        # ==============================

        for c in range(self.cols):
            dx = c * self.TILE_W
            self._fill(
                range(-21 + dx, -18 + dx),
                range(self.y_min, self.y_max + 1),
            )

        # ==============================
        # NHÁNH TRONG TỪNG Ô (giống map gốc, cộng offset ô)
        # ==============================

        for r in range(self.rows):
            for c in range(self.cols):
                dx = c * self.TILE_W
                dy = r * self.TILE_H

                # Nhánh giảng đường: x ≈ 7, y: 2 -> 6
                self._fill(range(6 + dx, 9 + dx), range(2 + dy, 7 + dy))

                # Nhánh thư viện: x ≈ -7, y: -7 -> -1
                self._fill(range(-8 + dx, -5 + dx), range(-7 + dy, 0 + dy))

                # Nhánh sân trường: x ≈ 12, y: 2 -> 6
                self._fill(range(11 + dx, 14 + dx), range(2 + dy, 7 + dy))

    # --------------------------------------------------
    # Vật cản nằm trên đường (lặp lại ở mọi ô)
    # --------------------------------------------------

    def create_obstacles(self):

        # (danh sách x, danh sách y) theo tọa độ world nguyên, trong 1 ô
        blocks = [
            ([0], [1, 2]),             # planter_box  (0, 1.5)
            ([5], [-2, -1]),           # traffic_cone (5, -1.5)
            ([-4, -3, -2], [-2, -1]),  # bench_1      (-3, -1.5)
        ]

        for r in range(self.rows):
            for c in range(self.cols):
                dx = c * self.TILE_W
                dy = r * self.TILE_H

                for xs, ys in blocks:
                    self._fill(
                        [x + dx for x in xs],
                        [y + dy for y in ys],
                        value=0,
                    )