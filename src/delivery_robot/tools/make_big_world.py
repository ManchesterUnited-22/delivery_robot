#!/usr/bin/env python3
"""
Sinh map lớn từ map gốc bằng cách nhân 10 (5 cột x 2 hàng) và ghép lại.

    python3 tools/make_big_world.py

Đầu vào : worlds/delivery_world_1x.sdf   (1 ô map gốc, 42 m x 30 m)
Đầu ra  : worlds/delivery_world_10x.sdf  (map lớn 210 m x 60 m)

Cách ghép:
  * Mỗi ô giữ nguyên toàn bộ toà nhà, cây, đèn, điểm giao hàng... của map gốc,
    chỉ dịch toạ độ (+42 m theo x, +30 m theo y) và đổi tên có tiền tố rXcY_.
  * Bỏ mặt đất + tường rào riêng của từng ô, thay bằng 1 mặt đất và 1 tường
    rào chung -> không còn đường nối/vạch giữa các ô.
  * Đường chính kéo liền xuyên suốt cả 5 ô (mỗi hàng 1 đường dài 210 m).
  * Mỗi ô có thêm 1 "đại lộ" dọc ở mép trái (x = -19.6) nối đường chính của
    hàng dưới với hàng trên -> xe chạy được giữa mọi ô, tạo thành lưới đường.
  * Vạch kẻ đường và vỉa hè được vẽ liền mạch qua các ô.

Giảm tải cho Gazebo (mặc định bật, thêm --full để giữ nguyên chi tiết):
  * Tắt đổ bóng (nặng nhất).
  * Bỏ bớt khối trang trí nhỏ (tán lá phụ, hoa, gờ mái, bậu cửa...).
  * Gộp toàn bộ model trang trí (không va chạm) của mỗi ô thành 1 model.
Vị trí, kích thước toà nhà/đường/va chạm giữ nguyên -> A* không bị ảnh hưởng.
"""
import argparse
import copy
import xml.etree.ElementTree as ET
from pathlib import Path

# ----------------------------------------------------------------------------
# THÔNG SỐ
# ----------------------------------------------------------------------------
TILE_W, TILE_H = 42.0, 30.0   # kích thước 1 ô (khớp với ground 42 x 30 gốc)
COLS, ROWS = 5, 2             # 5 x 2 = 10 ô
AVENUE_X = -19.6              # đại lộ dọc (toạ độ cục bộ trong ô)
AVENUE_W = 2.8                # bề rộng đại lộ: x = -21.0 .. -18.2
ROAD_HALF = 2.6               # nửa bề rộng đường chính (5.2 / 2)

# Các model của map gốc sẽ được vẽ lại ở mức toàn cục -> không copy theo ô
SKIP_EXACT = {"campus_ground", "main_road"}
SKIP_PREFIX = ("wall_", "road_center_dash_")

# Visual bị bỏ khi giảm tải: (tiền tố tên model) -> tập tên visual
DROP_VISUALS = {
    "tree_": {"leaves_left", "leaves_right", "leaves_front"},
    "hedge_": {"flower_0", "flower_1", "flower_2"},
    "lamp_": {"base", "arm"},
    "lecture_hall": {"plinth", "cornice", "roof_ridge", "door_frame",
                     "win_sill_0_l", "win_sill_0_r", "awning_post_l",
                     "awning_post_r", "porch", "side_win_l", "side_win_r"},
    "library": {"plinth", "cornice", "roof_ridge", "door_frame",
                "win_sill_0_l", "win_sill_0_r", "awning_post_l",
                "awning_post_r", "porch", "side_win_l", "side_win_r"},
    "admin_building": {"plinth", "cornice", "roof_ridge", "door_frame",
                       "win_sill_0_l", "win_sill_0_r", "awning_post_l",
                       "awning_post_r", "porch", "side_win_l", "side_win_r"},
}

# Màu lấy đúng từ map gốc
C_GROUND = ("0.149 0.31 0.105 1", "0.24 0.5 0.17 1")
C_WALL = ("0.341 0.329 0.31 1", "0.55 0.53 0.5 1")
C_ROAD = ("0.136 0.136 0.149 1", "0.22 0.22 0.24 1")
C_SIDEWALK = ("0.459 0.446 0.415 1", "0.74 0.72 0.67 1")
C_MARK = ("0.589 0.589 0.558 1", "0.95 0.95 0.9 1")


# ----------------------------------------------------------------------------
# HÀM TIỆN ÍCH
# ----------------------------------------------------------------------------
def box_model(name, x, y, z, sx, sy, sz, color, collision=False, link="link"):
    """Tạo 1 model tĩnh hình hộp."""
    coll = ""
    if collision:
        coll = (
            f'<collision name="collision"><geometry><box>'
            f'<size>{sx} {sy} {sz}</size></box></geometry></collision>'
        )
    xml = (
        f'<model name="{name}"><static>true</static>'
        f'<pose>{x:g} {y:g} {z:g} 0 0 0</pose>'
        f'<link name="{link}">{coll}'
        f'<visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size>'
        f'</box></geometry><material><ambient>{color[0]}</ambient>'
        f'<diffuse>{color[1]}</diffuse></material></visual>'
        f'</link></model>'
    )
    return ET.fromstring(xml)


def shift_pose(model, dx, dy):
    pose = model.find("pose")
    v = [float(t) for t in pose.text.split()]
    v[0] += dx
    v[1] += dy
    pose.text = " ".join(f"{t:g}" for t in v)


def drop_visuals(model):
    """Bỏ các visual nhỏ (giảm tải). Trả về số visual đã bỏ."""
    name = model.get("name")
    n = 0
    for key, names in DROP_VISUALS.items():
        if name.startswith(key):
            for link in model.findall("link"):
                for v in list(link.findall("visual")):
                    if v.get("name") in names:
                        link.remove(v)
                        n += 1
    return n


def has_collision(model):
    return model.find(".//collision") is not None


def _pose(el):
    p = el.find("pose")
    return [float(t) for t in p.text.split()] if p is not None else [0.0] * 6


def merge_decor(name, models):
    """Gộp nhiều model trang trí (không va chạm) thành 1 model 1 link.
    Chỉ gộp model không xoay; model có xoay được trả lại để giữ riêng."""
    merged = ET.fromstring(
        f'<model name="{name}"><static>true</static>'
        f'<pose>0 0 0 0 0 0</pose><link name="link"/></model>')
    link = merged.find("link")
    leftovers = []
    for m in models:
        mp = _pose(m)
        parts = []
        ok = not any(abs(a) > 1e-9 for a in mp[3:])
        for l in m.findall("link"):
            lp = _pose(l)
            ok = ok and not any(abs(a) > 1e-9 for a in lp[3:])
            for v in l.findall("visual"):
                vp = _pose(v)
                ok = ok and not any(abs(a) > 1e-9 for a in vp[3:])
                parts.append((v, lp, vp))
        if not ok:
            leftovers.append(m)
            continue
        for v, lp, vp in parts:
            v2 = copy.deepcopy(v)
            v2.set("name", f"{m.get('name')}__{v.get('name')}")
            old = v2.find("pose")
            if old is not None:
                v2.remove(old)
            pose = ET.Element("pose")
            pose.text = " ".join(
                f"{mp[i] + lp[i] + vp[i]:g}" for i in range(3)) + " 0 0 0"
            v2.insert(0, pose)
            link.append(v2)
    return merged, leftovers


def tile_extras(prefix, dx, dy):
    """Các chi tiết bổ sung cho từng ô (đại lộ, vỉa hè và vạch nối)."""
    out = []
    ax = AVENUE_X + dx
    seg_len = TILE_H / 2 - ROAD_HALF          # 12.4 m mỗi đoạn
    seg_c = ROAD_HALF + seg_len / 2           # tâm đoạn = 8.8 m

    # Đại lộ dọc: 2 đoạn (phía bắc / nam đường chính) để không trùng mặt
    # phẳng với đường chính tại ngã tư.
    for tag, sgn in (("n", 1), ("s", -1)):
        out.append(box_model(
            f"{prefix}avenue_{tag}", ax, dy + sgn * seg_c, 0.015,
            AVENUE_W, seg_len, 0.04, C_ROAD, collision=True, link="road"))

    # Vạch đứt giữa đại lộ (bỏ vùng ngã tư)
    for k, yy in enumerate((4.5, 7.5, 10.5, 13.5)):
        for sgn in (1, -1):
            out.append(box_model(
                f"{prefix}avenue_dash_{k}_{'n' if sgn > 0 else 's'}",
                ax, dy + sgn * yy, 0.045, 0.12, 1.5, 0.008, C_MARK))

    # Vỉa hè + vạch mép lấp đoạn từ x=16 -> mép ô và từ mép đại lộ -> x=-16
    for tag, cx, w in (("e", 18.5, 5.0), ("w", -17.1, 2.2)):
        for sgn, side in ((1, "n"), (-1, "s")):
            out.append(box_model(
                f"{prefix}sidewalk_fill_{tag}{side}", dx + cx, dy + sgn * 3.05,
                0.025, w, 0.9, 0.05, C_SIDEWALK))
            out.append(box_model(
                f"{prefix}edge_fill_{tag}{side}", dx + cx, dy + sgn * 2.38,
                0.045, w, 0.08, 0.008, C_MARK))

    # Vạch đứt giữa đường chính, bước 3 m, bỏ vùng ngã tư
    for m in range(-16, 21, 3):
        out.append(box_model(
            f"{prefix}center_dash_{m + 16}", dx + m, dy, 0.045,
            1.5, 0.12, 0.008, C_MARK))
    return out


# ----------------------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------------------
def build(src, dst, lite=True, shadows=False):
    tree = ET.parse(src)
    root = tree.getroot()
    world = root.find("world")

    tile_models = []     # model của 1 ô (đã loại các phần vẽ lại toàn cục)
    new_world = ET.Element("world", name=world.get("name"))
    for child in list(world):
        if child.tag == "model":
            name = child.get("name")
            if name in SKIP_EXACT or name.startswith(SKIP_PREFIX):
                continue
            tile_models.append(child)
        else:
            if not shadows:
                if child.tag == "scene" and child.find("shadows") is not None:
                    child.find("shadows").text = "false"
                if child.tag == "light" and child.find("cast_shadows") is not None:
                    child.find("cast_shadows").text = "false"
            new_world.append(child)          # plugin, scene, light

    W, H = COLS * TILE_W, ROWS * TILE_H
    cx = (COLS - 1) * TILE_W / 2             # tâm map lớn
    cy = (ROWS - 1) * TILE_H / 2

    # Mặt đất chung
    new_world.append(box_model("campus_ground", cx, cy, -0.112, W, H, 0.24,
                               C_GROUND, collision=True, link="ground"))
    # Tường rào chung (chỉ trang trí)
    t = 0.3
    new_world.append(box_model("wall_north", cx, cy + H / 2 - t / 2, 0.25,
                               W, t, 0.5, C_WALL))
    new_world.append(box_model("wall_south", cx, cy - H / 2 + t / 2, 0.25,
                               W, t, 0.5, C_WALL))
    new_world.append(box_model("wall_west", cx - W / 2 + t / 2, cy, 0.25,
                               t, H, 0.5, C_WALL))
    new_world.append(box_model("wall_east", cx + W / 2 - t / 2, cy, 0.25,
                               t, H, 0.5, C_WALL))
    # Đường chính liền mạch cho từng hàng
    for r in range(ROWS):
        new_world.append(box_model(
            f"main_road_r{r}", cx, r * TILE_H, 0.015, W, 5.2, 0.04,
            C_ROAD, collision=True, link="road"))

    # Nhân 10 ô
    for r in range(ROWS):
        for c in range(COLS):
            dx, dy = c * TILE_W, r * TILE_H
            prefix = f"r{r}c{c}_"
            tile_all = []
            for m in tile_models:
                m2 = copy.deepcopy(m)
                if lite:
                    drop_visuals(m2)         # phải làm trước khi đổi tên
                m2.set("name", prefix + m.get("name"))
                shift_pose(m2, dx, dy)
                tile_all.append(m2)
            tile_all += tile_extras(prefix, dx, dy)

            if lite:
                solid = [m for m in tile_all if has_collision(m)]
                decor = [m for m in tile_all if not has_collision(m)]
                merged, leftovers = merge_decor(prefix + "decor", decor)
                tile_all = solid + leftovers + [merged]
            for m in tile_all:
                new_world.append(m)

    root.remove(world)
    root.append(new_world)
    ET.indent(tree, space="  ")
    tree.write(dst, encoding="utf-8", xml_declaration=True)

    names = [m.get("name") for m in new_world.findall("model")]
    assert len(names) == len(set(names)), "Tên model bị trùng!"
    nvis = len(new_world.findall(".//visual"))
    print(f"Đã tạo {dst}: {len(names)} model, {nvis} visual, "
          f"map {W:g} x {H:g} m ({COLS}x{ROWS} = {COLS * ROWS} ô)"
          f" | {'GIẢM TẢI' if lite else 'ĐẦY ĐỦ CHI TIẾT'}"
          f" | đổ bóng {'BẬT' if shadows else 'TẮT'}")


if __name__ == "__main__":
    here = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=here / "worlds/delivery_world_1x.sdf")
    ap.add_argument("--dst", default=here / "worlds/delivery_world_10x.sdf")
    ap.add_argument("--full", action="store_true",
                    help="giữ đầy đủ chi tiết, không gộp model")
    ap.add_argument("--shadows", action="store_true", help="bật đổ bóng")
    a = ap.parse_args()
    build(a.src, a.dst, lite=not a.full, shadows=a.shadows)