#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能：
1) 自动检测 rosbag 中雷达点云类型：
   - sensor_msgs/PointCloud2  (如 /hesai/pandar)
   - livox_ros_driver/CustomMsg (如 /livox/lidar)
2) 按各自的解析方式把点云导出成一个带 intensity 的 PCD 文件 (x y z intensity, ASCII)
3) 使用 Open3D 对该 PCD 进行交互选点（至少 4 个点），并根据 4 个点计算包围范围，
   保存为同名 .txt 文件。
4) [新功能] 直接接受 .pcd 文件和距离过滤参数，自动计算标定板四个圆孔的雷达坐标。

依赖：
    - rosbag (仅 bag 模式)
    - sensor_msgs.point_cloud2 (仅 bag 模式)
    - open3d, numpy, scipy

用法示例（旧模式）：
    python scripts/distance_filter_tool.py /path/to/data.bag /path/to/output_dir
    python scripts/distance_filter_tool.py /path/to/cloud.pcd
    python scripts/distance_filter_tool.py /path/to/cloud.pcd /path/to/output_dir

用法示例（新模式 — 自动检测圆孔坐标）：
    python scripts/distance_filter_tool.py --pcd mid_360.pcd \\
        --x_min 2.3 --x_max 4.8 \\
        --y_min -0.7 --y_max 3.2 \\
        --z_min -0.1 --z_max 1.9

    可选参数（对应 config/qr_params.yaml 中的标定板几何参数）：
        --circle_radius        圆孔半径，默认 0.12 m
        --delta_width_circles  左右两圆圆心间距，默认 0.5 m
        --delta_height_circles 上下两圆圆心间距，默认 0.4 m
        --output               结果输出 JSON 文件路径，默认 <pcd_basename>_circles.json
"""

import os
import sys
import json
import numpy as np
import open3d as o3d

def import_rosbag_dependencies():
    """按需导入 rosbag 相关依赖，避免纯 PCD 模式受 ROS 环境限制。"""
    try:
        import rosbag
        import sensor_msgs.point_cloud2 as pc2
    except ImportError as exc:
        print(f"[ERROR] 处理 rosbag 需要 ROS Python 依赖: {exc}", file=sys.stderr)
        return None, None
    return rosbag, pc2

# ===================== 通用：保存 PCD =====================

def save_pcd_with_intensity(points, intensities, output_path):
    """
    保存点云为带 intensity 字段的 PCD 文件 (ASCII 格式)
    points: list/ndarray of [x, y, z]
    intensities: list/ndarray of intensity
    """
    N = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {N}
HEIGHT 1
POINTS {N}
DATA ascii
"""
    with open(output_path, 'w') as f:
        f.write(header)
        for (x, y, z), inten in zip(points, intensities):
            f.write(f"{x} {y} {z} {inten}\n")
    print(f"[PCD] 保存带 intensity 字段的点云到: {output_path}")

# ===================== 情况 1：PointCloud2 =====================

def find_intensity_field(msg):
    """在 PointCloud2 的 fields 中自动检测强度字段名称"""
    candidates = ["intensity", "reflectivity", "i", "ref"]
    for field in msg.fields:
        if field.name.lower() in candidates:
            return field.name
    return None


def convert_pointcloud2_bag_to_pcd(
    bag_file,
    output_dir,
    topic_name="/hesai/pandar",                        # 如有不同，可改成 topic 名称
    pcd_name="sensor_PointCloud2_inten_ascii.pcd"
):
    """
    将 rosbag 中 PointCloud2 类型的点云合并导出为一个 PCD 文件。
    保持原始雷达坐标，不做坐标变换。
    """
    rosbag, pc2 = import_rosbag_dependencies()
    if rosbag is None or pc2 is None:
        return None

    print(f"[Bag] 打开 rosbag: {bag_file}")
    bag = rosbag.Bag(bag_file, "r")

    # 1) 先检测强度字段
    intensity_field = None
    for topic, msg, t in bag.read_messages():
        if msg._type == "sensor_msgs/PointCloud2":
            intensity_field = find_intensity_field(msg)
            if intensity_field:
                print(f"[Bag] 检测到 intensity 字段: {intensity_field}")
            break

    if not intensity_field:
        print("[ERROR] 未找到强度字段! 退出 PointCloud2 转换。", file=sys.stderr)
        bag.close()
        return None

    # 2) 读取指定 topic 的所有点云
    all_points = []
    all_intensities = []

    print(f"[Bag] 开始从 topic '{topic_name}' 读取 PointCloud2 点云...")

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        if msg._type == "sensor_msgs/PointCloud2":
            try:
                field_names = ["x", "y", "z", intensity_field]
                for point in pc2.read_points(msg, field_names=field_names, skip_nans=True):
                    all_points.append([point[0], point[1], point[2]])
                    all_intensities.append(point[3])  # 强度是第四个字段
            except Exception as e:
                print(f"[ERROR] 读取错误: {str(e)}", file=sys.stderr)
                continue

    bag.close()

    if not all_points:
        print("[ERROR] 未找到 PointCloud2 点云数据！", file=sys.stderr)
        return None

    output_path = os.path.join(output_dir, pcd_name)
    save_pcd_with_intensity(all_points, all_intensities, output_path)
    return output_path

# ===================== 情况 2：Livox CustomMsg =====================

def parse_livox_custom_msg(msg):
    """
    从 livox_ros_driver/CustomMsg 中解析 x, y, z, reflectivity
    假设 msg.points 是 CustomPoint 对象列表，字段为 x, y, z, reflectivity
    """
    points = []
    intensities = []

    for pt in msg.points:
        points.append([pt.x, pt.y, pt.z])
        intensities.append(pt.reflectivity)

    return points, intensities

def convert_livox_custom_bag_to_pcd(
    bag_file,
    output_dir,
    topic_name="/livox/lidar",                     # 如有不同，可改成 topic 名称
    pcd_name="livox_CustomMsg_inten_ascii.pcd"
):
    """
    将 rosbag 中 livox_ros_driver/CustomMsg 类型的点云合并导出为一个 PCD 文件。
    保持原始雷达坐标，不做坐标变换。
    """
    rosbag, _ = import_rosbag_dependencies()
    if rosbag is None:
        return None

    print(f"[Bag] 打开 rosbag: {bag_file}")
    bag = rosbag.Bag(bag_file, "r")

    all_points = []
    all_intensities = []

    print(f"[Bag] 开始从 topic '{topic_name}' 读取 CustomMsg 点云...")

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        if msg._type == "livox_ros_driver/CustomMsg":
            pts, intens = parse_livox_custom_msg(msg)
            all_points.extend(pts)
            all_intensities.extend(intens)

    bag.close()

    if not all_points:
        print("[ERROR] 未找到 Livox CustomMsg 点云数据!", file=sys.stderr)
        return None

    output_path = os.path.join(output_dir, pcd_name)
    intensities = np.array(all_intensities, dtype=np.float32)
    save_pcd_with_intensity(all_points, intensities, output_path)
    return output_path

# ===================== 自动检测：这个 bag 用哪种方式 =====================

def detect_lidar_msg_type(bag_file):
    """
    在 bag 里扫一圈，检测是否有 PointCloud2 或 Livox CustomMsg。
    返回：
        "PointCloud2", "CustomMsg", 或 None
    如果两种都有，默认优先 PointCloud2,并打印提示。
    """
    rosbag, _ = import_rosbag_dependencies()
    if rosbag is None:
        return None

    has_pc2 = False
    has_livox = False

    print(f"[Detect] 扫描 bag: {bag_file}")
    bag = rosbag.Bag(bag_file, "r")

    for topic, msg, t in bag.read_messages():
        if msg._type == "sensor_msgs/PointCloud2":
            has_pc2 = True
        elif msg._type == "livox_ros_driver/CustomMsg":
            has_livox = True

        if has_pc2 and has_livox:
            break

    bag.close()

    if has_pc2 and has_livox:
        print("[Detect] 同时检测到 PointCloud2 和 Livox CustomMsg, 默认使用 PointCloud2。")
        return "PointCloud2"
    elif has_pc2:
        print("[Detect] 检测到 PointCloud2 点云。")
        return "PointCloud2"
    elif has_livox:
        print("[Detect] 检测到 Livox CustomMsg 点云。")
        return "CustomMsg"
    else:
        print("[Detect] 未检测到 PointCloud2 或 Livox CustomMsg 点云。")
        return None

def pick_points_from_cloud(pcd, target_name):
    """对点云进行交互选点，返回前 4 个选中点。"""
    print(f"\n正在处理: {target_name}")
    print("请在可视化窗口中按住 Shift 用鼠标左键选择点(至少4个)，然后按 Q 键关闭窗口")

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=f"选择点 - {target_name}")
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    selected_indices = vis.get_picked_points()
    if not selected_indices:
        print(f"[ERROR] 未选择任何点，{target_name} 没有保存文件", file=sys.stderr)
        return None

    if len(selected_indices) < 4:
        print(f"[ERROR] 只选中了 {len(selected_indices)} 个点，少于 4 个，跳过该文件", file=sys.stderr)
        return None

    all_points = np.asarray(pcd.points)
    return all_points[selected_indices[:4], :]


def save_selected_points_and_ranges(selected_points, save_file):
    """保存选中点及其扩展 0.2m 后的 xyz 范围。"""
    mins = selected_points.min(axis=0)
    maxs = selected_points.max(axis=0)

    x_min = mins[0] - 0.2
    x_max = maxs[0] + 0.2
    y_min = mins[1] - 0.2
    y_max = maxs[1] + 0.2
    z_min = mins[2] - 0.2
    z_max = maxs[2] + 0.2

    with open(save_file, 'w') as f:
        f.write("# 4 selected points (x y z)\n")
        for p in selected_points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

        f.write("# range values in order:\n")
        f.write(f"x_min: {x_min:.1f}\n")
        f.write(f"x_max: {x_max:.1f}\n")
        f.write(f"y_min: {y_min:.1f}\n")
        f.write(f"y_max: {y_max:.1f}\n")
        f.write(f"z_min: {z_min:.1f}\n")
        f.write(f"z_max: {z_max:.1f}\n")

    print(f"[Save] 已保存选点与范围到: {save_file}")
    print("点云处理完成。")
    return save_file


def process_pcd_file(pcd_path, output_dir=None):
    """直接处理现有 PCD 文件并保存同名 txt。"""
    if not os.path.isfile(pcd_path):
        print(f"[ERROR] PCD 文件不存在: {pcd_path}", file=sys.stderr)
        return None

    if output_dir is None:
        output_dir = os.path.dirname(pcd_path) or os.getcwd()

    if not os.path.isdir(output_dir):
        print(f"[ERROR] 输出目录 '{output_dir}' 不存在", file=sys.stderr)
        return None

    pcd = o3d.io.read_point_cloud(pcd_path)
    target_name = os.path.basename(pcd_path)
    if not pcd.has_points():
        print(f"[ERROR] {target_name} 中没有点云数据，已跳过", file=sys.stderr)
        return None

    selected_points = pick_points_from_cloud(pcd, target_name)
    if selected_points is None:
        return None

    base_name = os.path.splitext(target_name)[0]
    save_file = os.path.join(output_dir, f"{base_name}.txt")
    return save_selected_points_and_ranges(selected_points, save_file)


# ===================== Open3D 交互选点 & 保存范围 =====================

def select_and_save_points(pcd_folder, target_pcd_name):
    """
    在给定目录中读取指定 PCD 文件，用 Open3D 交互式选点并保存范围。
    """
    pcd_path = os.path.join(pcd_folder, target_pcd_name)
    return process_pcd_file(pcd_path, output_dir=pcd_folder)


# ===================== 新功能：直接从 PCD 检测标定板四个圆孔坐标 =====================

def detect_circles_from_pcd(
    pcd_path,
    x_min, x_max,
    y_min, y_max,
    z_min, z_max,
    circle_radius=0.12,
    delta_width_circles=0.5,
    delta_height_circles=0.4,
    output_json=None,
):
    """
    从 PCD 文件中检测标定板的四个圆孔中心坐标（LiDAR 坐标系）。

    算法概述（对应 src/lidar_detect.hpp 中的 ROS 实现）：
      1. 读取 PCD，用距离过滤框裁剪点云。
      2. 使用 Open3D RANSAC 平面拟合，估计标定板所在平面的法向量，
         将点云投影到平面坐标系（z 轴即法向量，xy 平面即板面）。
      3. 在投影后的 2D 点云中，反复用 scipy 的圆拟合（RANSAC 思路）提取圆心，
         直到找到 4 个候选圆心或无法再找到为止。
         - 假设：圆孔半径 circle_radius（默认 0.12 m，与 qr_params.yaml 一致）。
      4. 从候选圆心中，验证是否存在一组 4 个圆心构成矩形，
         宽 delta_width_circles / 高 delta_height_circles（与 qr_params.yaml 一致）。
      5. 将选出的 4 个圆心坐标从板面坐标系反投影回 LiDAR 坐标系输出。

    注意（实现假设）：
      - 滤波后的点云需要以标定板为主体（噪点不能太多）。
      - 圆孔至少需要有足够密度的点覆盖才能被 RANSAC 探测到。
      - 若点云稀疏（如远距离 MID-360），估计精度会有所降低。

    参数：
        pcd_path              : .pcd 文件路径
        x_min/x_max           : X 方向过滤范围（m）
        y_min/y_max           : Y 方向过滤范围（m）
        z_min/z_max           : Z 方向过滤范围（m）
        circle_radius         : 圆孔半径（m），对应 qr_params.yaml circle_radius
        delta_width_circles   : 左右两圆圆心水平间距（m），对应 delta_width_circles
        delta_height_circles  : 上下两圆圆心垂直间距（m），对应 delta_height_circles
        output_json           : 结果保存路径；None 时默认为 <pcd_basename>_circles.json

    返回：
        list of 4 dicts [{x, y, z}, ...] （LiDAR 坐标），检测失败返回 None
    """
    # ---------- 1. 读取并过滤点云 ----------
    if not os.path.isfile(pcd_path):
        print(f"[ERROR] PCD 文件不存在: {pcd_path}", file=sys.stderr)
        return None

    print(f"[Circle] 读取 PCD: {pcd_path}")
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        print("[ERROR] PCD 文件中没有点云数据", file=sys.stderr)
        return None

    pts = np.asarray(pcd.points)
    print(f"[Circle] 原始点数: {len(pts)}")

    mask = (
        (pts[:, 0] >= x_min) & (pts[:, 0] <= x_max) &
        (pts[:, 1] >= y_min) & (pts[:, 1] <= y_max) &
        (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
    )
    filtered = pts[mask]
    print(f"[Circle] 过滤后点数: {len(filtered)}")

    if len(filtered) < 20:
        print("[ERROR] 过滤后点云太少（<20），请检查距离过滤参数", file=sys.stderr)
        return None

    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered)

    # ---------- 2. 平面拟合，把点云投影到板面坐标系 ----------
    # 使用 Open3D 的 RANSAC 平面分割，得到平面法向量 [a, b, c] 和偏置 d
    plane_model, inliers = filtered_pcd.segment_plane(
        distance_threshold=0.02,
        ransac_n=3,
        num_iterations=1000,
    )
    a, b, c, d = plane_model
    normal = np.array([a, b, c])
    normal /= np.linalg.norm(normal)  # 单位化法向量
    print(f"[Circle] 拟合平面法向量: {normal}，平面内点数: {len(inliers)}")

    # 取平面内点参与后续圆检测
    board_pts = filtered[np.array(inliers)]

    # 构建板面坐标系：z_axis=法向量, x_axis/y_axis 任意正交
    z_axis = normal
    # 选一个与 z_axis 不平行的向量来叉积
    arbitrary = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(arbitrary, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)

    # 板面原点：所有内点的质心
    origin = board_pts.mean(axis=0)

    # 投影到 2D 板面坐标（u, v）
    shifted = board_pts - origin
    u = shifted @ x_axis
    v = shifted @ y_axis
    uv = np.column_stack([u, v])

    # ---------- 3. 在 2D 板面上反复 RANSAC 拟合圆 ----------
    # 采用简化的代数圆拟合：给定 3 个点求圆心，统计内点数量
    MAX_CIRCLES = 4
    RANSAC_ITERS = 500
    INLIER_THRESHOLD = 0.015   # 点到圆周距离阈值（m）
    MIN_INLIERS = 5            # 一个有效圆至少需要的内点数

    r_lo = circle_radius - 0.04
    r_hi = circle_radius + 0.04

    def fit_circle_3pts(p1, p2, p3):
        """代数法用 3 点求圆心和半径。失败返回 (None, None)。"""
        ax, ay = p1; bx, by = p2; cx, cy = p3
        D = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        if abs(D) < 1e-10:
            return None, None
        ux = ((ax**2 + ay**2) * (by - cy) + (bx**2 + by**2) * (cy - ay) + (cx**2 + cy**2) * (ay - by)) / D
        uy = ((ax**2 + ay**2) * (cx - bx) + (bx**2 + by**2) * (ax - cx) + (cx**2 + cy**2) * (bx - ax)) / D
        r = np.sqrt((ax - ux)**2 + (ay - uy)**2)
        return np.array([ux, uy]), r

    remaining = uv.copy()
    circle_centers_2d = []
    rng = np.random.default_rng(42)  # single RNG for reproducibility across all circle searches

    for _ in range(MAX_CIRCLES):
        if len(remaining) < MIN_INLIERS:
            break

        best_center = None
        best_count = 0
        best_inlier_mask = None

        for _ in range(RANSAC_ITERS):
            if len(remaining) < 3:
                break
            idx = rng.choice(len(remaining), 3, replace=False)
            center, r = fit_circle_3pts(remaining[idx[0]], remaining[idx[1]], remaining[idx[2]])
            if center is None or r < r_lo or r > r_hi:
                continue
            dists = np.abs(np.linalg.norm(remaining - center, axis=1) - r)
            inlier_mask = dists < INLIER_THRESHOLD
            count = inlier_mask.sum()
            if count > best_count:
                best_count = count
                best_center = center
                best_inlier_mask = inlier_mask

        if best_center is None or best_count < MIN_INLIERS:
            print(f"[Circle] 无法再找到新的圆（已找到 {len(circle_centers_2d)} 个）")
            break

        # 精化圆心：用所有内点做最小二乘
        inlier_pts = remaining[best_inlier_mask]
        # 代数最小二乘：(x-a)^2+(y-b)^2=r^2 => x^2+y^2 = 2ax+2by+(r^2-a^2-b^2)
        # 令 A=[2x, 2y, 1], b_vec=x^2+y^2
        A_mat = np.column_stack([2 * inlier_pts[:, 0], 2 * inlier_pts[:, 1], np.ones(len(inlier_pts))])
        b_vec = inlier_pts[:, 0]**2 + inlier_pts[:, 1]**2
        result, _, _, _ = np.linalg.lstsq(A_mat, b_vec, rcond=None)
        refined_center = result[:2]

        circle_centers_2d.append(refined_center)
        print(f"[Circle] 找到圆心 2D: {refined_center}，内点数: {best_count}")

        # 移除这个圆的内点，继续找下一个
        remaining = remaining[~best_inlier_mask]

    if len(circle_centers_2d) < 4:
        print(f"[ERROR] 只找到 {len(circle_centers_2d)} 个圆，需要 4 个。"
              "建议检查过滤范围或圆孔半径参数。", file=sys.stderr)
        return None

    # ---------- 4. 几何验证：找符合矩形布局的 4 个圆心 ----------
    # 从所有候选圆心中选出 4 个，验证是否满足宽×高的矩形结构
    # （对应 src/lidar_detect.hpp 中 Square 类的几何一致性检验）
    centers_arr = np.array(circle_centers_2d)

    def check_rectangle(pts_4, dw, dh, tol=0.05):
        """
        验证 4 个 2D 点是否构成以 dw×dh 为尺寸的矩形（允许旋转）。
        tol: 允许的边长误差（m）。
        """
        # 计算所有两点间距离
        dists = []
        for i in range(4):
            for j in range(i + 1, 4):
                dists.append(np.linalg.norm(pts_4[i] - pts_4[j]))
        dists.sort()
        # 矩形应有 4 条边（2 种长度各 2 条）和 2 条对角线
        # 期望边长：dw, dh；对角线：sqrt(dw^2+dh^2)
        expected_sides = sorted([dw, dw, dh, dh])
        expected_diags = sorted([np.sqrt(dw**2 + dh**2)] * 2)
        expected = sorted(expected_sides + expected_diags)
        if len(dists) != 6:
            return False
        return all(abs(dists[i] - expected[i]) < tol for i in range(6))

    from itertools import combinations
    best_group = None
    for group in combinations(range(len(centers_arr)), 4):
        pts_4 = centers_arr[list(group)]
        if check_rectangle(pts_4, delta_width_circles, delta_height_circles):
            best_group = list(group)
            break

    if best_group is None:
        print("[WARN] 未找到严格符合矩形几何的 4 个圆心，使用前 4 个候选结果（精度可能较低）。",
              file=sys.stderr)
        best_group = list(range(min(4, len(centers_arr))))

    selected_2d = centers_arr[best_group]

    # ---------- 5. 反投影回 LiDAR 坐标系 ----------
    # 板面的原点 origin 是板内点的质心，它已经位于拟合平面上。
    # 每个 2D 圆心只需沿 x_axis / y_axis 方向平移即可，无需额外法向量偏移。
    lidar_coords = []
    for uv_pt in selected_2d:
        pt_3d = origin + uv_pt[0] * x_axis + uv_pt[1] * y_axis
        lidar_coords.append({"x": float(pt_3d[0]), "y": float(pt_3d[1]), "z": float(pt_3d[2])})

    # ---------- 6. 输出结果 ----------
    print("\n[Circle] ===== 标定板四个圆孔坐标（LiDAR 坐标系）=====")
    for i, coord in enumerate(lidar_coords):
        print(f"  圆孔 {i+1}: x={coord['x']:.4f}  y={coord['y']:.4f}  z={coord['z']:.4f}")
    print("[Circle] ============================================\n")

    result = {
        "input_pcd": pcd_path,
        "distance_filter": {
            "x_min": x_min, "x_max": x_max,
            "y_min": y_min, "y_max": y_max,
            "z_min": z_min, "z_max": z_max,
        },
        "board_params": {
            "circle_radius": circle_radius,
            "delta_width_circles": delta_width_circles,
            "delta_height_circles": delta_height_circles,
        },
        "circle_centers_lidar": lidar_coords,
    }

    if output_json is None:
        base = os.path.splitext(os.path.basename(pcd_path))[0]
        output_json = os.path.join(os.path.dirname(pcd_path) or ".", f"{base}_circles.json")

    with open(output_json, "w") as f:
        json.dump(result, f, indent=2)
    print(f"[Circle] 结果已保存到: {output_json}")

    return lidar_coords


# ===================== main =====================

def _parse_args_detect_mode(argv):
    """简单解析 --pcd / --x_min 等参数（避免引入 argparse 以外的依赖）。"""
    import argparse
    parser = argparse.ArgumentParser(
        description="从 PCD 文件自动检测标定板四个圆孔的 LiDAR 坐标",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例：
  python scripts/distance_filter_tool.py --pcd mid_360.pcd \\
      --x_min 2.3 --x_max 4.8 \\
      --y_min -0.7 --y_max 3.2 \\
      --z_min -0.1 --z_max 1.9

  # 自定义板子几何（与 config/qr_params.yaml 保持一致）：
  python scripts/distance_filter_tool.py --pcd mid_360.pcd \\
      --x_min 2.3 --x_max 4.8 --y_min -0.7 --y_max 3.2 --z_min -0.1 --z_max 1.9 \\
      --circle_radius 0.12 --delta_width_circles 0.5 --delta_height_circles 0.4 \\
      --output result.json
""",
    )
    parser.add_argument("--pcd", required=True, help=".pcd 文件路径")
    parser.add_argument("--x_min", type=float, required=True)
    parser.add_argument("--x_max", type=float, required=True)
    parser.add_argument("--y_min", type=float, required=True)
    parser.add_argument("--y_max", type=float, required=True)
    parser.add_argument("--z_min", type=float, required=True)
    parser.add_argument("--z_max", type=float, required=True)
    parser.add_argument("--circle_radius", type=float, default=0.12,
                        help="圆孔半径（m），默认 0.12，对应 qr_params.yaml circle_radius")
    parser.add_argument("--delta_width_circles", type=float, default=0.5,
                        help="左右两圆心水平间距（m），默认 0.5，对应 delta_width_circles")
    parser.add_argument("--delta_height_circles", type=float, default=0.4,
                        help="上下两圆心垂直间距（m），默认 0.4，对应 delta_height_circles")
    parser.add_argument("--output", default=None,
                        help="输出 JSON 文件路径，默认 <pcd_basename>_circles.json")
    return parser.parse_args(argv)


if __name__ == "__main__":
    # 新模式：有 --pcd 参数时走自动圆检测流程
    if "--pcd" in sys.argv:
        args = _parse_args_detect_mode(sys.argv[1:])
        coords = detect_circles_from_pcd(
            pcd_path=args.pcd,
            x_min=args.x_min, x_max=args.x_max,
            y_min=args.y_min, y_max=args.y_max,
            z_min=args.z_min, z_max=args.z_max,
            circle_radius=args.circle_radius,
            delta_width_circles=args.delta_width_circles,
            delta_height_circles=args.delta_height_circles,
            output_json=args.output,
        )
        sys.exit(0 if coords is not None else 1)

    # 旧模式：positional 参数
    if len(sys.argv) < 2:
        print(
            "用法（旧模式）：python scripts/distance_filter_tool.py <bag|pcd> [output_dir]\n"
            "用法（新模式）：python scripts/distance_filter_tool.py --pcd <file.pcd> "
            "--x_min X --x_max X --y_min Y --y_max Y --z_min Z --z_max Z",
            file=sys.stderr,
        )
        sys.exit(1)

    input_path = sys.argv[1]

    if input_path.lower().endswith(".pcd"):
        output_dir = sys.argv[2] if len(sys.argv) > 2 else None
        result = process_pcd_file(input_path, output_dir=output_dir)
        sys.exit(0 if result else 1)

    bag_file = input_path
    if len(sys.argv) > 2:
        output_dir = sys.argv[2]
    else:
        output_dir = os.getcwd()
        print(f"未指定输出目录，使用当前目录: {output_dir}")

    if not os.path.isfile(bag_file):
        print(f"[ERROR] bag 文件 '{bag_file}' 不存在", file=sys.stderr)
        sys.exit(1)

    if not os.path.isdir(output_dir):
        print(f"[ERROR] 输出目录 '{output_dir}' 不存在", file=sys.stderr)
        sys.exit(1)

    msg_type = detect_lidar_msg_type(bag_file)
    if msg_type is None:
        print("[ERROR] 未检测到支持的雷达消息类型，退出。", file=sys.stderr)
        sys.exit(1)

    # 4) 根据类型做对应的 PCD 转换
    if msg_type == "PointCloud2":
        pcd_path = convert_pointcloud2_bag_to_pcd(
            bag_file=bag_file,
            output_dir=output_dir,
            topic_name="/hesai/pandar",  # 如有不同，可改成 topic 名称
            pcd_name="sensor_PointCloud2_inten_ascii.pcd"
        )
    else:  # "CustomMsg"
        pcd_path = convert_livox_custom_bag_to_pcd(
            bag_file=bag_file,
            output_dir=output_dir,
            topic_name="/livox/lidar",  # 如有不同，可改成 topic 名称
            pcd_name="livox_CustomMsg_inten_ascii.pcd"
        )

    if pcd_path is None:
        print("[ERROR] PCD 生成失败，退出。", file=sys.stderr)
        sys.exit(1)

    # 5) 对刚生成的这个 PCD 做交互式选点 + 范围保存
    select_and_save_points(
        pcd_folder=output_dir,
        target_pcd_name=os.path.basename(pcd_path)
    )
