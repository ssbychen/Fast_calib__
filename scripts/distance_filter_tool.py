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

依赖：
    - rosbag
    - sensor_msgs.point_cloud2
    - open3d, numpy

用法示例：
    python scripts/distance_filter_tool.py /path/to/data.bag /path/to/output_dir
    python scripts/distance_filter_tool.py /path/to/cloud.pcd
    python scripts/distance_filter_tool.py /path/to/cloud.pcd /path/to/output_dir
"""

import os
import sys
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

# ===================== main =====================

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法：python scripts/distance_filter_tool.py <bag|pcd> [output_dir]", file=sys.stderr)
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
