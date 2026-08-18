# FAST-Calib

## FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second

FAST-Calib is an efficient target-based extrinsic calibration tool for LiDAR-camera systems (eg., [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)). 

**Key highlights include:** 

1. Support solid-state and mechanical LiDAR.
2. No need for any initial extrinsic parameters.
3. Achieve highly accurate calibration results **in just one seconds**.

In short, it makes extrinsic calibration as simple as intrinsic calibration.

**Related paper:** 

[FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second](https://www.arxiv.org/pdf/2507.17210)

📬 For further assistance or inquiries, please feel free to contact Chunran Zheng at zhengcr@connect.hku.hk.

<p align="center">
  <img src="./pics/calib.jpg" width="100%">
  <font color=#a0a0a0 size=2>Left: Example of Mid360 LiDAR calibration. Right: Point cloud colored with the calibrated extrinsics.</font>
</p>

<p align="center">
  <img src="./pics/all_lidar_type.jpg" width="100%">
  <font color=#a0a0a0 size=2>Circular hole extraction supports multiple LiDAR models.</font>
</p>

## 1. Prerequisites
PCL>=1.8, OpenCV>=4.0.

## 2. Run our examples
1. Prepare the static acquisition data in the `calib_data` folder (see [Single-scene Calibration Sample Data](https://drive.google.com/drive/folders/1W87Dx3MUuPhTpCLvaavWqNUJZV03yU6L?usp=drive_link) from Mid360, Avia and Ouster, and [Multi-scene Calibration Sample Data](https://drive.google.com/drive/folders/1g__plgFqp5tsk-TY7Ioh4RXru62AdLmr?usp=drive_link) from Avia):
- rosbag containing point cloud messages
- corresponding image

2. Run the single-scene calibration process:
```bash
roslaunch fast_calib calib.launch
```

3. After completing Step 2 for at least three different scenes, you can perform multi-scene joint calibration:
```bash
roslaunch fast_calib multi_calib.launch
```

## 3. Run on your own sensor suite
1. Customize the calibration target in the image below, with the CAD model available [here](https://drive.google.com/file/d/1hdC8xGCHNP47a-wSLPyjr_tpOeynNFEG/view?usp=sharing).
2. Collect data from three scenes, with placement illustrated below, and record them into the corresponding rosbags.
3. Provide the instrinsic matrix in `qr_params.yaml`.
4. Set distance filter in `qr_params.yaml` for board point cloud (extra points are acceptable).
5. Calibrate now!

💡 **Note:** You can run `scripts/distance_filter_tool.py` to quickly obtain suitable filter parameters from a rosbag or an existing `.pcd` file, for example `python scripts/distance_filter_tool.py /path/to/cloud.pcd`.

### Automatic circle detection from a `.pcd` file

Once you have a `.pcd` file and know the rough distance filter bounds (from the step above), you can directly compute the four circular-hole coordinates of the calibration board in the LiDAR frame without ROS:

```bash
python scripts/distance_filter_tool.py --pcd mid_360.pcd \
    --x_min 2.3 --x_max 4.8 \
    --y_min -0.7 --y_max 3.2 \
    --z_min -0.1 --z_max 1.9
```

Optional arguments (defaults match `config/qr_params.yaml`):

| Argument | Default | Description |
|---|---|---|
| `--circle_radius` | `0.12` | Hole radius (m) |
| `--delta_width_circles` | `0.5` | Horizontal distance between circle centres (m) |
| `--delta_height_circles` | `0.4` | Vertical distance between circle centres (m) |
| `--output` | `<pcd_name>_circles.json` | Path to save the result JSON |

**Expected output (stdout + JSON file):**
```
[Circle] ===== 标定板四个圆孔坐标（LiDAR 坐标系）=====
  圆孔 1: x=3.0001  y=-0.2498  z=-0.1997
  圆孔 2: x=3.0002  y= 0.2503  z= 0.2001
  圆孔 3: x=2.9999  y= 0.2497  z=-0.2003
  圆孔 4: x=3.0000  y=-0.2501  z= 0.1996
[Circle] ============================================
[Circle] 结果已保存到: mid_360_circles.json
```

The JSON file contains the input parameters and the four hole coordinates, suitable for further processing or integration with the calibration pipeline.
<p align="center">
  <img src="./pics/calibration_target.jpg" width="100%">
  <font color=#a0a0a0 size=2>Left: Actual calibration target | Right: Technical drawing with annotated dimensions.</font>
</p>
<p align="center">
  <img src="./pics/multi-scene.jpg" width="100%">
  <font color=#a0a0a0 size=2>Placement of the calibration target for multi-scene data collection: (a) facing forward, (b) oriented to the right, (c) oriented to the left.</font>
</p>

## 4. Appendix
The calibration target design is based on the [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration).

For further details on the algorithm workflow, see [this document](https://github.com/xuankuzcr/FAST-Calib/blob/main/workflow.md).
## 5. Acknowledgments

Special thanks to [Jiaming Xu](https://github.com/Xujiaming1) for his support, [Haotian Li](https://github.com/luo-xue) for the equipment, and the [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration) algorithm.
