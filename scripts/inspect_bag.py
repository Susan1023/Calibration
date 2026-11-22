"""只读检查 sqlite3 rosbag2 第一帧 PointCloud2 的字段、数值和强度。"""
import argparse
import json
import sqlite3
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

p = argparse.ArgumentParser()
p.add_argument('bag_db3')
p.add_argument('--cloud-topic', required=True)
p.add_argument('--pcd', help='可选：把首帧有限 XYZ 点输出为 ASCII PCD（仅用于调试）')
a = p.parse_args()
conn = sqlite3.connect(f'file:{Path(a.bag_db3).resolve()}?mode=ro', uri=True)
row = conn.execute('SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name=? LIMIT 1',
                   (a.cloud_topic,)).fetchone()
if row is None:
    raise SystemExit(f'没有点云话题 {a.cloud_topic}')
msg = deserialize_message(row[0], PointCloud2)
fields = {f.name: {'offset': f.offset, 'datatype': f.datatype} for f in msg.fields}
if not {'x', 'y', 'z'} <= set(fields):
    raise SystemExit('缺少 x/y/z 字段')
requested = ['x', 'y', 'z'] + (['intensity'] if 'intensity' in fields else [])
cloud = point_cloud2.read_points(msg, field_names=requested, skip_nans=False)
xyz = np.column_stack([cloud[n].astype(float) for n in ('x', 'y', 'z')])
valid = np.isfinite(xyz).all(axis=1)
distance = np.linalg.norm(xyz[valid], axis=1)
result = {'topic': a.cloud_topic, 'frame_id': msg.header.frame_id,
          'header_stamp': [msg.header.stamp.sec, msg.header.stamp.nanosec],
          'point_count': len(xyz), 'finite_xyz_count': int(valid.sum()),
          'fields': fields,
          'distance_m': {'min': float(distance.min()), 'max': float(distance.max())} if len(distance) else None}
if 'intensity' in fields:
    intensity = cloud['intensity'].astype(float)
    finite_intensity = np.isfinite(intensity)
    result['intensity'] = {'finite_count': int(finite_intensity.sum()),
                           'zero_count': int((intensity[finite_intensity] == 0).sum()),
                           'min': float(intensity[finite_intensity].min()) if finite_intensity.any() else None,
                           'max': float(intensity[finite_intensity].max()) if finite_intensity.any() else None}
print(json.dumps(result, ensure_ascii=False, indent=2))
if a.pcd:
    points = xyz[valid]
    with open(a.pcd, 'w') as f:
        f.write(f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n')
        np.savetxt(f, points, fmt='%.6f')
