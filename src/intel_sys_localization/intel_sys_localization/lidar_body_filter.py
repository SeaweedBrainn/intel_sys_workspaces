#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import numpy as np


# ROS 2 PointField datatype to numpy dtype mapping
TYPE_MAP = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}


class LidarBodyFilter(Node):
    """
    Filters out LiDAR points within the robot's physical bounding box (chassis, wheels, camera).
    Preserves all PointCloud2 fields (x, y, z, intensity, time, ring) and runs at native C speed.
    """
    def __init__(self):
        super().__init__('lidar_body_filter')

        # Declare bounding box parameters (in lidar_link frame)
        self.declare_parameter('input_topic', '/lidar/points_raw')
        self.declare_parameter('output_topic', '/lidar/points')
        self.declare_parameter('min_x', -0.30)
        self.declare_parameter('max_x', 0.08)
        self.declare_parameter('min_y', -0.18)
        self.declare_parameter('max_y', 0.18)
        self.declare_parameter('min_z', -0.20)
        self.declare_parameter('max_z', 0.12)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.min_x = float(self.get_parameter('min_x').get_parameter_value().double_value)
        self.max_x = float(self.get_parameter('max_x').get_parameter_value().double_value)
        self.min_y = float(self.get_parameter('min_y').get_parameter_value().double_value)
        self.max_y = float(self.get_parameter('max_y').get_parameter_value().double_value)
        self.min_z = float(self.get_parameter('min_z').get_parameter_value().double_value)
        self.max_z = float(self.get_parameter('max_z').get_parameter_value().double_value)

        # QoS Profiles
        sub_qos = QoSProfile(depth=10)
        sub_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sub_qos.durability = DurabilityPolicy.VOLATILE

        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.durability = DurabilityPolicy.VOLATILE

        self.pub_ = self.create_publisher(PointCloud2, output_topic, pub_qos)
        self.sub_ = self.create_subscription(PointCloud2, input_topic, self.cloud_callback, sub_qos)

        self.get_logger().info(
            f'LiDAR Body Filter active: {input_topic} -> {output_topic} '
            f'(Box: X[{self.min_x:.2f}, {self.max_x:.2f}], '
            f'Y[{self.min_y:.2f}, {self.max_y:.2f}], '
            f'Z[{self.min_z:.2f}, {self.max_z:.2f}])'
        )

    def build_numpy_dtype(self, fields, point_step):
        offset = 0
        dtype_list = []
        for f in fields:
            if f.offset > offset:
                pad_size = f.offset - offset
                dtype_list.append((f'_pad_{offset}', np.uint8, (pad_size,)))
                offset = f.offset
            np_type = TYPE_MAP.get(f.datatype, np.float32)
            dtype_list.append((f.name, np_type))
            offset += np.dtype(np_type).itemsize

        if point_step > offset:
            pad_size = point_step - offset
            dtype_list.append(('_pad_end', np.uint8, (pad_size,)))

        return np.dtype(dtype_list)

    def cloud_callback(self, msg: PointCloud2):
        if not msg.data or msg.width == 0:
            return

        try:
            dtype = self.build_numpy_dtype(msg.fields, msg.point_step)
            points = np.frombuffer(msg.data, dtype=dtype)
        except Exception as e:
            self.get_logger().error(f'Failed to parse point cloud data: {e}')
            return

        if len(points) == 0:
            return

        x = points['x']
        y = points['y']
        z = points['z']

        # Filter out NaN/inf and points inside the robot bounding box
        valid_mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        inside_box = (
            (x >= self.min_x) & (x <= self.max_x) &
            (y >= self.min_y) & (y <= self.max_y) &
            (z >= self.min_z) & (z <= self.max_z)
        )
        keep_mask = valid_mask & (~inside_box)

        filtered = points[keep_mask]
        num_pts = len(filtered)
        if num_pts == 0:
            return

        # Fast direct byte array serialization
        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.height = 1
        out_msg.width = num_pts
        out_msg.fields = msg.fields
        out_msg.is_bigendian = msg.is_bigendian
        out_msg.point_step = msg.point_step
        out_msg.row_step = msg.point_step * num_pts
        out_msg.data = filtered.tobytes()
        out_msg.is_dense = True

        self.pub_.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarBodyFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
