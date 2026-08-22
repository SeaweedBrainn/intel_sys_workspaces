#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np

# Map ROS PointField datatypes to NumPy dtypes
DTYPE_MAP = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


def quat_to_rot_matrix(qx, qy, qz, qw):
    """Converts a quaternion to a 3x3 rotation matrix."""
    return np.array([
        [1.0 - 2.0 * (qy**2 + qz**2), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx**2 + qz**2), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx**2 + qy**2)]
    ])


def rot_matrix_to_quat(R):
    """Converts a 3x3 rotation matrix to a quaternion [x, y, z, w]."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    q = np.array([qx, qy, qz, qw])
    return q / np.linalg.norm(q)


def quat_multiply(q1, q2):
    """Multiplies two quaternions [x, y, z, w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    ])


class PointLioFrameAdapter(Node):
    """
    Standardizes Point-LIO legacy output frames (camera_init / body) into standard ROS REP-105 frames.
    Directly broadcasts the dynamic odom -> base_footprint TF transform at 250Hz with 0ms latency,
    eliminating redundant EKF filtering and lag.
    """
    def __init__(self):
        super().__init__('point_lio_frame_adapter')

        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('child_frame', 'lidar_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('raw_odom_topic', '/point_lio/raw_odom')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('raw_cloud_topic', '/point_lio/raw_cloud_registered')
        self.declare_parameter('cloud_topic', '/point_lio/cloud_registered')
        self.declare_parameter('raw_path_topic', '/point_lio/raw_path')
        self.declare_parameter('path_topic', '/point_lio/path')

        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        raw_odom_topic = self.get_parameter('raw_odom_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        raw_cloud_topic = self.get_parameter('raw_cloud_topic').get_parameter_value().string_value
        cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        raw_path_topic = self.get_parameter('raw_path_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value

        # TF2 listener & broadcasters
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Dynamic mounting transform (base_footprint -> lidar_link)
        self.mount_T = np.zeros(3)
        self.mount_R = np.eye(3)
        self.mount_Q = np.array([0.0, 0.0, 0.0, 1.0])
        self.tf_calibrated = False

        # Timer to lookup dynamic mount transform from URDF
        self.tf_timer = self.create_timer(0.2, self.lookup_mount_transform)

        # QoS Profiles
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.durability = DurabilityPolicy.VOLATILE

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, odom_topic, reliable_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, cloud_topic, reliable_qos)
        self.cloud_alias_pub = self.create_publisher(PointCloud2, '/cloud_registered', reliable_qos)
        self.path_pub = self.create_publisher(Path, path_topic, reliable_qos)
        self.path_alias_pub = self.create_publisher(Path, '/path', reliable_qos)

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, raw_odom_topic, self.odom_callback, reliable_qos)
        self.cloud_sub = self.create_subscription(PointCloud2, raw_cloud_topic, self.cloud_callback, sensor_qos)
        self.path_sub = self.create_subscription(Path, raw_path_topic, self.path_callback, reliable_qos)

        self.get_logger().info(
            f'Point-LIO Frame Adapter active: Direct Odometry & TF publisher [{self.odom_frame} -> {self.base_frame}]'
        )

    def lookup_mount_transform(self):
        """Dynamically fetches the 6-DOF mounting pose of the sensor from the live URDF TF tree."""
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.child_frame,
                Time()
            )
            t = tf_stamped.transform.translation
            r = tf_stamped.transform.rotation

            self.mount_T = np.array([t.x, t.y, t.z])
            self.mount_Q = np.array([r.x, r.y, r.z, r.w])
            self.mount_R = quat_to_rot_matrix(r.x, r.y, r.z, r.w)
            self.tf_calibrated = True

            # Publish dynamic static transform: odom -> camera_init
            static_tf = TransformStamped()
            static_tf.header.stamp = self.get_clock().now().to_msg()
            static_tf.header.frame_id = self.odom_frame
            static_tf.child_frame_id = 'camera_init'
            static_tf.transform.translation.x = t.x
            static_tf.transform.translation.y = t.y
            static_tf.transform.translation.z = t.z
            static_tf.transform.rotation = r
            self.static_tf_broadcaster.sendTransform(static_tf)

            self.get_logger().info(
                f'Dynamic URDF Calibration Locked: [{self.base_frame} -> {self.child_frame}] '
                f'Translation={self.mount_T.round(4)}, Quat={self.mount_Q.round(4)}. '
                f'Direct TF mode active: [{self.odom_frame} -> {self.base_frame}]'
            )
            # Stop periodic timer once transform is acquired
            self.tf_timer.cancel()
        except Exception:
            pass

    def odom_callback(self, msg: Odometry):
        if not self.tf_calibrated:
            return

        # 1. Raw Point-LIO sensor pose in camera_init
        p_raw = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        q_raw = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        # 2. Sensor pose in odom frame (T_odom_sensor = T_odom_caminit * T_caminit_sensor)
        p_sensor_odom = self.mount_R @ p_raw + self.mount_T
        q_sensor_odom = quat_multiply(self.mount_Q, q_raw)
        R_sensor_odom = quat_to_rot_matrix(q_sensor_odom[0], q_sensor_odom[1], q_sensor_odom[2], q_sensor_odom[3])

        # 3. Base_footprint pose in odom frame (T_odom_base = T_odom_sensor * T_sensor_base)
        R_base_odom = R_sensor_odom @ self.mount_R.T
        p_base_odom = p_sensor_odom - R_base_odom @ self.mount_T
        q_base_odom = rot_matrix_to_quat(R_base_odom)

        # 4. Construct standard REP-105 Odometry message
        out_odom = Odometry()
        out_odom.header.stamp = msg.header.stamp
        out_odom.header.frame_id = self.odom_frame
        out_odom.child_frame_id = self.base_frame

        out_odom.pose.pose.position.x = float(p_base_odom[0])
        out_odom.pose.pose.position.y = float(p_base_odom[1])
        out_odom.pose.pose.position.z = float(p_base_odom[2])
        out_odom.pose.pose.orientation.x = float(q_base_odom[0])
        out_odom.pose.pose.orientation.y = float(q_base_odom[1])
        out_odom.pose.pose.orientation.z = float(q_base_odom[2])
        out_odom.pose.pose.orientation.w = float(q_base_odom[3])

        # Preserve linear and angular velocities
        out_odom.twist.twist = msg.twist.twist

        self.odom_pub.publish(out_odom)

        # 5. Broadcast direct dynamic TF (odom -> base_footprint) at full 250Hz speed
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = float(p_base_odom[0])
            tf_msg.transform.translation.y = float(p_base_odom[1])
            tf_msg.transform.translation.z = float(p_base_odom[2])
            tf_msg.transform.rotation.x = float(q_base_odom[0])
            tf_msg.transform.rotation.y = float(q_base_odom[1])
            tf_msg.transform.rotation.z = float(q_base_odom[2])
            tf_msg.transform.rotation.w = float(q_base_odom[3])
            self.tf_broadcaster.sendTransform(tf_msg)

    def cloud_callback(self, msg: PointCloud2):
        # Transform points from camera_init into odom using dynamic 6-DOF mount pose
        if self.tf_calibrated and len(msg.data) > 0:
            try:
                # Build numpy dtype from message fields
                fields_dict = {f.name: (DTYPE_MAP[f.datatype], f.offset) for f in msg.fields if f.datatype in DTYPE_MAP}
                if 'x' in fields_dict and 'y' in fields_dict and 'z' in fields_dict:
                    itemsize = msg.point_step
                    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, itemsize)
                    
                    x = np.frombuffer(arr[:, fields_dict['x'][1]:fields_dict['x'][1]+4].tobytes(), dtype=np.float32)
                    y = np.frombuffer(arr[:, fields_dict['y'][1]:fields_dict['y'][1]+4].tobytes(), dtype=np.float32)
                    z = np.frombuffer(arr[:, fields_dict['z'][1]:fields_dict['z'][1]+4].tobytes(), dtype=np.float32)
                    
                    xyz = np.column_stack([x, y, z])
                    xyz_trans = (self.mount_R @ xyz.T).T + self.mount_T

                    # Overwrite transformed xyz back into data buffer
                    arr_copy = arr.copy()
                    arr_copy[:, fields_dict['x'][1]:fields_dict['x'][1]+4] = np.frombuffer(xyz_trans[:, 0].astype(np.float32).tobytes(), dtype=np.uint8).reshape(-1, 4)
                    arr_copy[:, fields_dict['y'][1]:fields_dict['y'][1]+4] = np.frombuffer(xyz_trans[:, 1].astype(np.float32).tobytes(), dtype=np.uint8).reshape(-1, 4)
                    arr_copy[:, fields_dict['z'][1]:fields_dict['z'][1]+4] = np.frombuffer(xyz_trans[:, 2].astype(np.float32).tobytes(), dtype=np.uint8).reshape(-1, 4)
                    msg.data = arr_copy.tobytes()
            except Exception:
                pass

        msg.header.frame_id = self.odom_frame
        self.cloud_pub.publish(msg)
        self.cloud_alias_pub.publish(msg)

    def path_callback(self, msg: Path):
        msg.header.frame_id = self.odom_frame
        for pose in msg.poses:
            pose.header.frame_id = self.odom_frame
            if self.tf_calibrated:
                p = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
                p_trans = self.mount_R @ p + self.mount_T
                pose.pose.position.x = float(p_trans[0])
                pose.pose.position.y = float(p_trans[1])
                pose.pose.position.z = float(p_trans[2])

                q_orig = np.array([
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w
                ])
                q_trans = quat_multiply(self.mount_Q, q_orig)
                pose.pose.orientation.x = float(q_trans[0])
                pose.pose.orientation.y = float(q_trans[1])
                pose.pose.orientation.z = float(q_trans[2])
                pose.pose.orientation.w = float(q_trans[3])

        self.path_pub.publish(msg)
        self.path_alias_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointLioFrameAdapter()
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
