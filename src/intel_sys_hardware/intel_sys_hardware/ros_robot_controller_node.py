#!/usr/bin/env python3
# encoding: utf-8
import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import UInt16, Bool

from intel_sys_hardware.ros_robot_controller_sdk import Board
from intel_sys_interfaces.srv import GetBusServoState, GetPWMServoState
from intel_sys_interfaces.msg import (
    ButtonState, BuzzerState, LedState, MotorsState,
    BusServoState, SetBusServoState, ServosPosition,
    SetPWMServoState, PWMServoState, Sbus, OLEDState
)

class RosRobotControllerNode(Node):
    gravity = 9.80665

    def __init__(self):
        super().__init__('ros_robot_controller')
        self.declare_parameter('port', '/dev/rrc')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('motor_ids', [1, 2, 3, 4])

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.IMU_FRAME = self.get_parameter('imu_frame').value
        self.motor_ids = self.get_parameter('motor_ids').value

        self.get_logger().info(f'Connecting to STM32 board on {port} @ {baudrate} baud...')
        try:
            self.board = Board(device=port, baudrate=baudrate)
            self.board.enable_reception(True)
        except Exception as e:
            self.get_logger().error(f'Failed to open serial device {port}: {e}')
            raise e

        self.running = True
        self.reception_enabled = True

        # Callback groups for thread concurrency
        self.sensor_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.sub_cb_group = ReentrantCallbackGroup()

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu_raw', 10)
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.sbus_pub = self.create_publisher(Sbus, 'sbus', 10)
        self.button_pub = self.create_publisher(ButtonState, 'button', 10)
        self.battery_pub = self.create_publisher(UInt16, 'battery', 10)

        # Subscribers
        self.create_subscription(LedState, 'set_led', self.set_led_state, 5, callback_group=self.sub_cb_group)
        self.create_subscription(BuzzerState, 'set_buzzer', self.set_buzzer_state, 5, callback_group=self.sub_cb_group)
        self.create_subscription(OLEDState, 'set_oled', self.set_oled_state, 5, callback_group=self.sub_cb_group)
        self.create_subscription(MotorsState, 'set_motor', self.set_motor_state, 10, callback_group=self.sub_cb_group)
        self.create_subscription(Bool, 'enable_reception', self.enable_reception_cb, 1, callback_group=self.sub_cb_group)

        self.create_subscription(SetBusServoState, 'bus_servo/set_state', self.set_bus_servo_state, 10, callback_group=self.sub_cb_group)
        self.create_subscription(ServosPosition, 'bus_servo/set_position', self.set_bus_servo_position, 10, callback_group=self.sub_cb_group)
        self.create_subscription(SetPWMServoState, 'pwm_servo/set_state', self.set_pwm_servo_state, 10, callback_group=self.sub_cb_group)

        # Services
        self.create_service(GetBusServoState, 'bus_servo/get_state', self.get_bus_servo_state, callback_group=self.service_cb_group)
        self.create_service(GetPWMServoState, 'pwm_servo/get_state', self.get_pwm_servo_state, callback_group=self.service_cb_group)
        self.create_service(Trigger, 'init_finish', self.get_node_state, callback_group=self.service_cb_group)

        # Initial board state
        self.board.pwm_servo_set_offset(1, 0)
        self.board.set_motor_speed([[mid, 0] for mid in self.motor_ids])
        self.clock = self.get_clock()

        # 50Hz Sensor Polling Timer
        self.timer = self.create_timer(0.02, self.pub_loop, callback_group=self.sensor_cb_group)

        self.get_logger().info('STM32 controller bridge initialized and running with MultiThreadedExecutor.')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def pub_loop(self):
        if self.running and self.reception_enabled:
            self.pub_button_data()
            self.pub_joy_data()
            self.pub_imu_data()
            self.pub_sbus_data()
            self.pub_battery_data()

    def enable_reception_cb(self, msg):
        self.reception_enabled = msg.data
        self.board.enable_reception(msg.data)

    def set_led_state(self, msg):
        self.board.set_led(msg.on_time, msg.off_time, msg.repeat, msg.id)

    def set_buzzer_state(self, msg):
        self.board.set_buzzer(msg.freq, msg.on_time, msg.off_time, msg.repeat)

    def set_motor_state(self, msg):
        data = [[i.id, i.rps] for i in msg.data]
        self.board.set_motor_speed(data)

    def set_oled_text(self, msg):
        self.board.set_oled_text(int(msg.index), msg.text)

    def set_pwm_servo_state(self, msg):
        data = []
        for i in msg.state:
            if i.id and i.position:
                data.append([i.id[0], i.position[0]])
            if i.id and i.offset:
                self.board.pwm_servo_set_offset(i.id[0], i.offset[0])
        if data:
            self.board.pwm_servo_set_position(msg.duration, data)

    def get_pwm_servo_state(self, request, response):
        states = []
        for i in request.cmd:
            data = PWMServoState()
            if i.get_position:
                state = self.board.pwm_servo_read_position(i.id)
                if state is not None: data.position = state
            if i.get_offset:
                state = self.board.pwm_servo_read_offset(i.id)
                if state is not None: data.offset = state
            states.append(data)
        response.state = states
        response.success = True
        return response

    def set_bus_servo_position(self, msg):
        data = [[i.id, i.position] for i in msg.position]
        if data:
            self.board.bus_servo_set_position(msg.duration, data)

    def set_bus_servo_state(self, msg):
        data = []
        servo_ids = []
        for i in msg.state:
            if i.present_id and i.present_id[0]:
                sid = i.present_id[1]
                if i.target_id and i.target_id[0]: self.board.bus_servo_set_id(sid, i.target_id[1])
                if i.position and i.position[0]: data.append([sid, i.position[1]])
                if i.offset and i.offset[0]: self.board.bus_servo_set_offset(sid, i.offset[1])
                if i.position_limit and i.position_limit[0]: self.board.bus_servo_set_angle_limit(sid, i.position_limit[1:])
                if i.voltage_limit and i.voltage_limit[0]: self.board.bus_servo_set_vin_limit(sid, i.voltage_limit[1:])
                if i.max_temperature_limit and i.max_temperature_limit[0]: self.board.bus_servo_set_temp_limit(sid, i.max_temperature_limit[1])
                if i.enable_torque and i.enable_torque[0]: self.board.bus_servo_enable_torque(sid, i.enable_torque[1])
                if i.save_offset and i.save_offset[0]: self.board.bus_servo_save_offset(sid)
                if i.stop and i.stop[0]: servo_ids.append(sid)
        if data:
            self.board.bus_servo_set_position(msg.duration, data)
        if servo_ids:
            self.board.bus_servo_stop(servo_ids)

    def get_bus_servo_state(self, request, response):
        states = []
        for i in request.cmd:
            data = BusServoState()
            if i.get_id:
                state = self.board.bus_servo_read_id(i.id)
                if state is not None: data.present_id = state
            if i.get_position:
                state = self.board.bus_servo_read_position(i.id)
                if state is not None: data.position = state
            if i.get_offset:
                state = self.board.bus_servo_read_offset(i.id)
                if state is not None: data.offset = state
            if i.get_voltage:
                state = self.board.bus_servo_read_vin(i.id)
                if state is not None: data.voltage = state
            if i.get_temperature:
                state = self.board.bus_servo_read_temp(i.id)
                if state is not None: data.temperature = state
            if i.get_position_limit:
                state = self.board.bus_servo_read_angle_limit(i.id)
                if state is not None: data.position_limit = state
            if i.get_voltage_limit:
                state = self.board.bus_servo_read_vin_limit(i.id)
                if state is not None: data.voltage_limit = state
            if i.get_max_temperature_limit:
                state = self.board.bus_servo_read_temp_limit(i.id)
                if state is not None: data.max_temperature_limit = state
            if i.get_torque_state:
                state = self.board.bus_servo_read_torque_state(i.id)
                if state is not None: data.enable_torque = [1, int(state)]
            states.append(data)
        response.state = states
        response.success = True
        return response

    def pub_battery_data(self):
        data = self.board.get_battery()
        if data is not None:
            msg = UInt16()
            msg.data = data
            self.battery_pub.publish(msg)

    def pub_button_data(self):
        data = self.board.get_button()
        if data is not None:
            msg = ButtonState()
            msg.id = data[0]
            msg.state = data[1]
            self.button_pub.publish(msg)

    def pub_joy_data(self):
        data = self.board.get_gamepad()
        if data is not None:
            msg = Joy()
            msg.axes = data[0]
            msg.buttons = data[1]
            msg.header.stamp = self.clock.now().to_msg()
            self.joy_pub.publish(msg)

    def pub_sbus_data(self):
        data = self.board.get_sbus()
        if data is not None:
            msg = Sbus()
            msg.channel = data
            msg.header.stamp = self.clock.now().to_msg()
            self.sbus_pub.publish(msg)

    def pub_imu_data(self):
        data = self.board.get_imu()
        if data is not None:
            ax, ay, az, gx, gy, gz = data
            msg = Imu()
            msg.header.frame_id = self.IMU_FRAME
            msg.header.stamp = self.clock.now().to_msg()
            msg.linear_acceleration.x = ax * self.gravity
            msg.linear_acceleration.y = ay * self.gravity
            msg.linear_acceleration.z = az * self.gravity
            msg.angular_velocity.x = math.radians(gx)
            msg.angular_velocity.y = math.radians(gy)
            msg.angular_velocity.z = math.radians(gz)
            msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            msg.linear_acceleration_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.004]
            self.imu_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        try:
            self.board.set_motor_speed([[mid, 0] for mid in self.motor_ids])
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RosRobotControllerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
