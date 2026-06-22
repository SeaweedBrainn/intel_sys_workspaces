#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/03/24
# @author:aiden
# 智能巡逻(intelligent patrolling)
import math
import time
import rclpy
import threading
from rclpy.node import Node
from app.common import Heart
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from interfaces.srv import SetInt64

class PatrolController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.running_mode = 0
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.rectangle_h = 0.5
        self.rectangle_w = 0.5
        self.triangle_l = 0.5
        self.th = None
        self.last_mode = 0
        self.thread_running = True
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 底盘控制(chassis control)
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)  # 进入玩法(enter the game)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)  # 退出玩法(exit the game)
        self.create_service(SetInt64, '~/set_running', self.set_running_srv_callback)  # 开启玩法(start the game)
        Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包(heartbeat package)
        self.mecanum_pub.publish(Twist())
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def reset_value(self):
        '''
        重置参数(reset parameter)
        :return:
        '''
        self.running_mode = 0
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.th = None
        self.last_mode = 0
        self.thread_running = False

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "patrol enter")
        self.reset_value()
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "patrol exit")
        self.reset_value()
        self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "exit"
        return response

    def set_running_srv_callback(self, request, response):
        '''
        设置模式(set the mode)
        :param req:
        :return:
        '''
        new_running_mode = request.data
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_running" + str(new_running_mode))
        response.success = True
        response.message = "set_running"
        if not 0 <= new_running_mode <= 4:
            response.success = False
            response.message = "Invalid running mode {}".format(new_running_mode)
            self.mecanum_pub.publish(Twist())
        else:
            # 以子线程模式运行，以允许停止(run under child thread mode to allow pause)
            if new_running_mode == 1:
                if self.th is None:
                    self.th = threading.Thread(target=self.rectangle)
                    self.th.start()
                else:
                    if not self.th.is_alive():
                        self.th = threading.Thread(target=self.rectangle)
                        self.th.start()
                    elif self.last_mode == new_running_mode:
                        pass
                    else:
                        self.thread_running = False
                        time.sleep(0.1)
                        self.rectangle()
            elif new_running_mode == 2:
                if self.th is None:
                    self.th = threading.Thread(target=self.triangle)
                    self.th.start()
                else:
                    if not self.th.is_alive():
                        self.th = threading.Thread(target=self.triangle)
                        self.th.start()
                    elif self.last_mode == new_running_mode:
                        pass
                    else:
                        self.thread_running = False
                        time.sleep(0.1)
                        self.triangle()
            elif new_running_mode == 3:
                if self.th is None:
                    self.th = threading.Thread(target=self.circle)
                    self.th.start()
                else:
                    if not self.th.is_alive():
                        self.th = threading.Thread(target=self.circle)
                        self.th.start()
                    elif self.last_mode == new_running_mode:
                        pass
                    else:
                        self.thread_running = False
                        time.sleep(0.1)
                        self.circle()
            elif new_running_mode == 4:
                if self.th is None:
                    self.th = threading.Thread(target=self.parallelogram)
                    self.th.start()
                else:
                    if not self.th.is_alive():
                        self.th = threading.Thread(target=self.parallelogram)
                        self.th.start()
                    elif self.last_mode == new_running_mode:
                        pass
                    else:
                        self.thread_running = False
                        time.sleep(0.1)
                        self.parallelogram()
            elif new_running_mode == 0:
                self.thread_running = False
                self.mecanum_pub.publish(Twist())
            self.running_mode = new_running_mode
            self.last_mode = self.running_mode
        return response

    def rectangle(self):
        # 走矩形
        status = 0
        t_start = time.time()
        self.thread_running = True
        while self.thread_running:
            current_time = time.time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_h / self.linear_speed
            elif status == 1 and t_start < current_time:
                status = 2
                twist = Twist()
                twist.linear.y = -self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_w / self.linear_speed
            elif status == 2 and t_start < current_time:
                status = 3
                twist = Twist()
                twist.linear.x = -self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_h / self.linear_speed
            elif status == 3 and t_start < current_time:
                status = 4
                twist = Twist()
                twist.linear.y = self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_w / self.linear_speed
            elif status == 4 and t_start < current_time:
                break

        self.mecanum_pub.publish(Twist())

    def parallelogram(self):
        # 走平行四边形(patrol along parallelogram)
        status = 0
        t_start = time.time()
        self.thread_running = True
        while self.thread_running:
            current_time = time.time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = Twist()
                twist.linear.x = self.linear_speed * math.cos(math.pi / 6)
                twist.linear.y = -self.linear_speed * math.sin(math.pi / 6)
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_w / self.linear_speed
            elif status == 1 and t_start < current_time:
                status = 2
                twist = Twist()
                twist.linear.y = -self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_h / self.linear_speed
            elif status == 2 and t_start < current_time:
                status = 3
                twist = Twist()
                twist.linear.x = -self.linear_speed * math.cos(math.pi / 6)
                twist.linear.y = self.linear_speed * math.sin(math.pi / 6)
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_w / self.linear_speed
            elif status == 3 and t_start < current_time:
                status = 4
                twist = Twist()
                twist.linear.y = self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_h / self.linear_speed
            elif status == 4 and t_start < current_time:
                break

        self.mecanum_pub.publish(Twist())

    def triangle(self):
        # 走三角 (patrol along triangle)
        status = 0
        t_start = time.time()
        self.thread_running = True
        while self.thread_running:
            current_time = time.time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = Twist()
                twist.linear.x = self.linear_speed * math.cos(math.pi / 6)
                twist.linear.y = -self.linear_speed * math.sin(math.pi / 6)
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.triangle_l / self.linear_speed
            elif status == 1 and t_start < current_time:
                status = 2
                twist = Twist()
                twist.linear.x = -self.linear_speed * math.cos(math.pi / 6)
                twist.linear.y = -self.linear_speed * math.sin(math.pi / 6)
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.triangle_l / self.linear_speed
            elif status == 2 and t_start < current_time:
                status = 3
                twist = Twist()
                twist.linear.y = self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.triangle_l / self.linear_speed
            elif status == 3 and t_start < current_time:
                break

        self.mecanum_pub.publish(Twist())

    def circle(self):
        # 走圆形(patrol along circle)
        status = 0
        t_start = time.time()
        self.thread_running = True
        while self.thread_running:
            current_time = time.time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = -0.5
                self.mecanum_pub.publish(twist)
                t_start = current_time + 2 * math.pi / 0.5
            elif status == 1 and t_start < current_time:
                break

        self.mecanum_pub.publish(Twist())

def main():
    node = PatrolController('patrol_app')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
