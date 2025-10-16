#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math
import logging
import numpy as np
from pika.sense import Sense
from xarm.wrapper import XArmAPI

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('robot_teleop')


class Transformations:
    @staticmethod
    def quaternion_to_rotation_matrix(q):
        """
        将四元数转换为旋转矩阵
        
        注: 四元素顺序为xyzw
        """
        norm = np.linalg.norm(q)
        if norm < 1e-6:
            raise ValueError('零四元数无法归一化')

        x, y, z, w = q / norm  # 归一化
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z

        R = np.array([
            [1 - 2 * (yy + zz),     2 * (xy - wz),      2 * (xz + wy)],
            [    2 * (xy + wz), 1 - 2 * (xx + zz),      2 * (yz - wx)],
            [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)]
        ])
        return R

    @staticmethod
    def rpy_to_rotation_matrix(roll, pitch, yaw):
        """RPY角到旋转矩阵的转换"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R = np.array([
            [cp*cy,  -cr*sy + sr*sp*cy,    sr*sy + cr*sp*cy],
            [cp*sy,   cr*cy + sr*sp*sy,   -sr*cy + cr*sp*sy],
            [ -sp,        sr*cp,               cr*cp],
        ])

        return R
    
    @staticmethod
    def rotation_matrix_to_rpy(R, yaw_zero=True):
        """
        旋转矩阵到RPY角的转换
        
        yaw_zero: 万向节锁情况下, True就把yaw置0, False就把roll置0
        """
        epsilon = 1e-6
        if abs(R[2, 0]) > 1 - epsilon: # 万向节锁(pitch=±90°)
            pitch = np.arcsin(-R[2, 0])
            roll_yaw = np.arctan2(-R[0, 1], R[1, 1])
            if yaw_zero:
                # 保留roll, 把yaw置0
                roll, yaw = roll_yaw, 0
            else:
                # 保留yaw, 把roll置0
                roll, yaw = 0, roll_yaw
        else:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arcsin(-R[2, 0])
            yaw = np.arctan2(R[1, 0], R[0, 0])

        return roll, pitch, yaw


class UFRobotTeleop(object):
    def __init__(self, robot_ip, pika_to_robot_eef=None, robot_mode=7, gripper_type=0):

        self.pika_to_robot_eef = [0, 0, 0, math.pi, 0, 0] if pika_to_robot_eef is None else pika_to_robot_eef
        self.robot_mode = robot_mode
        self.gripper_type = gripper_type

        self.arm = XArmAPI(robot_ip, is_radian=True)
        
        logger.info('开始获取WM0设备的位姿数据...')

        # 初始化Sense对象
        self.sense = Sense()

        # 连接设备
        logger.info('连接Sense设备...')
        if not self.sense.connect():
            logger.error('连接Sense设备失败')
            return False
        
        # 配置Vive Tracker（可选）
        # sense.set_vive_tracker_config(config_path='path/to/config', lh_config='lighthouse_config')

        logger.info('Pika Sense设备连接成功')
        tracker = self.sense.get_vive_tracker()
        if not tracker:
            logger.error('Vive Tracker初始化失败')
            self.sense.disconnect()
            return

        logger.info('Vive Tracker初始化成功')
        time.sleep(2)

        devices = self.sense.get_tracker_devices()
        if not devices:
            logger.error('未检测到Vive Tracker设备')
            self.sense.disconnect()
            return

        logger.info('检测到Vive Tracker设备: {}'.format(devices))

        self.target_device = None
        for device in devices:
            if device.startswith('WM'):
                self.target_device = device
                break
        
        if self.target_device is None and devices:
            self.target_device = devices[0]
        
        if not self.target_device:
            logger.error('未找到可跟踪的设备')
            self.sense.disconnect()
            return

        logger.info('开始跟踪设备: {}\n'.format(self.target_device))

    @staticmethod
    def xyzq_to_rotation_matrix(x, y, z, q):
        T = np.eye(4)
        T[:3, :3] = Transformations.quaternion_to_rotation_matrix(q)
        T[:3, 3] = [x, y, z]
        return T

    @staticmethod
    def xyzrpy_to_rotation_matrix(x, y, z, roll, pitch, yaw):
        """构造4x4齐次变换矩阵"""
        T = np.eye(4)
        T[:3, :3] = Transformations.rpy_to_rotation_matrix(roll, pitch, yaw)
        T[:3, 3] = [x, y, z]
        return T

    @staticmethod
    def rotation_matrix_to_xyzrpy(rotation_matrix):
        """从4x4齐次变换矩阵到xyzrpy的转换"""
        x, y, z = rotation_matrix[0, 3], rotation_matrix[1, 3], rotation_matrix[2, 3]
        roll, pitch, yaw = Transformations.rotation_matrix_to_rpy(rotation_matrix)
        return [x, y, z, roll, pitch, yaw]

    def pika_pose_to_robot_matrix(self, x, y, z, q, pika_to_robot_matrix):
        # pika位置对应的变换矩阵
        pika_matrix = self.xyzq_to_rotation_matrix(x, y, z, q)
        # pika位置转换到机械臂坐标系后对应的变换矩阵
        robot_matrix = np.dot(pika_matrix, pika_to_robot_matrix)
        return robot_matrix
    
    def pika_robot_matrix_to_robot_pose(self, pika_begin_robot_matrix, pika_end_robot_matrix, robot_base_matrix):
        # 机械臂目标位置对应的变换矩阵
        robot_martix = np.dot(robot_base_matrix, np.dot(np.linalg.inv(pika_begin_robot_matrix), pika_end_robot_matrix))
        return self.rotation_matrix_to_xyzrpy(robot_martix)
    
    def robot_init(self):
        self.arm.clean_error()
        self.arm.clean_warn()
        self.arm.motion_enable(True)
        self.arm.set_mode(self.robot_mode)
        self.arm.set_state(0)
    
    def set_robot_position(self, pose):
        # logger.info('[运动]: {}, {}, {}, {}, {}, {}'.format(*pose))
        if self.robot_mode == 7:
            # mode 7
            return self.arm.set_position(*pose, radius=0, speed=1000, mvacc=5000)
        else:
            # mode 1
            return self.arm.set_servo_cartesian(pose, speed=500, mvacc=300)

    def set_gripper_position(self, distance):
        if self.gripper_type == 1:
            pos = 0 + (distance / 100) * (850 - 0)
            return self.arm.set_gripper_position(pos, wait=False, wait_motion=False)
        elif self.gripper_type == 2:
            pos = 0 + (distance / 100) * (84 - 0)
            return self.arm.set_gripper_g2_position(pos, speed=225, wait=False, wait_motion=False)
        elif self.gripper_type == 3:
            pos = 71 + (distance / 100) * (150 - 71)
            return self.arm.set_bio_gripper_g2_position(pos, speed=4500, wait=False, wait_motion=False)

    def run(self):
        init_state = self.sense.get_command_state()
        curr_state = init_state

        last_gripper_distance = 0

        ctrl_flag = False # 是否开启遥操作
        need_initial = False
        frequency = 100
        sleep_time = 1 / frequency

        self.robot_init()

        self.arm.set_linear_spd_limit_factor(2.0)

        if self.gripper_type == 1 or self.gripper_type == 2:
            self.arm.set_gripper_enable(True)
            self.arm.set_gripper_mode(0)
            self.arm.set_gripper_speed(5000)
        elif self.gripper_type == 3:
            self.arm.set_bio_gripper_enable(True)

        # pika坐标系到机械臂坐标系的变换关系对应的变换矩阵
        pika_to_robot_matrix = self.xyzrpy_to_rotation_matrix(*self.pika_to_robot_eef)
        # 机械臂初始位置对应的变换矩阵
        robot_base_matrix = None
        # pika初始位置转换到机械臂坐标系后对应的变换矩阵
        pika_begin_robot_matrix = None
        # pika目标位置转换到机械臂坐标系后对应的变换矩阵
        pika_end_robot_matrix = None

        while True:
            time.sleep(sleep_time)

            state = self.sense.get_command_state()
            if state != curr_state:
                curr_state = state
                if not ctrl_flag and curr_state != init_state:
                    ctrl_flag = True
                    need_initial = True
                    self.robot_init()
                    logger.info('开始遥操作')
                    time.sleep(1)
                elif ctrl_flag and curr_state == init_state:
                    ctrl_flag = False
                    logger.info('停止遥操作')
                    continue
            
            if ctrl_flag and (not self.arm.connected or self.arm.error_code != 0 or self.arm.state >= 4):
                logger.info('机械臂原因, 遥操作自动停止')
                init_state = state
                curr_state = state
                ctrl_flag = False
                continue
            
            if not ctrl_flag:
                continue

            if self.gripper_type > 0:
                distance  = self.sense.get_gripper_distance()

                if abs(last_gripper_distance - distance) > 2:
                    last_gripper_distance = distance
                    self.set_gripper_position(distance)

            pose = self.sense.get_pose(self.target_device)
            if not pose:
                continue
            x, y, z = pose.position[0] * 1000, pose.position[1] * 1000, pose.position[2] * 1000

            if need_initial:
                need_initial = False
                _, robot_pos = self.arm.get_position()
                logger.info('[初始] 机械臂位置: {}'.format(robot_pos))

                # 机械臂初始位置对应的变换矩阵
                robot_base_matrix = self.xyzrpy_to_rotation_matrix(*robot_pos)

                # pika初始位置转换到机械臂坐标系后对应的变换矩阵
                pika_begin_robot_matrix = self.pika_pose_to_robot_matrix(x, y, z, pose.rotation, pika_to_robot_matrix)
                pika_end_robot_matrix = pika_begin_robot_matrix
            else:
                # pika目标位置转换到机械臂坐标系后对应的变换矩阵
                pika_end_robot_matrix = self.pika_pose_to_robot_matrix(x, y, z, pose.rotation, pika_to_robot_matrix)

            robot_target_pose = self.pika_robot_matrix_to_robot_pose(pika_begin_robot_matrix, pika_end_robot_matrix, robot_base_matrix)

            self.set_robot_position(robot_target_pose)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: {} {{robot_ip}} {{robot_mode}} {{gripper_type}}'.format(sys.argv[0]))
        print('  robot_mode: 1/7')
        print('     1: servo motion mode')
        print('     7: (default) cartesian online trajectory planning mode')
        print('  gripper_type: 0/1/2/3')
        print('     0: (default) no gripper')
        print('     1: xArm Gripper')
        print('     2: xArm Gripper G2')
        print('     3: BIO Gripper G2')
        exit(1)
    robot_ip = sys.argv[1]
    robot_mode = 7 if len(sys.argv) <= 2 else int(sys.argv[2])
    robot_mode = robot_mode if robot_mode in [1, 7] else 7
    # robot_mode==1: Servo模式
    # robot_mode==7: 笛卡尔在线轨迹规划模式
    gripper_type = 0 if len(sys.argv) <= 3 else int(sys.argv[3])
    gripper_type = gripper_type if gripper_type in [0, 1, 2, 3] else 0
    # gripper_type==0: 没有机械爪
    # gripper_type==1: xArm Gripper
    # gripper_type==2: xArm Gripper G2
    # gripper_type==3: BIO Gripper G2
    pika_to_robot_eef = [0, 0, 0, math.pi, -math.pi / 2, 0]
    print('**********************************************************************')
    print('* robot_ip: {}'.format(robot_ip))
    print('* robot_mode: {}'.format(robot_mode))
    print('* gripper_type: {}'.format(gripper_type))
    print('* pika_to_robot: {}'.format(pika_to_robot_eef))
    print('**********************************************************************')
    teleop = UFRobotTeleop(robot_ip, pika_to_robot_eef, robot_mode=robot_mode, gripper_type=gripper_type)
    teleop.run()
