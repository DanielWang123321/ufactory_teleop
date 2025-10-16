# Pika 遥操作系统

使用松灵机器人的Pika Sense (https://global.agilex.ai/products/pika) 进对 UFACTORY(深圳市众为创造科技有限公司) 的机械臂(https://www.ufactory.cc/xarm-collaborative-robot/)的遥操作控制。

## 系统要求

### 操作系统
- **支持**: Ubuntu 22.04
- **不支持**: Windows/Mac OS

### Python 版本
- Python 3.9/3.10

### 硬件要求
- UFACTORY xArm 机械臂(xArm 5/6/7, Lite 6或850)
- Pika Sense

## 安装

### 1. 下载项目

```bash
git clone https://github.com/xArm-Developer/ufactory_teleop
cd ufactory_teleop/pika_teleop
```

### 2. 创建虚拟环境与安装依赖

创建虚拟环境(推荐)
```bash
python3.9 -m venv py39
```
激活虚拟环境
```bash
source py39/bin/activate
```

安装依赖
```bash
pip install -r requirements.txt
```

## 使用说明

### 基本用法

```bash
python uf_robot_pika_teleop.py <robot_ip> [robot_mode] [gripper_type]
```

### 参数说明

#### 必需参数
- `robot_ip`: 机械臂的 IP 地址，例如 192.168.1.200

#### 可选参数

**robot_mode** (默认: 7)
- `1`: Servo 伺服运动模式
- `7`: 笛卡尔在线轨迹规划模式 (推荐)

**gripper_type** (默认: 0)
- `0`: 无夹爪
- `1`: xArm Gripper
- `2`: xArm Gripper G2
- `3`: BIO Gripper G2

### 使用方法

#### 跟踪设备校准

首次使用Pika Sense 或者基站位置变动需要校准。

* 使用survive-cli.py 进行校准

```bash
python survive-cli.py
```

* 使用ROS 指令校准(适用于ROS1/2 开发者)

参考 https://agilexsupport.yuque.com/staff-hso6mo/peoot3/axi8hh9h9t2sh2su

#### 遥操作示例 : 使用笛卡尔在线规划模式，使用机械爪
```bash
python uf_robot_pika_teleop.py 192.168.1.100 7 1
```


