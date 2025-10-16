# Pika Teleoperation System

Use Agilex Robotics' Pika Sense (https://global.agilex.ai/products/pika) for teleoperation control of UFACTORY's robotic arms (https://www.ufactory.cc/xarm-collaborative-robot/).

## System Requirements

### Operating System
- **Supported**: Ubuntu 22.04
- **Not supported**: Windows/Mac OS

### Python Version
- Python 3.9/3.10

### Hardware Requirements
- UFACTORY xArm robotic arm (xArm 5/6/7, Lite 6 or 850)
- Pika Sense

## Installation

### 1. Download the Project

```bash
git clone https://github.com/xArm-Developer/ufactory_teleop
cd ufactory_teleop/pika_teleop
```

### 2. Create Virtual Environment and Install Dependencies

Create virtual environment (recommended)
```bash
python3.9 -m venv py39
```
Activate virtual environment
```bash
source py39/bin/activate
```

Install dependencies
```bash
pip install -r requirements.txt
```

## Usage

### Basic Usage

```bash
python uf_robot_pika_teleop.py <robot_ip> [robot_mode] [gripper_type]
```

### Parameter Description

#### Required Parameters
- `robot_ip`: IP address of the robotic arm, e.g., 192.168.1.200

#### Optional Parameters

**robot_mode** (default: 7)
- `1`: Servo motion mode
- `7`: Cartesian online trajectory planning mode (recommended)

**gripper_type** (default: 0)
- `0`: No gripper
- `1`: xArm Gripper
- `2`: xArm Gripper G2
- `3`: BIO Gripper G2

### How to Use

#### Tracking Device Calibration

First-time use of Pika Sense or when the base station position changes requires calibration.

* Use survive-cli.py for calibration

```bash
python survive-cli.py
```

* Use ROS commands for calibration (for ROS1/2 developers)

Reference: https://agilexsupport.yuque.com/staff-hso6mo/peoot3/axi8hh9h9t2sh2su

#### Teleoperation Example: Using Cartesian online planning mode with gripper
```bash
python uf_robot_pika_teleop.py 192.168.1.100 7 1
```