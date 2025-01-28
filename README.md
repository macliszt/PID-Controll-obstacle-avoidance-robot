# PID Controlled Obstacle Avoidance Robot

This repository contains the code and resources for a **PID Controlled Obstacle Avoidance Robot**. The robot is designed to navigate autonomously while avoiding obstacles using Proportional-Integral-Derivative (PID) control for smooth and efficient motion. It integrates sensors, actuators, and control algorithms to achieve real-time obstacle detection and avoidance.

## Features

- **PID Control Algorithm**: Ensures precise and stable motion control.
- **Ultrasonic Sensors**: Detect obstacles and measure distances in real time.
- **Autonomous Navigation**: Robot can independently navigate around obstacles.
- **Smooth Motion Control**: Reduces sudden movements using PID tuning.
- **Arduino-Based**: Code implemented using Arduino for easy customization.
- **Expandable Design**: Easily adaptable to include additional sensors or features.

## Components Used

- **Microcontroller**: Arduino Uno/Nano/Mega
- **Distance Sensors**: Ultrasonic sensors (e.g., HC-SR04)
- **Motors**: DC motors with encoders (optional for PID tuning)
- **Motor Driver**: L293D or L298N motor driver
- **Power Supply**: Battery pack
- **Chassis**: Robot base with wheels
- **Miscellaneous**: Jumper wires, breadboard, and connectors

## How It Works

1. **Sensor Input**: Ultrasonic sensors detect obstacles by measuring distance.
2. **PID Algorithm**: The microcontroller uses the PID control algorithm to calculate the error between the desired distance and the actual distance to an obstacle.
3. **Motor Control**: The calculated PID output adjusts the motor speed and direction to steer the robot away from obstacles.
4. **Navigation**: The robot continuously scans for obstacles and avoids collisions while maintaining smooth movement.

## Installation and Usage

1. Clone this repository:
   ```bash
   git clone https://github.com/your-username/pid-controlled-obstacle-avoidance-robot.git
   cd pid-controlled-obstacle-avoidance-robot
