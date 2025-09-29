
# Fire-Fighting Robot

A robotics project designed to autonomously detect and extinguish fires. This repository contains all the necessary code, schematics, and documentation to build and operate a Fire-Fighting Robot, perfect for robotics enthusiasts, students, and hobbyists interested in automation and safety applications.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [How It Works](#how-it-works)
- [Getting Started](#getting-started)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Introduction

The Fire-Fighting Robot is designed to detect the presence of fire using sensors and automatically extinguish it using a water pump or fan mechanism. This project demonstrates the integration of sensors, actuators, and microcontrollers to solve real-world problems.

## Features

- **Fire Detection:** Uses flame sensors to identify fire sources.
- **Obstacle Avoidance:** Ultrasonic sensors prevent collisions.
- **Autonomous Navigation:** Moves towards the fire and avoids obstacles.
- **Extinguishing Mechanism:** Activates a pump or fan to put out the fire.
- **Expandable:** Easily add more sensors or features.

## Hardware Requirements

- Microcontroller (e.g., Arduino Uno)
- Flame sensors
- Ultrasonic sensors
- Servo motors/Continuous DC motors
- Water pump or fan module
- Motor driver (e.g., L298N)
- Chassis, wheels
- Power supply (battery pack)
- Jumper wires, breadboard, etc.

## Software Requirements

- Arduino IDE
- Required Arduino libraries (e.g., Servo, NewPing)
- (Optional) Simulation software like Tinkercad or Proteus

## How It Works

1. **Scanning:** The robot continuously scans its environment using flame and ultrasonic sensors.
2. **Detection:** When a fire is detected, the robot navigates towards it while avoiding obstacles.
3. **Extinguishing:** Upon reaching the fire, it activates the extinguishing mechanism.
4. **Patrolling:** After extinguishing, the robot resumes patrolling.

## Getting Started

1. **Clone the Repository**
   ```bash
   git clone https://github.com/Sai-Pratham05/Fire-Fighting-Robot.git
   ```
2. **Assemble the Hardware**
   - Follow the schematic diagrams provided in the `/docs` or `/images` folder (if available).
3. **Upload the Code**
   - Open the code in Arduino IDE.
   - Connect your microcontroller via USB.
   - Select the correct board and port.
   - Upload the code.

## Usage

- Power on the robot.
- Place it in the area to be monitored.
- The robot will start patrolling and respond automatically if it detects fire.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for suggestions or improvements.

## License

This project is licensed under the [MIT License](LICENSE).

---

Made with 💡 by [Sai-Pratham05](https://github.com/Sai-Pratham05)
````
