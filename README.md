# Group 7 - Term 3 Robotics Project

## Overview

This repository contains the complete firmware and control system for a four-wheeled autonomous robot designed to navigate a complex obstacle course. The robot employs intelligent sensor-based path planning and motor control to overcome challenges including wall following, tunnel navigation, stair climbing, and gap crossing.

## Project Objectives

- Develop a robust autonomous navigation system for multi-terrain obstacles
- Implement real-time sensor processing and decision-making logic
- Create a modular, maintainable codebase with multiple operational modes
- Ensure reliable safety mechanisms through fail-safe controls

## Hardware Specifications

### Platform & Architecture
- **Microcontroller**: Arduino Giga
- **Chassis**: Hybrid construction (3D-printed components + laser-cut pieces)

### Actuators
- **Motors**: 4 DC motors (wheel drive system)
- **Servos**: 2 servo motors (auxiliary mechanisms)

### Sensors
- Line-following sensors (obstacle detection and path guidance)
- Distance sensors (proximity and wall detection)

## Software Architecture

### Control System
The robot operates on a multi-mode control system managed through two button interfaces:

1. **Kill Switch**: Safety mechanism that must be engaged to initialize robot operation
2. **Mode Selector**: Rotates through operational modes in sequence (A → B → C → A)

### Operational Modes

| Mode | Function |
|------|----------|
| **Line Following Mode (A)** | Uses line sensors to navigate along predetermined paths |
| **Wall Following Mode (B)** | Maintains proximity to walls for corridor navigation |
| **Lava Pit Mode (C)** | Specialized gap-crossing and obstacle avoidance protocol |

### Execution Flow

```
1. Check kill switch status
2. Read mode selector input
3. Execute corresponding control routine:
   - Collect sensor data
   - Process inputs
   - Apply motor outputs
4. Loop
```

## Repository Structure

```
Group-7-Term-3-Code/
├── README.md
├── firmware/
│   ├── main_controller.ino
│   ├── modes/
│   │   ├── line_following.ino
│   │   ├── wall_following.ino
│   │   └── lava_pit.ino
│   └── sensors/
│       ├── line_sensor.ino
│       └── distance_sensor.ino
└── docs/
    └── hardware_schematic.pdf
```

## Getting Started

### Prerequisites
- Arduino IDE (compatible with Arduino Giga)
- Required libraries: [List any specific libraries used]

### Installation
1. Clone this repository
2. Open the main sketch in Arduino IDE
3. Select Arduino Giga as target board
4. Upload firmware to the microcontroller

### Operation
1. Power on the robot
2. Press the kill switch to enable the system
3. Press the mode selector button to choose operation mode
4. Robot will execute the selected mode autonomously

## Safety Considerations

- Always engage the kill switch before powering down or during emergencies
- Ensure obstacle course is clear before robot operation
- Maintain visual supervision during testing
- Check battery levels before extended operation

## Authors & Contributors

- **Group 7** - Term 3 Robotics Team

## License

[Specify your license here, e.g., MIT, GPL, etc.]

## Contact & Support

For questions or issues, please refer to the repository issues section or contact the development team.

---

**Last Updated**: February 2026