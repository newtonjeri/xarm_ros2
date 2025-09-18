# XARM7 Hybrid State Machine

A robust, modal state machine for autonomous pick-and-place operations on the XARM7 robotic arm, featuring intelligent sequence planning, comprehensive error handling, and efficient state-based completion detection.

## 📋 Table of Contents
- [Overview](#overview)
- [Architecture](#architecture)
- [State Machine Design](#state-machine-design)
- [Components](#components)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [API Reference](#api-reference)
- [Error Handling](#error-handling)
- [Troubleshooting](#troubleshooting)
- [Development](#development)

## 🎯 Overview

The XARM7 Hybrid State Machine provides a complete solution for autonomous pick-and-place operations. It combines a sophisticated state machine with a high-level service interface, supporting multiple robot operation modes and providing robust error recovery mechanisms.

### Key Capabilities
- **Autonomous Pick-and-Place**: Full pick-and-place sequences for gear box assembly
- **Modal Operation**: Supports POSITION, SERVOJ, and TEACHING_JOINT modes
- **Intelligent Planning**: Automatic sequence planning based on operation type
- **Differentiated Error Handling**: Automatic recovery for MoveIt2 errors, manual recovery for hardware errors
- **Real-time Monitoring**: State-based completion detection with immediate response
- **Multi-threaded**: Efficient callback group management for concurrent operations

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    XARM7 Hybrid State Machine                  │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────┐    ┌─────────────────────────────────┐ │
│  │  Pick & Place       │    │  State Machine Core             │ │
│  │  Service            │    │                                 │ │
│  │                     │    │  ┌─────┐  ┌─────┐  ┌─────┐     │ │
│  │  • High-level API   │◄──►│  │IDLE │─►│MOVING│─►│PICKING│    │ │
│  │  • Part management  │    │  └─────┘  └─────┘  └─────┘     │ │
│  │  • State monitoring │    │      ▲        │        │       │ │
│  │  • Timeout handling │    │      │        ▼        ▼       │ │
│  └─────────────────────┘    │  ┌─────┐  ┌─────┐  ┌─────┐     │ │
│                              │  │ERROR│◄─│FINAL│◄─│PLACING│    │ │
│                              │  └─────┘  └─────┘  └─────┘     │ │
│                              └─────────────────────────────────┘ │
├─────────────────────────────────────────────────────────────────┤
│                        ROS2 Interface Layer                     │
│  Topics: /xarm7_state_topic, /xarm7_state_machine_state        │
│  Services: /xarm7/pick_and_place_service, /recovery            │
├─────────────────────────────────────────────────────────────────┤
│                         MoveIt Integration                      │
│  • Motion planning    • Collision detection    • Execution     │
├─────────────────────────────────────────────────────────────────┤
│                          XARM7 Hardware                        │
│  • Robot arm control • Gripper control • State feedback       │
└─────────────────────────────────────────────────────────────────┘
```

## 🔄 State Machine Design

### States Overview
```
IDLE (0)     → Ready to accept commands
MOVING (1)   → Executing arm movements
PICKING (2)  → Performing pick operation (gripper + movement)
PLACING (3)  → Performing place operation (gripper + movement)
FINAL (4)    → Operation completed successfully
ERROR (5)    → Error state requiring manual recovery
```

### State Transitions

#### Normal Operation Flow
```
IDLE → MOVING → PICKING → MOVING → PLACING → FINAL → IDLE
```

#### Error Handling Flow
```
Any State → ERROR → [Manual Recovery] → IDLE
```

### Modal Operation Support

#### POSITION/SERVOJ Mode (Full Capability)
- ✅ All states available: IDLE, MOVING, PICKING, PLACING, FINAL, ERROR
- ✅ Complete pick-and-place sequences
- ✅ MoveIt motion planning

#### TEACHING_JOINT Mode (Limited Capability)
- ✅ Available states: IDLE, MOVING, ERROR
- ❌ No picking/placing operations
- ✅ Manual teaching and basic movements

## 🧩 Components

### 1. State Machine Core (`PickAndPlaceStateMachine`)
**File**: `src/pick_and_place_sm.cpp`

**Responsibilities**:
- State transition management
- Sequence planning and execution
- Robot state monitoring
- Error detection and handling
- MoveIt integration

**Key Features**:
- Thread-safe state management with mutex protection
- Modal state validation based on robot mode
- Automatic sequence planning from high-level commands
- Real-time robot state monitoring

### 2. Pick and Place Service (`PickAndPlaceServiceNode`)
**File**: `src/pick_and_place_service.cpp`

**Responsibilities**:
- High-level pick-and-place API
- Part pose management
- Operation monitoring
- State-based completion detection

**Key Features**:
- Predefined poses for gear box parts
- Efficient condition variable-based waiting
- Dynamic timeout based on operation complexity
- Collision object management

### 3. Launch Files
- `pick_and_place_state_machine.launch.py`: Core state machine
- `pick_and_place_service.launch.py`: Service layer
- `state_machine.launch.py`: Combined system

## ✨ Features

### 🎯 Intelligent Sequence Planning
The state machine automatically plans optimal sequences based on:
- **Operation type**: Pick-only vs full pick-and-place
- **Gripper state**: Current open/closed status
- **Pose analysis**: Pick vs place pose detection
- **Robot mode**: Available capabilities

Example sequences:
```cpp
// Simple movement
IDLE → MOVING → FINAL

// Pick operation
IDLE → MOVING → PICKING → FINAL

// Full pick-and-place
IDLE → MOVING → PICKING → MOVING → PLACING → FINAL
```

### 🛡️ Differentiated Error Handling
The state machine implements intelligent error handling that distinguishes between different error types:

#### **MoveIt2 Planning Errors**
- **Automatic Recovery**: Direct ERROR → IDLE transition for task replanning
- **Use Case**: Path planning failures, kinematic constraints, unreachable poses
- **Behavior**: State machine automatically returns to IDLE after 1 second
- **Action**: Send new command or retry with modified poses

#### **Hardware/xarm Errors** 
- **Manual Recovery**: Requires intervention via recovery service or mode switcher
- **Use Case**: Robot error codes, emergency stops, hardware malfunctions
- **Behavior**: State machine stays in ERROR state until manual recovery
- **Action**: Resolve hardware issues, then use recovery service or mode switcher

#### Integration with Mode Switcher
- Switch to MANUAL mode for hands-on hardware error resolution
- Always allows mode switching FROM ERROR state for recovery
- Blocks mode switching during critical operations (PICKING/PLACING)

See [Differentiated Error Handling Documentation](docs/DIFFERENTIATED_ERROR_HANDLING.md) for detailed information.

### 🛡️ Robust Error Recovery
**No Automatic Recovery**: System stays in ERROR state until manual intervention
**Manual Recovery Service**: Comprehensive robot reset including:
- Clear hardware errors via `/xarm/clean_error`
- Set robot mode to POSITION via `/xarm/set_mode`
- Set robot state to READY via `/xarm/set_state`
- Verify all operations before transitioning to IDLE

### ⚡ Efficient State-Based Completion
**Immediate Response**: Operations complete as soon as state machine reaches FINAL/ERROR
**No Polling**: Uses condition variables for instant notification
**Dynamic Timeouts**: 
- Cover plate (pick-only): 20 seconds
- Full pick-and-place: 30 seconds
- Actual completion: Usually 2-15 seconds

### 🔄 Multi-threaded Architecture
**Separate Callback Groups**:
- State monitoring callbacks
- Stop command callbacks  
- Robot state callbacks
- Service callbacks

## 📦 Installation

### Prerequisites
- ROS2 Jazzy
- XARM ROS2 package
- MoveIt2
- Colcon build tools

### Build Instructions
```bash
cd /path/to/your/workspace
colcon build --packages-select xarm7_hybrid_state_machine
source install/setup.bash
```

## 🚀 Usage

### Basic Operation

#### 1. Launch the State Machine
```bash
ros2 launch xarm7_hybrid_state_machine pick_and_place_state_machine.launch.py
```

#### 2. Launch the Service (Optional)
```bash
ros2 launch xarm7_hybrid_state_machine pick_and_place_service.launch.py
```

#### 3. Use Pick and Place Service
```bash
# Pick and place spindle_2
ros2 service call /xarm7/pick_and_place_service xarm_msgs/srv/PickAndPlaceService "{part_name: 'spindle_2'}"

# Pick and place pinion gear
ros2 service call /xarm7/pick_and_place_service xarm_msgs/srv/PickAndPlaceService "{part_name: 'pinion_gear'}"

# Pick cover plate (pick-only operation)
ros2 service call /xarm7/pick_and_place_service xarm_msgs/srv/PickAndPlaceService "{part_name: 'cover_plate'}"
```

#### 4. Direct State Machine Commands
```bash
# Send direct command to state machine
ros2 topic pub /xarm7_state_topic xarm_msgs/msg/RobotStateAndTargetPose "{
  robot_next_state: 1,
  target_pose_1: {
    position: {x: 0.4912, y: 0.0661, z: 0.0650},
    orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
  },
  target_pose_2: {
    position: {x: 0.4612, y: -0.4039, z: 0.15},
    orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
  }
}"
```

### Monitoring

#### Monitor State Machine State
```bash
ros2 topic echo /xarm7_state_machine_state
```

#### Monitor Robot States
```bash
ros2 topic echo /xarm/robot_states
```

## 📚 API Reference

### Services

#### `/xarm7/pick_and_place_service`
**Type**: `xarm_msgs/srv/PickAndPlaceService`

**Request**:
```yaml
string part_name  # "spindle_2", "pinion_gear", "idler_gear", "cover_plate"
```

**Response**:
```yaml
bool success      # Operation success/failure
string message    # Detailed result message
```

#### `/xarm7_state_machine/recover`
**Type**: `std_srvs/srv/Trigger`

**Purpose**: Manual recovery from ERROR state

**Response**:
```yaml
bool success      # Recovery success/failure  
string message    # Detailed recovery status
```

### Topics

#### Published Topics

##### `/xarm7_state_machine_state`
**Type**: `std_msgs/msg/UInt8`
**Description**: Current state machine state (0-5)

##### `/robot_mode`
**Type**: `xarm_msgs/msg/RobotMode`
**Description**: Robot mode commands

#### Subscribed Topics

##### `/xarm7_state_topic`
**Type**: `xarm_msgs/msg/RobotStateAndTargetPose`
**Description**: High-level operation commands

##### `/xarm7_stop_command`
**Type**: `xarm_msgs/msg/StopCommand`
**Description**: Emergency stop commands

##### `/xarm/robot_states`
**Type**: `xarm_msgs/msg/RobotMsg`
**Description**: Real-time robot state feedback

### Predefined Part Poses

#### Spindle_2
```yaml
pick:  {x: 0.3285, y: -0.08823, z: 0.1261}
place: {x: 0.3665, y: -0.3788,  z: 0.15}
```

#### Pinion Gear
```yaml
pick:  {x: 0.4912, y: 0.0661,  z: 0.0650}
place: {x: 0.4612, y: -0.4039, z: 0.15}
```

#### Idler Gear
```yaml
pick:  {x: 0.3357, y: 0.0295,  z: 0.0558}
place: {x: 0.4506, y: -0.3707, z: 0.1407}
```

#### Cover Plate
```yaml
pick:  {x: 0.5052, y: -0.08395, z: 0.06}
place: N/A (pick-only operation)
```

## 🔧 Error Handling

The state machine implements **differentiated error handling** to provide appropriate recovery mechanisms based on error type.

### Error Types

#### **MoveIt2 Planning Errors** (Automatic Recovery)
- **Detection**: Planning failures, kinematic constraint violations
- **Recovery**: Automatic ERROR → IDLE transition after 1 second  
- **Action**: Retry operation or send new commands

#### **Hardware/xarm Errors** (Manual Recovery)
- **Detection**: Robot error codes, emergency stops, hardware malfunctions
- **Recovery**: Manual intervention required via recovery service or mode switcher
- **Action**: Resolve hardware issues first, then recover manually

### Recovery Methods

#### Automatic Recovery (MoveIt2 errors only)
No action required - state machine automatically returns to IDLE

#### Manual Recovery Service
```bash
ros2 service call /xarm7_state_machine/recover std_srvs/srv/Trigger
```

#### Mode Switcher Recovery
1. Switch to MANUAL mode for hands-on troubleshooting
2. Resolve hardware issues manually  
3. Switch back to MOVEIT mode when ready

### Error States and Recovery

#### Automatic Error Detection
- **Hardware Errors**: Detected via `/xarm/robot_states` error codes (→ Manual recovery)
- **Planning Errors**: MoveIt planning failures (→ Automatic recovery)
- **State Validation**: Invalid transitions for current robot mode  
- **Emergency Stop**: Via `/xarm7_stop_command` topic (→ Manual recovery)

#### Manual Recovery Process (Hardware Errors)
1. **Error Occurs**: State machine transitions to ERROR state with HARDWARE_ERROR type
2. **Manual Intervention**: User calls recovery service or uses mode switcher
3. **Comprehensive Reset**: 
   - Clear robot errors
   - Set POSITION mode
   - Set READY state
   - Verify operations
4. **Return to Operation**: Transition back to IDLE state

### Error Prevention
- **State Validation**: Prevents invalid operations in each mode
- **Robot State Monitoring**: Continuous monitoring of robot health
- **Timeout Protection**: Safety timeouts for all operations
- **Thread Safety**: Mutex protection for concurrent access

## 🔍 Troubleshooting

### Common Issues

#### 1. State Machine Not Responding
**Symptoms**: Commands sent but no state changes
**Solutions**:
- Check robot is powered and connected
- Verify robot mode is appropriate for operation
- Check for error codes in `/xarm/robot_states`
- Use recovery service if in ERROR state

#### 2. Service Timeouts
**Symptoms**: Operations timeout even when working
**Diagnosis**: Check state machine state during operation
**Solutions**:
- Verify state machine is running
- Check topic connections
- Monitor robot state feedback

#### 3. Invalid State Transitions
**Symptoms**: Warnings about invalid transitions
**Cause**: Attempting operations not supported in current mode
**Solutions**:
- Switch to POSITION mode for full pick-and-place
- Use only MOVING operations in TEACHING_JOINT mode

#### 4. Robot Error Codes
**Symptoms**: Robot stops responding, error state
**Solutions**:
1. Check robot manual for error code meaning
2. Fix underlying hardware issue
3. Use recovery service
4. Verify robot returns to normal operation

### Diagnostic Commands

#### Check State Machine Status
```bash
ros2 topic echo /xarm7_state_machine_state
```

#### Check Robot Status
```bash
ros2 topic echo /xarm/robot_states
```

#### Check Available Services
```bash
ros2 service list | grep xarm
```

#### Check Node Status
```bash
ros2 node list
ros2 node info /pick_and_place_sm_node
```

## 🔬 Development

### Code Structure
```
xarm7_hybrid_state_machine/
├── include/xarm7_hybrid_state_machine/
│   ├── pick_and_place_sm.hpp          # State machine core
│   ├── pick_and_place_service.hpp     # Service interface
│   └── moveit_include.hpp              # MoveIt integration
├── src/
│   ├── pick_and_place_sm.cpp          # State machine implementation
│   ├── pick_and_place_service.cpp     # Service implementation
│   └── pick_and_place_sm_node.cpp     # Node entry point
├── launch/                            # Launch files
├── docs/                              # Documentation
└── README.md                          # This file
```

### Key Design Patterns

#### State Machine Pattern
- Clear state definitions and transitions
- State validation based on robot capabilities
- Sequence planning for complex operations

#### Observer Pattern
- State monitoring via callbacks
- Real-time robot state observation
- Event-driven state transitions

#### Command Pattern
- High-level operation commands
- Encapsulation of complex sequences
- Undo capability via error recovery

### Extension Points

#### Adding New Parts
1. Define poses in `initializePoses()`
2. Add part ID in `PART_TYPES` enum
3. Update validation in `isValidPartName()`

#### Adding New States
1. Define state in `STATES` enum
2. Add transition validation in `isValidTransition()`
3. Implement state logic in `execute*State()` methods

#### Adding New Robot Modes
1. Define mode in `XarmMode` enum
2. Update modal validation in `isStateValidForCurrentMode()`
3. Add mode-specific behavior

### Testing

#### Unit Testing
```bash
colcon test --packages-select xarm7_hybrid_state_machine
```

#### Integration Testing
1. Launch state machine
2. Send test commands
3. Verify state transitions
4. Test error recovery

## 📝 License

Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.

Software License Agreement (BSD License)

## 👥 Authors

- Newton Kariuki <newtonkaris45@gmail.com>

## 📞 Support

For issues and questions:
1. Check this README and troubleshooting section
2. Review log files for error details
3. Test with recovery service for error states
4. Contact maintainer for additional support

---

**Last Updated**: December 2025  
**Version**: 1.0.0  
**ROS2 Distribution**: Jazzy
