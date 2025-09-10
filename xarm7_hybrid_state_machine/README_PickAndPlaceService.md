# Pick and Place Service Node

## Overview

The Pick and Place Service Node acts as a service server that provides a high-level interface for performing pick and place operations on robotic parts. It communicates with the `pick_and_place_sm` state machine to execute the actual movements and provides feedback on operation success/failure.

## Features

- **Service-based Interface**: Provides a ROS2 service interface for easy integration
- **Part-specific Operations**: Supports different part types with predefined poses
- **State Machine Integration**: Communicates with the hybrid state machine for execution
- **Completion Monitoring**: Monitors state machine state to determine operation success
- **Collision Management**: Handles collision object removal after successful operations
- **Timeout Protection**: Implements timeout mechanism to prevent hanging operations

## Supported Parts

The service supports the following parts with their respective pick and place operations:

1. **spindle_2** - Pick and place operation for spindle component
2. **pinion_gear** - Pick and place operation for pinion gear
3. **idler_gear** - Pick and place operation for idler gear  
4. **cover_plate** - Pick-only operation for cover plate

## Service Definition

**Service Name**: `/xarm7/pick_and_place_service`

**Service Type**: `xarm_msgs/srv/PickAndPlaceService`

```
# Request
string part_name    # Name of the part to pick and place

---

# Response  
bool success        # True if operation completed successfully
string message      # Status message with details
```

## Architecture

### Communication Flow

1. **Service Request** → Pick and Place Service Node
2. **State Command** → Pick and Place State Machine (via `/xarm7_state_topic`)
3. **State Monitoring** → Service Node monitors state machine progress (via `/xarm7_state_machine_state`)
4. **Service Response** → Client receives success/failure status

### Topics

- **Publisher**: `/xarm7_state_topic` (xarm_msgs/msg/RobotStateAndTargetPose)
- **Subscriber**: `/xarm7_state_machine_state` (std_msgs/msg/UInt8)

## Usage

### Starting the Service

#### Launch File Method (Recommended)
```bash
ros2 launch xarm7_hybrid_state_machine pick_and_place_service.launch.py
```

#### Direct Node Execution
```bash
ros2 run xarm7_hybrid_state_machine pick_and_place_service_standalone
```

### Service Calls

#### Command Line
```bash
# Pick and place spindle_2
ros2 service call /xarm7/pick_and_place_service xarm_msgs/srv/PickAndPlaceService "{part_name: 'spindle_2'}"

# Pick and place pinion gear
ros2 service call /xarm7/pick_and_place_service xarm_msgs/srv/PickAndPlaceService "{part_name: 'pinion_gear'}"

# Pick and place idler gear
ros2 service call /xarm7/pick_and_place_service xarm_msgs/srv/PickAndPlaceService "{part_name: 'idler_gear'}"

# Pick cover plate
ros2 service call /xarm7/pick_and_place_service xarm_msgs/srv/PickAndPlaceService "{part_name: 'cover_plate'}"
```

#### Python Client Example
```python
import rclpy
from rclpy.node import Node
from xarm_msgs.srv import PickAndPlaceService

class PickPlaceClient(Node):
    def __init__(self):
        super().__init__('pick_place_client')
        self.client = self.create_client(PickAndPlaceService, '/xarm7/pick_and_place_service')
    
    def send_request(self, part_name):
        request = PickAndPlaceService.Request()
        request.part_name = part_name
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

# Usage
client = PickPlaceClient()
response = client.send_request('spindle_2')
if response.success:
    print(f"Success: {response.message}")
else:
    print(f"Failed: {response.message}")
```

#### Test Client
A test client is provided for easy testing:
```bash
ros2 run xarm7_hybrid_state_machine pick_and_place_service_test_client spindle_2
```

### C++ Client Example
```cpp
#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/srv/pick_and_place_service.hpp>

class PickPlaceClient : public rclcpp::Node {
private:
    rclcpp::Client<xarm_msgs::srv::PickAndPlaceService>::SharedPtr client_;

public:
    PickPlaceClient() : Node("pick_place_client") {
        client_ = this->create_client<xarm_msgs::srv::PickAndPlaceService>("/xarm7/pick_and_place_service");
    }
    
    bool send_request(const std::string& part_name) {
        auto request = std::make_shared<xarm_msgs::srv::PickAndPlaceService::Request>();
        request->part_name = part_name;
        
        auto future = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(shared_from_this(), future) == 
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            return response->success;
        }
        return false;
    }
};
```

## State Machine Integration

The service integrates with the Pick and Place State Machine by:

1. **Sending Commands**: Publishing to `/xarm7_state_topic` with part-specific state IDs and poses
2. **Monitoring Progress**: Subscribing to `/xarm7_state_machine_state` to track execution
3. **Detecting Completion**: Monitoring for FINAL (4) or ERROR (5) states
4. **Managing Timeouts**: 60-second timeout for operations

### State IDs
- **SPINDLE_2**: 1
- **PINION_GEAR**: 2 
- **IDLER_GEAR**: 3
- **COVER_PLATE**: 4

## Error Handling

The service provides comprehensive error handling:

- **Invalid Part Name**: Returns failure with supported parts list
- **Operation in Progress**: Prevents concurrent operations
- **State Machine Errors**: Detects ERROR state from state machine
- **Timeouts**: 60-second timeout prevents hanging operations
- **Service Unavailable**: Client-side checks for service availability

## Dependencies

### Build Dependencies
- rclcpp
- rclcpp_components  
- xarm_msgs
- std_msgs
- std_srvs
- geometry_msgs
- moveit_ros_planning_interface
- moveit_visual_tools

### Runtime Dependencies
- Pick and Place State Machine (`pick_and_place_sm_node`)
- xarm_msgs package for service definition

## Building

```bash
cd /VMLabs-ws/xarm7_ws/dev_ws
colcon build --packages-select xarm_msgs xarm7_hybrid_state_machine
source install/setup.bash
```

## Troubleshooting

### Service Not Available
- Ensure the service node is running
- Check for proper ROS2 setup and sourcing

### Operation Timeouts
- Check if state machine is running and responsive
- Verify robot is in proper operational state
- Check for collision objects or planning failures

### Invalid Poses
- Verify workspace setup matches predefined poses
- Check for proper coordinate frame setup

## Configuration

Part poses are configured in the source code (`pick_and_place_service.cpp`) and derived from `experiment_002.cpp`. To modify poses, update the `initializePoses()` function and rebuild.

## Integration Notes

This service is designed to work alongside the existing experiment nodes and provides a cleaner, service-based interface for pick and place operations compared to direct topic-based communication with the state machine.
