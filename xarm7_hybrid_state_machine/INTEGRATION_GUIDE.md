/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Integration Guide: Pick and Place Service with Experiment 002
 *
 * This document explains how the Pick and Place Service integrates with
 * the existing experiment_002.cpp functionality and the state machine.
 ============================================================================*/

# Integration with Experiment 002

## Task Sequence Mapping

The Pick and Place Service provides individual part operations that correspond 
to the sequence used in experiment_002.cpp:

### Original Experiment 002 Sequence (executeTaskBasedOnId):
1. **task_id = 1**: pickAndPlaceSpindle2()
2. **task_id = 2**: pickAndPlaceIdlerGear() 
3. **task_id = 3**: pickAndPlacePinionGear()
4. **task_id = 4**: pickCover()

### Service-Based Approach:
Instead of sequential execution, each operation can be called independently:

```bash
# Individual operations (equivalent to experiment_002 tasks)
ros2 service call /xarm7/pick_and_place_service xarm_msgs/srv/PickAndPlaceService "{part_name: 'spindle_2'}"    # task_id=1
ros2 service call /xarm7/pick_and_place_service xarm_msgs/srv/PickAndPlaceService "{part_name: 'idler_gear'}"   # task_id=2  
ros2 service call /xarm7/pick_and_place_service xarm_msgs/srv/PickAndPlaceService "{part_name: 'pinion_gear'}"  # task_id=3
ros2 service call /xarm7/pick_and_place_service xarm_msgs/srv/PickAndPlaceService "{part_name: 'cover_plate'}"  # task_id=4
```

## Key Differences from Experiment 002

### Experiment 002 Node:
- ✅ Sequential execution based on `is_next` topic
- ✅ Direct MoveIt interface usage
- ✅ Hardcoded task sequence (1→2→3→4)
- ❌ No service interface
- ❌ No individual task control

### Pick and Place Service Node:
- ✅ Service-based interface (synchronous)
- ✅ Individual part control
- ✅ State machine integration
- ✅ Completion monitoring
- ✅ Error handling and timeouts
- ❌ Requires state machine to be running

## Communication Architecture

```
Client → Pick&Place Service → State Machine → MoveIt → Robot
   ↑          ↓                      ↓
   └─── Response ←──── State Monitor ←┘
```

## Collision Object Management

Both implementations handle collision objects but differently:

### Experiment 002:
```cpp
// Remove after successful operation
psi.removeCollisionObjects({"spindle_2"});
```

### Pick and Place Service:
```cpp  
// Remove after successful operation (in service callback)
psi_.removeCollisionObjects({request->part_name});
```

## Pose Compatibility

The service uses identical poses from experiment_002.cpp:
- **spindle_pick_pose** / **spindle_place_pose**
- **pinion_pick_pose** / **pinion_place_pose**  
- **idler_pick_pose** / **idler_place_pose**
- **cover_pick_pose** (pick-only)

## Migration Path

To migrate from experiment_002 to the service-based approach:

1. **Keep experiment_002** for sequential demonstrations
2. **Use Pick and Place Service** for:
   - Individual part operations
   - External system integration  
   - Programmatic control
   - Error recovery scenarios

## Complementary Usage

Both can coexist in the system:
- **experiment_002.cpp**: Demonstrates full assembly sequence
- **Pick and Place Service**: Provides flexible, service-based access

This allows for both demonstration purposes (experiment_002) and practical integration (service) scenarios.
