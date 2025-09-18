# Differentiated Error Handling in xarm7 Hybrid State Machine

## Overview

The xarm7 hybrid state machine now implements differentiated error handling to distinguish between two types of errors:

1. **MoveIt2 Planning Failures** - Allow automatic recovery with task replanning
2. **Hardware/xarm Error Codes** - Require manual recovery via service call or mode switcher

## Error Types

### ErrorType::MOVEIT_PLANNING
- **Cause**: MoveIt2 planning failures (e.g., no valid path found, kinematic constraints violated)
- **Behavior**: Automatic transition from ERROR → IDLE state for task replanning
- **Recovery**: Automatic - state machine will return to IDLE state after 1 second
- **Usage**: Send the same command again or modify target poses and retry

### ErrorType::HARDWARE_ERROR  
- **Cause**: xarm hardware errors, stop commands, robot state errors
- **Behavior**: Requires manual intervention for recovery
- **Recovery**: Manual via recovery service or mode switcher
- **Usage**: Resolve hardware issues first, then recover manually

## State Transition Rules

### Normal Transitions
```
IDLE → MOVING → PICKING/PLACING → FINAL → IDLE
```

### Error Transitions

#### MoveIt2 Planning Errors
```
ANY_STATE → ERROR (MOVEIT_PLANNING) → IDLE (automatic after 1s)
```

#### Hardware Errors  
```
ANY_STATE → ERROR (HARDWARE_ERROR) → [stays in ERROR until manual recovery]
```

## Integration with Mode Switcher

The differentiated error handling works seamlessly with the mode switcher node:

### For Hardware Errors:
1. Error detected → State machine enters ERROR state with HARDWARE_ERROR type
2. Mode switcher can switch to MANUAL mode for manual recovery
3. User resolves hardware issues manually
4. Mode switcher switches back to MOVEIT mode or use recovery service
5. State machine returns to IDLE state

### Safe Mode Switching:
- Mode switching is only allowed in safe states (IDLE, MOVING, FINAL) 
- Mode switching is always allowed FROM ERROR state for recovery
- Mode switching is blocked during critical operations (PICKING, PLACING)

## Recovery Methods

### Automatic Recovery (MoveIt2 errors only)
- No action required
- State machine automatically returns to IDLE after 1 second
- Can immediately send new commands

### Manual Recovery Service
```bash
ros2 service call /xarm7_state_machine/recover std_srvs/srv/Trigger
```
- Works for any error type when in ERROR state
- Clears robot errors, sets mode to POSITION, sets state to READY
- Returns state machine to IDLE state

### Mode Switcher Recovery
- Switch to MANUAL mode: allows manual robot control
- Resolve issues manually using robot teaching pendant or manual commands
- Switch back to MOVEIT mode when ready

## Error Detection Points

### MoveIt2 Planning Errors:
- `performMovement()` planning failures
- `executeMovingState()` failures  
- `executePickingState()` failures
- `executePlacingState()` failures

### Hardware Errors:
- xarm robot error codes detected in `robotStateCallback()`
- Stop commands received in `stopCallback()`
- Robot state validation failures

## Logging and Debugging

### Error Type Identification:
```
MODE: POSITION -- XARM7-STATE: ERROR: Error encountered in state MOVING - Robot state: RUNNING, Error code: 0, Error type: MOVEIT_PLANNING
```

### MoveIt2 Error Recovery:
```
=== MOVEIT PLANNING ERROR DETECTED ===
This error allows automatic recovery with task replanning
State machine will automatically transition back to IDLE for retry
```

### Hardware Error Recovery:
```
=== HARDWARE ERROR DETECTED ===
This error requires manual intervention and recovery
Hardware error code: 19
Consider switching to MANUAL mode via mode switcher for manual recovery
```

## Testing Scenarios

### Test MoveIt2 Error Recovery:
1. Send invalid target pose (unreachable position)
2. Verify ERROR state with MOVEIT_PLANNING type
3. Verify automatic return to IDLE after 1 second
4. Send new valid command and verify normal operation

### Test Hardware Error Recovery:
1. Trigger hardware error (e.g., emergency stop)
2. Verify ERROR state with HARDWARE_ERROR type  
3. Verify state machine stays in ERROR
4. Use recovery service or mode switcher for manual recovery
5. Verify return to IDLE state after recovery

### Test Mode Switcher Integration:
1. Trigger hardware error
2. Switch to MANUAL mode via mode switcher
3. Resolve issues manually
4. Switch back to MOVEIT mode
5. Verify normal operation resumes

## Benefits

1. **Improved Robustness**: Automatic recovery from common planning failures
2. **Safety**: Manual intervention required for serious hardware issues
3. **Productivity**: Reduces downtime from transient planning failures
4. **Flexibility**: Multiple recovery methods available based on error type
5. **Integration**: Works seamlessly with existing mode switcher and service architecture

## Future Enhancements

- Add retry count limits for automatic MoveIt2 error recovery
- Implement different recovery strategies based on specific error codes
- Add telemetry and statistics on error types and recovery success rates
- Extend error classification for more granular handling
