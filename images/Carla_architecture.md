# Motion Planning Stack Architecture

## System Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         CARLA SIMULATOR                                  │
│                    (Environment, Obstacles, Waypoints)                   │
└───────────────────────────────┬───────────────────────────────────────────┘
                                │
                                │ Sensor Data
                                │ (Position, Speed, Obstacles)
                                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         BEHAVIORAL PLANNER                              │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ State Machine:                                                   │   │
│  │   FOLLOW_LANE → DECELERATE_TO_STOP → STAY_STOPPED              │   │
│  │                                                                  │   │
│  │ Priority Rules:                                                 │   │
│  │   1. Need to stop (highest)                                  │   │
│  │   2. Check Lead Vehicle (medium)                                │   │
│  │   3. Follow Lane (default)                                       │   │
│  │                                                                  │   │
│  │ Output: Goal State [x_goal, y_goal, v_goal]                    │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└───────────────────────────────┬───────────────────────────────────────────┘
                                │ Goal State
                                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          LOCAL PLANNER                                  │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ Goal State Set Generation                                        │   │
│  │   - Generate lateral offset goal states                         │   │
│  │   - Transform to vehicle frame                                   │   │
│  │                                                                  │   │
│  │ Output: Multiple Goal States (vehicle frame)                    │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└───────────────────────────────┬───────────────────────────────────────────┘
                                │ Goal State Set
                                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         PATH OPTIMIZER                                   │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ For each goal state:                                             │   │
│  │   - Polynomial Spiral Optimization                                │   │
│  │   - Minimize: bending energy + position error + yaw error        │   │
│  │   - Sample spiral to generate discrete path points               │   │
│  │                                                                  │   │
│  │ Output: Multiple Candidate Paths [x, y, θ]                      │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└───────────────────────────────┬───────────────────────────────────────────┘
                                │ Candidate Paths
                                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                       COLLISION CHECKER                                  │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ Collision Detection                                               │   │
│  │   - Circle-based approximation                                    │   │
│  │   - Check intersection with static obstacles                     │   │
│  │                                                                  │   │
│  │ Path Selection                                                   │   │
│  │   - Filter collision-free paths                                  │   │
│  │   - Score: centerline tracking + obstacle proximity              │   │
│  │   - Select best path (fallback to previous if none valid)        │   │
│  │                                                                  │   │
│  │ Output: Best Path [x, y, θ]                                     │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└───────────────────────────────┬───────────────────────────────────────────┘
                                │ Best Path
                                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                        VELOCITY PLANNER                                  │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ Speed Profile Generation                                          │   │
│  │                                                                  │   │
│  │ Mode Selection:                                                   │   │
│  │   ├─ Decelerate to Stop                                          │   │
│  │   ├─ Follow Lead Vehicle (lead vehicle detected)                 │   │
│  │   └─ Nominal Profile (default lane following)                    │   │
│  │                                                                  │   │
│  │ Output: Waypoints with Speed Profile [[x, y, v], ...]          │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└───────────────────────────────┬───────────────────────────────────────────┘
                                │ Waypoints with Speed
                                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                           CONTROLLER                                     │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ PID Control                                                       │   │
│  │   - Lateral Control: Steering angle                               │   │
│  │   - Longitudinal Control: Throttle/Brake                         │   │
│  │   - Waypoint tracking                                             │   │
│  │                                                                  │   │
│  │ Output: Control Commands [throttle, steer, brake]               │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└───────────────────────────────┬───────────────────────────────────────────┘
                                │ Control Commands
                                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         CARLA SIMULATOR                                  │
│                    (Vehicle Execution)                                   │
└─────────────────────────────────────────────────────────────────────────┘
```

## Data Flow Summary

```
Waypoints + Obstacles
    │
    ▼
Behavioral Planner → Goal State [x, y, v]
    │
    ▼
Local Planner → Multiple Goal States (lateral offsets)
    │
    ▼
Path Optimizer → Multiple Candidate Paths
    │
    ▼
Collision Checker → Best Path
    │
    ▼
Velocity Planner → Waypoints with Speed Profile
    │
    ▼
Controller → [Throttle, Steer, Brake]
    │
    ▼
CARLA Vehicle Execution
```

## Behavioral States

**FOLLOW_LANE**
- Normal lane following
- Select goal waypoint within lookahead distance
- Check for stop signs ahead

**DECELERATE_TO_STOP**
- Approaching stop sign
- Set goal speed to zero
- Transition when speed below threshold

**STAY_STOPPED**
- At stop sign
- Count cycles before proceeding
- Transition back to FOLLOW_LANE when clear

## Velocity Planning Modes

**Decelerate to Stop**
- Trapezoidal deceleration profile
- Stop before stop line buffer

**Follow Lead Vehicle**
- Time-gap based following
- Match lead vehicle speed by time-gap point

**Nominal Profile**
- Accelerate/decelerate to desired speed
- Maintain speed along path

## Real-Time Design

- **Receding Horizon**: Replan at fixed intervals
- **Warm-Start**: Previous path fallback if planning fails
- **Decoupled Planning**: Path and velocity planned separately
- **Fixed Candidate Count**: Predictable computation time
