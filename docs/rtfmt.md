# Real-Time Fast Marching Trees (RT-FMT)

## Overview
This repository includes a MATLAB implementation of Real-Time Fast Marching Trees (RT-FMT) for motion planning in dynamic environments.
RT-FMT enables efficient replanning by reusing previously generated samples and tree structures, allowing the planner to react in real time to changes in the environment.


## Background
RT-FMT was proposed by Silveira et al. as a real-time variant of FMT*, capable of handling dynamic obstacles and online map updates.

The original RT-FMT reference implementation was released in C#:
- [https://github.com/offroad-robotics/rt-fmt-icra](https://github.com/offroad-robotics/rt-fmt-icra)


## This MATLAB Implementation
This repository provides a MATLAB port and adaptation of the RT-FMT algorithm described by Silveira et al. While the core algorithmic ideas are preserved, the implementation details differ significantly from the original C# version.In particular, this implementation uses vectorized operations instead of object-oriented data structures
> [!NOTE]
> RT-FMT is implemented as an extension of the existing FMT planner in this repository and shares several core functions


## Usage
RT-FMT is designed to operate in a real-time planning loop, where the planner incrementally expands the tree, updates the environment, and produces a local path for execution.

Example scripts are provided in:

```
examples/test_rt.m          % RT-FMT with straight-line edge connections
examples/test_rt_dubins.m   % RT-FMT with Dubins path edge connections
```

### 1. Create the RT-FMT planner

First, initialize the planner structure using `rt_fmt_planner`:

```matlab
planner = rt_fmt_planner( planMap, mapLimits, start, goal, rn, ...
    'N', N, ...
    'w', 0, ...
    'expandTreeRate', 32, ...
    'safeRadiusDObstacle', 100, ...
    'goalRadius', goalRadius );
```

#### Required arguments

* `planMap`: 3D occupancy map (`occupancyMap3D`), used for planning
* `mapLimits`: Planning bounds `[xmin xmax; ymin ymax; zmin zmax]`
* `start`: Initial vehicle position
* `goal`: Goal position
* `rn`: Neighborhood radius used for connecting samples

#### Optional parameters

* `N`: Number of samples used by the planner
* `w`: Cost weight penalty (same formulation as in the standard FMT planner)
* `goalRadius`: Acceptable distance to the goal for termination
* `expandTreeRate`: Number of tree expansion iterations executed per planning cycle
* `safeRadiusDObstacle`: Safety radius maintained around dynamic obstacles
* `flightParams`: Enables Dubins-based motion constraints.
  Passed as an array:
  ```matlab
  [MaxRollAngle, AirSpeed, MinFlightPathAngle, MaxFlightPathAngle]
  ```
### 2. Run a real-time planning step
At each control cycle, update the planner using the current vehicle state and dynamic obstacles:

```matlab
planner = tick(dynamicObstacles, q_now, planner);
```
Where:

* `dynamicObstacles`: `n × 3` array of obstacle positions `[x y z]`
* `q_now`: is the current vehicle position

This function executes `expandTreeRate` iterations of RT-FMT expansion.

### 3. Generate a local path
After expanding the tree, request a local path for execution:
```matlab
[path, goalHit, pathFound, planner] = generatePath(planner);
```
Outputs:

* `path`: Local path segment to be followed by the vehicle
* `goalHit`: Boolean indicating whether the goal region has been reached
* `pathFound`: Boolean indicating whether a valid path exists
* `planner`: Updated planner state
  
## Acknowledgements
The RT-FMT algorithm and original reference implementation were developed by **Silveira et al.**

> Silveira, J., Cabral, K., Givigi, S., & Marshall, J. A.  
> *Real-Time Fast Marching Tree for Mobile Robot Motion Planning in Dynamic Environments*.  
> IEEE International Conference on Robotics and Automation (ICRA), 2023.

Original C# implementation:
https://github.com/offroad-robotics/rt-fmt-icra
