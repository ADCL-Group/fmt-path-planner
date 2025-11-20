# Fast Marching Trees (FMT) for UAV Path Planning
This repository implements the **Fast Marching Trees** (FMT) algorithm for motion planning, integrates it with MATLAB’s UAV and Navigation Toolboxes, and demonstrates path planning and smoothing for a fixed-wing UAV flying in an urban environment.

## Overview

This repository brings together Fast Marching Trees (FMT) and Dubins-curve smoothing for fixed-wing UAV navigation. Beginning with the FMT algorithm described by Janson & Pavone, it builds a tree of possible vehicle states on an occupancy map of an urban environment. To ensure safety margins around obstacles, the planner operates on an “inflated” version of that map, then hands off the raw FMT path to a Dubins-curve smoother. Smoothing happens on a slightly less-inflated map so the vehicle can cut tighter turns without risking collision. If any smoothed segment still intersects an obstacle, the algorithm automatically replans just that leg with FMT and retries smoothing.

## Files

- `runExamples.m`  
  Launcher for the example scripts. Prompts you to choose which scenario to run (custom map vs. real map, 3D vs. 4D waypoints) and then calls the corresponding script.

- `setupPath.m`  
  Helper script that adds all relevant project folders (e.g., `src`, `examples`, `data`, etc.) to the MATLAB path so the planner, map builders, and smoothing functions are accessible.

- `examples/`  
  Contains the example scripts:
  - custom 3D test map with single and multiple waypoints (3D and 4D)
  - real 3D map from OpenStreetMap with multiple 3D waypoints
  - real 3D map from OpenStreetMap with multiple 4D waypoints (x, y, z, heading)

- `data/osm/boston.osm`  
  Example OpenStreetMap XML file used to build a real 3D occupancy map of an urban area for the FMT planner examples.

- `src/`  
  Core implementation:
  - FMT-based path planner
  - Dubins-curve smoothing functions
  - Map creation utilities (custom test maps and OpenStreetMap-based occupancy maps)


## Usage
The simplest way to get started is to run one of the example scripts found in the `examples/` folder. All examples require adding the project folders to the MATLAB path, which is done automatically by calling:
```matlab
setupPath();
```
For convenience, you can also launch examples through:
```matlab
runExamples
```
### Custom 700 m×700 m×700 m test map (examples/test_customMap_waypoints.m)
This example includes:
- A single-goal 3D planning test
- A multi-waypoint 3D test
- A multi-waypoint 4D test

This script begins by building three occupancy‐map layers (original, inflated for planning, and lightly inflated for smoothing) with
```matlab
[origMap, smoothMap, planMap, mapLimits] = createTest3DMap(20, 35);
```
Here, `20` and `35` are the dimensions that definen by how much to inflate occupied locations in meters; `planMap` is your inflated planning map, and `smoothMap` the map with smaller clearance for Dubins smoothing.

Next, the planning limits and waypoints are set:
```matlab
limits = [mapLimits; 0 2*pi];      % xyz limits + heading limits
start = [0 0 60];
wpt1  = [110 360 65];
wpt2  = [420 340 80];
goal  = [440 650 60];
```

Then to configure FMT parameters*:

```matlab
N = 5000;                                % number of samples
rn = optimalRadius(N+2, limits(1:3,:));  % neighbor radius
goalRadius = 10;                          % acceptable goal region radius
```

Run FMT on the inflated planning map* 

```matlab
wpts = [start; goal];
[traj, E, V] = FMTWaypoints(planMap, limits(1:3,:), wpts(:,1:3), N, rn, goalRadius);
```
The outputs `E` and `V` describe the tree structure: `V` holds all sampled nodes, and `E` indexes which of those nodes were connected into the final tree.
Finally, the raw path is smoothed using the Dubins‐based algorithm. After specifying your aircraft’s maximum roll angle, constant airspeed, and flight‐path‐angle limits:

```matlab
MaxRollAngle         = 50*pi/180;
AirSpeed             = 20; % m/s
FlightPathAngleLimit = [-45, 45]*pi/180;
flightParams = [MaxRollAngle, AirSpeed, FlightPathAngleLimit];
smoothedTraj = smoothTrajectory( flightParams, limits, smoothMap, traj, wpts, goalRadius);
```
The result, `smoothedTraj`, is again an array of waypoints (X, Y, Z, heading).

You can extend `wpts` to more than two rows (start plus multiple intermediate goals) and it will automatically plan, smooth, and stitch together each successive leg into one continuous, flyable path.

### Real-world OpenStreetMap (.osm) building model (test_realMap_waypoints_3D.m and test_realMap_waypoints_4D.m)
These files use a function that loads a real-world building model from an OpenStreetMap XML file. This script loads a `.osm` file (downloaded via Overpass API, e.g., from [osmbuildings.org](https://osmbuildings.org/)):
```matlab
   result = buildOccupancyMap("Occupancy_Map/osm/boston.osm", 0.1, 20, 35);
   origMap   = result.maps{1};
   smoothMap = result.maps{2};
   planMap   = result.maps{3};
   mapLimits = result.mapLimits;
```
where `0.1` m is the map resolution, and `20`, `35` m are the plan/smooth inflation distances. The planning and smoothing steps are identical to the custom-map scenario.

## References

1. Janson, L. & Pavone, M. (2013). *Fast Marching Trees: A Fast Marching Sampling-Based Method for Optimal Motion Planning in Many Dimensions*. Int. Symp. on Robotics Research.
   (PDF: https://stanfordasl.github.io/wp-content/papercite-data/pdf/Janson.Pavone.ISRR13.pdf)  
2. MathWorks. (n.d.). *Motion Planning with RRT for Fixed-Wing UAV*.  
   https://www.mathworks.com/help/uav/ug/motion-planning-with-rrt-for-fixed-wing-uav.html
