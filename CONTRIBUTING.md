# Contributing

## Prerequisites
- ROS 2 Jazzy
- `colcon`
- `rosdep`

## Setup
```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

If switching from a Humble-built workspace to Jazzy:
```bash
rm -rf build install log
```

## Build and Test
```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src
source install/setup.bash
colcon test --base-paths src
colcon test-result --verbose
```

## Style
- Keep package metadata (`package.xml`, `CMakeLists.txt`) in sync.
- Keep launch/config changes documented in `README.md`.
