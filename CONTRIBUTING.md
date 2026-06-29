# Contributing

## Development setup

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-org/ROS2Plan.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Build and test cycle

```bash
# Build a single package
colcon build --packages-select ros2plan_knowledge_base

# Build everything
colcon build

# Run tests
colcon test
colcon test-result --verbose
```

## Code conventions

- C++17, formatted with the ROS2 style guide (4-space indent, `snake_case` methods)
- Python 3.10+, PEP 8
- New service servers: use the `~/service_name` prefix so they resolve under the node namespace
- New publishers on latched/transient topics: declare `rclcpp::QoS(1).transient_local()` on **both** publisher and subscriber
- Blocking service calls from setup code or executor callbacks: use the satellite-node pattern with `rclcpp::spin_until_future_complete`

## Reporting issues

Open a GitHub issue with:
- ROS2 distro and Ubuntu version
- Package(s) affected
- Steps to reproduce
- Relevant log output (`ros2 run ... 2>&1`)

## Pull requests

1. Branch from `master`
2. One logical change per PR
3. Update `CHANGELOG.md` under the `Unreleased` section
4. Ensure `colcon build` and `colcon test` pass before opening the PR
