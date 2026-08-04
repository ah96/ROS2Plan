# ros2plan_knowledge_base — tests

These tests are built and run with `colcon` (ament). They are launch-tests: a
`launch_testing` harness (`test/launch/knowledge_base_test.py`) brings up the `knowledgeBase`
node with the amazon (`driverlog-simple`) fixture, then runs the gtest executable
(`test/src/KnowledgeBaseTests.cpp`) which exercises the knowledge-base services.

## Build and run

From the colcon workspace root:

```bash
# The test launches a node named ros2plan_knowledge_base, so any other ROS 2 system on the
# same domain is indistinguishable from it and the test queries the wrong one. Isolate:
export ROS_DOMAIN_ID=78
colcon build --packages-select ros2plan_knowledge_base
colcon test  --packages-select ros2plan_knowledge_base
colcon test-result --verbose
```

## Run a single launch-test directly (with console output)

```bash
# build first (the gtest binary is produced under build/, not installed)
launch_test src/<path-to>/ros2plan_knowledge_base/test/launch/knowledge_base_test.py \
    test_exe:=build/ros2plan_knowledge_base/knowledge_base_test
```
