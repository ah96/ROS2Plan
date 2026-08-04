# ros2plan_planning_system — tests

These tests are built and run with `colcon` (ament). There are two kinds:

* **`chimp_problem_test`** — a pure gtest unit test (no ROS runtime) for CHIMP problem-file
  generation (`test/src/CHIMPProblemTests.cpp`).
* **launch-tests** — each pairs a gtest executable (`test/src/*Tests.cpp`) with a
  `launch_testing` harness (`test/launch/*_test.py`) that brings up the knowledge base and the
  node(s) under test, then runs the gtest against the live services/topics:
  * `problem_interface_test` — problem generation
  * `parsing_interface_test` — plan parsing (simple + esterel)
  * `plan_dispatch_test` — plan dispatch
  * `planner_interface_test` — POPF planner interface (full KB → problem → plan chain)
  * `chimp_planner_interface_test` — only built when CHIMP is enabled
    (`-DROS2PLAN_COMPILE_CHIMP=ON`, requires the Java CHIMP planner)

## Build and run

From the colcon workspace root:

```bash
export ROS_DOMAIN_ID=78   # see "Notes" — isolate from other ROS 2 systems
colcon build --packages-select ros2plan_planning_system
colcon test  --packages-select ros2plan_planning_system --executor sequential
colcon test-result --verbose
```

## Run a single launch-test directly (with console output)

```bash
launch_test src/<path-to>/ros2plan_planning_system/test/launch/problem_interface_test.py \
    test_exe:=build/ros2plan_planning_system/problem_interface_test
```

## Notes

* **Run on a dedicated `ROS_DOMAIN_ID`.** The tests launch nodes under fixed names
  (`ros2plan_knowledge_base` and friends), so any other ROS 2 system reachable on the same
  domain is indistinguishable from the one under test — the tests will query the wrong nodes
  and fail in confusing ways (a knowledge base answering with a different domain's predicates).
  On a machine running another ROS2Plan system, the same suite gives 8 failures on the default
  domain and passes fully on a dedicated one. Run the packages' tests `--executor sequential`
  too: the fixed service timeouts flake when several packages' tests run concurrently.
* In ROS 2 the planning-system nodes advertise their trigger services/topics with relative
  names in the default namespace, so they resolve to the **root** namespace
  (`/problem_generation_server_params`, `/planning_server_params`, `/planner_output`,
  `/complete_plan`, `/dispatch_plan`, …) — not under the node name. The tests use those
  resolved names. The two plan parsers are launched in separate namespaces (`simple`/`esterel`)
  to deconflict their identical relative names.
* The PDDL problem generator emits a cosmetic blank line after each domain predicate (matching
  upstream ROSPlan); string comparisons in the tests ignore blank lines and check the
  meaningful content.
