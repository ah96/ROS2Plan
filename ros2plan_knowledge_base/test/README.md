# ros2plan_knowledge_base - unit tests

## build tests manually

Build ros2plan_knowledge_base tests:

        catkin build --no-deps ros2plan_knowledge_base --make-args run_tests

Build all catkin ws tests:

        catkin build --make-args run_tests

## run tests manually

Knowledge base tests:

        rostest ros2plan_knowledge_base knowledge_base.test --text

Run all catkin ws tests:

        catkin run_tests
