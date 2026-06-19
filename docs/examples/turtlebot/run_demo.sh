#!/usr/bin/env bash
# run_demo.sh  —  Full turtlebot ROS2Plan demo in a single terminal using tmux.
#
# Usage:
#   bash run_demo.sh [--waypoints N]   (default: 5)
#
# Requirements: tmux, ROS2 Humble, ROS2Plan built and sourced

set -e

WPS=5
while [[ $# -gt 0 ]]; do
    case $1 in
        --waypoints) WPS="$2"; shift 2 ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

DOMAIN=$(ros2 pkg prefix rosplan_planning_system)/share/rosplan_planning_system/test/pddl/turtlebot/domain.pddl
POPF=$(ros2 pkg prefix rosplan_planning_system)/share/rosplan_planning_system/common/bin/popf
PLANNER_CMD="timeout 30 ${POPF} DOMAIN PROBLEM"

echo "=== ROS2Plan Turtlebot Demo ==="
echo "Waypoints : ${WPS}"
echo "Domain    : ${DOMAIN}"
echo ""

# ── Step 1: Knowledge Base ────────────────────────────────────────────────────
echo "[1/7] Starting Knowledge Base..."
ros2 launch rosplan_knowledge_base rosplan_knowledge_base.launch.py \
    domain_path:="${DOMAIN}" &
KB_PID=$!
sleep 3

# ── Step 2: Load initial state ────────────────────────────────────────────────
echo "[2/7] Loading initial state (${WPS} waypoints, robot: kenny)..."
python3 "$(dirname "$0")/load_state.py" --waypoints "${WPS}" --robot kenny

# ── Step 3: Problem generation ────────────────────────────────────────────────
echo "[3/7] Launching Problem Interface..."
ros2 run rosplan_planning_system problemInterface &
PROB_PID=$!
sleep 2

echo "      Generating problem file..."
timeout 15 ros2 service call \
    /rosplan_problem_interface/problem_generation_server \
    std_srvs/srv/Empty "{}"

# ── Step 4: Planning ──────────────────────────────────────────────────────────
echo "[4/7] Launching Planner Interface (POPF)..."
ros2 run rosplan_planning_system popfPlannerInterface \
    --ros-args \
    -p domain_path:="${DOMAIN}" \
    -p planner_command:="${PLANNER_CMD}" &
PLAN_PID=$!
sleep 2

echo "      Calling planner..."
ros2 action send_goal /rosplan_planner_interface/start_planning \
    rosplan_dispatch_msgs/action/Plan "{}"

# ── Step 5: Plan parsing ──────────────────────────────────────────────────────
echo "[5/7] Launching Plan Parser..."
ros2 run rosplan_planning_system PDDLSimplePlanParser \
    --ros-args \
    -p planner_topic:=planner_output &
PARSER_PID=$!
sleep 2

echo "      Parsing plan..."
timeout 15 ros2 service call \
    /rosplan_parsing_interface/parse_plan \
    std_srvs/srv/Empty "{}"

# ── Step 6: Simulated action interfaces ──────────────────────────────────────
echo "[6/7] Starting simulated action interfaces..."
for action in undock dock localise goto_waypoint; do
    ros2 launch rosplan_planning_system simulated_action.launch.py \
        pddl_action_name:="${action}" \
        action_duration:=1.0 \
        action_probability:=1.0 &
done
sleep 3

# ── Step 7: Dispatch ──────────────────────────────────────────────────────────
echo "[7/7] Launching dispatcher and dispatching plan..."
ros2 run rosplan_planning_system SimplePlanDispatcher \
    --ros-args \
    -p plan_topic:=complete_plan &
DISP_PID=$!
sleep 2

echo "      Dispatching..."
timeout 120 ros2 service call \
    /dispatch_plan \
    rosplan_dispatch_msgs/srv/DispatchService "{}"

echo ""
echo "=== Dispatch complete. ==="
echo ""
echo "Check KB state:"
echo "  ros2 service call /rosplan_knowledge_base/state/propositions rosplan_knowledge_msgs/srv/GetAttributeService \"{attribute_name: 'visited'}\""
echo "  ros2 service call /rosplan_knowledge_base/state/propositions rosplan_knowledge_msgs/srv/GetAttributeService \"{attribute_name: 'docked'}\""
