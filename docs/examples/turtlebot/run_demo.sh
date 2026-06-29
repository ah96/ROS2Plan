#!/usr/bin/env bash
# run_demo.sh  —  Full turtlebot ROS2Plan demo, end to end, in one terminal.
#
# Pipeline: knowledge base -> problem generation -> POPF planning -> plan
#           parsing -> dispatch to simulated action interfaces.
#
# Usage:
#   bash run_demo.sh [--waypoints N]      (default: 5)
#
# Requirements: ROS2 Humble, ROS2Plan built and sourced:
#   source /opt/ros/humble/setup.bash
#   source ~/git/ros2_ws/install/setup.bash
#
# Each node is started with `ros2 run` and explicit parameters. The nodes
# advertise their services/topics in the root namespace (e.g. /start_planning,
# /parse_plan, /dispatch_plan, /problem_instance, /planner_output,
# /complete_plan), so the default relative names already wire them together.

set -u

WPS=5
while [[ $# -gt 0 ]]; do
    case $1 in
        --waypoints) WPS="$2"; shift 2 ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

PS=ros2plan_planning_system
KB=ros2plan_knowledge_base
HERE="$(cd "$(dirname "$0")" && pwd)"
SHARE="$(ros2 pkg prefix "$PS")/share/$PS"
DOMAIN="$SHARE/test/pddl/turtlebot/domain.pddl"
POPF="$SHARE/common/bin/popf"
WORK="$(mktemp -d /tmp/ros2plan_demo.XXXXXX)"

PIDS=()
cleanup() { for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done; }
trap cleanup EXIT

echo "=== ROS2Plan Turtlebot Demo (${WPS} waypoints, robot: kenny) ==="
echo "    domain : $DOMAIN"
echo "    workdir: $WORK"

# ── Step 1: Knowledge base ───────────────────────────────────────────────────
echo "[1/6] Starting the knowledge base..."
ros2 run "$KB" knowledgeBase --ros-args \
    -p domain_path:="$DOMAIN" > "$WORK/kb.log" 2>&1 &
PIDS+=($!)
for i in $(seq 1 30); do
    ros2 service list 2>/dev/null | grep -q "/$KB/domain/name" && break; sleep 1
done

# ── Step 2: Load the initial state and goals ─────────────────────────────────
echo "[2/6] Loading initial state and goals..."
python3 "$HERE/load_state.py" --waypoints "$WPS" --robot kenny

# ── Step 3: Start the planning-system nodes and simulated actions ────────────
echo "[3/6] Starting problem / planner / parser / dispatcher and action interfaces..."
ros2 run "$PS" problemInterface --ros-args \
    -p knowledge_base:="$KB" \
    -p problem_path:="$WORK/problem.pddl" \
    -p planning_language:=PDDL > "$WORK/problem.log" 2>&1 &
PIDS+=($!)

ros2 run "$PS" popf_planner_interface --ros-args \
    -p use_problem_topic:=true \
    -p domain_path:="$DOMAIN" \
    -p problem_path:="$WORK/problem.pddl" \
    -p data_path:="$WORK/" \
    -p planner_command:="timeout 30 $POPF DOMAIN PROBLEM" > "$WORK/planner.log" 2>&1 &
PIDS+=($!)

ros2 run "$PS" pddl_simple_plan_parser --ros-args \
    -p knowledge_base:="$KB" > "$WORK/parser.log" 2>&1 &
PIDS+=($!)

ros2 run "$PS" pddl_simple_plan_dispatcher --ros-args \
    -p knowledge_base:="$KB" > "$WORK/dispatcher.log" 2>&1 &
PIDS+=($!)

for action in undock dock localise goto_waypoint; do
    ros2 run "$PS" simulatedAction --ros-args \
        -p knowledge_base:="$KB" \
        -p pddl_action_name:="$action" \
        -p action_duration:=0.5 \
        -p action_probability:=1.0 > "$WORK/sim_${action}.log" 2>&1 &
    PIDS+=($!)
done
sleep 5

# ── Step 4: Generate the problem and plan ────────────────────────────────────
# Use the /planning_server Empty service (not the /start_planning action): the
# service runs the planner using the node parameters set above, whereas the
# action reads domain/problem/planner_command from the action goal instead.
echo "[4/6] Generating the problem and planning (POPF)..."
timeout 20 ros2 service call /problem_generation_server std_srvs/srv/Empty "{}" > /dev/null
sleep 2
timeout 40 ros2 service call /planning_server std_srvs/srv/Empty "{}" > /dev/null

# ── Step 5: Parse and dispatch ───────────────────────────────────────────────
echo "[5/6] Parsing and dispatching the plan..."
timeout 20 ros2 service call /parse_plan std_srvs/srv/Empty "{}" > /dev/null
timeout 150 ros2 service call /dispatch_plan ros2plan_dispatch_msgs/srv/DispatchService "{}" \
    | grep -E "success|goal_achieved" | head

# ── Step 6: Report the final knowledge-base state ────────────────────────────
echo "[6/6] Final knowledge-base state:"
echo "  visited waypoints:"
timeout 12 ros2 service call "/$KB/state/propositions" \
    ros2plan_knowledge_msgs/srv/GetAttributeService "{predicate_name: 'visited'}" \
    2>/dev/null | grep -oE "wp[0-9]+" | sort -u | sed 's/^/    /'
echo "  docked:"
timeout 12 ros2 service call "/$KB/state/propositions" \
    ros2plan_knowledge_msgs/srv/GetAttributeService "{predicate_name: 'docked'}" \
    2>/dev/null | grep -oE "value='?kenny'?" | head -1 | sed 's/^/    robot /'

echo "=== Demo complete. (logs in $WORK) ==="
