# ROS2Plan Tutorial

The tutorial has moved to a structured set of pages under [docs/](docs/index.md), modelled
on the original ROSPlan documentation.

Start here:

- **[Documentation home](docs/index.md)**
- [Installation](docs/installation.md)

Work through the tutorials in order — they all use the same turtlebot example:

1. [The Knowledge Base](docs/tutorials/01-knowledge-base.md)
2. [Problem Generation](docs/tutorials/02-problem-generation.md)
3. [Planning](docs/tutorials/03-planning.md)
4. [Plan Execution](docs/tutorials/04-plan-execution.md)
5. [Simulated Actions](docs/tutorials/05-simulated-actions.md)
6. [Action Interfaces](docs/tutorials/06-action-interface.md)
7. [The Sensing Interface](docs/tutorials/07-sensing-interface.md)
8. [The rqt GUI](docs/tutorials/08-rqt-gui.md)

To run the whole turtlebot pipeline end to end in one command:

```bash
bash docs/examples/turtlebot/run_demo.sh --waypoints 5
```

Reference material (node parameters, services, message types) is linked from the
[documentation home](docs/index.md#reference).
