# task_planning
Controlling MTC through an action interface

## Usage:

Build and run:
```bash
roslaunch task_planning_core demo.launch
```

In two separate terminals, do:
```bash
rosrun task_planning_core add_pick_action_client.py
```

and then:

```bash
rosrun task_planning_core plan_action_client.py
```

This first populates the task with a "Pick" action, and then requests the task to be planned.

Afterwards, you can inspect the result in RViz.

## Issue

If you plan twice, on commit 30a1643310deb96689cea3ab95567d8bb0536dd7 the program freezes, but on commit 71af9aacd83bf1af3c00977d1235792d8f234a7c it completes.
