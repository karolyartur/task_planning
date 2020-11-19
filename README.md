# task_planning
Control MTC through action interface

## Usage:

Build and run:
```bash
roslaunch task_planning_core demo.launch
```

In two separate terminals type:
```bash
actionlib axclient.py /control_task
```

and

```bash
actionlib axclient.py /add_pick
```

The name of the object is "object". Send a goal to the `add_pick` action with this parameter to add a pick container to the task.

After that, send a goal to the `control_task` action with data set to "10" and operation set to `3` to start the planning.

Inspect the result in RViz

## Issue

Add the same pick twice so, the task fails. With commit 30a1643310deb96689cea3ab95567d8bb0536dd7 this should freeze the action server, but with commit 71af9aacd83bf1af3c00977d1235792d8f234a7c it completes.