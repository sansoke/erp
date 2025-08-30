# virtual_frame_broadcaster

## Launchfile configuration

- `parent_frame`: frame which is broadcasted by localization
- `child_frame`: frame which will be listened by planner (movebase)
- `cmd_vel_topic`: cmd_vel topic which is published by planner
- `dt`: local planner duration
- `delayed_step`: delayed step btw high-level and low-level PC

## Usage

```bash
$ roslaunch virtual_frame_broadcaster virtual_frame_broadcaster.launch
```
