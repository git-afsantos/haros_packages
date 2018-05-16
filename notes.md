# Setup

1. Initialisation
```python
call on SafetyControllerNodelet.onInit()
implies subscribe on "enable"
  and subscribe on "disable"
  and subscribe on "events/bumper"
  and subscribe on "events/cliff"
  and subscribe on "events/wheel_drop"
  and subscribe on "reset"
  and advertise on "cmd_vel"
  and spawn thread on SafetyControllerNodelet.update()
```

2.  Worker thread calls `spin`
```python
call on SafetyControllerNodelet.update()
implies SafetyController.getState() becomes True
  and call on SafetyController.spin()
      every 10 hz
      while SafetyControllerNodelet.shutdown_requested_ is False
        and ROS.ok() is True
```

# Subscribers

1. Topic `enable` when disabled
```python
receive on "enable"
  and SafetyController.getState() is False
implies SafetyController.getState() becomes True
```

2. Topic `enable` when enabled
```python
receive on "enable"
  and SafetyController.getState() is True
implies SafetyController.getState() remains True
```

3. Topic `disable` when enabled
```python
receive on "disable"
  and SafetyController.getState() is True
implies SafetyController.getState() becomes False
```

4. Topic `disable` when disabled
```python
receive on "disable"
  and SafetyController.getState() is False
implies SafetyController.getState() remains False
```

5. Topic `events/cliff`
```python
receive on "events/cliff"
  and msg.state is CliffEvent.CLIFF
implies SafetyController.last_event_time_ becomes Time.now()
```

6. Topic `events/cliff`
```python
receive on "events/cliff"
  and msg.state is CliffEvent.CLIFF
  and msg.sensor is CliffEvent.LEFT
implies SafetyController.cliff_left_detected_ becomes True
```

7. Topic `events/cliff`
```python
receive on "events/cliff"
  and msg.state is CliffEvent.CLIFF
  and msg.sensor is CliffEvent.CENTER
implies SafetyController.cliff_center_detected_ becomes True
```

8. Topic `events/cliff`
```python
receive on "events/cliff"
  and msg.state is CliffEvent.CLIFF
  and msg.sensor is CliffEvent.RIGHT
implies SafetyController.cliff_right_detected_ becomes True
```

9. Topic `events/cliff`
```python
receive on "events/cliff"
  and msg.state is CliffEvent.FLOOR
  and msg.sensor is CliffEvent.LEFT
implies SafetyController.cliff_left_detected_ becomes False
```

10. Topic `events/cliff`
```python
receive on "events/cliff"
  and msg.state is CliffEvent.FLOOR
  and msg.sensor is CliffEvent.CENTER
implies SafetyController.cliff_center_detected_ becomes False
```

11. Topic `events/cliff`
```python
receive on "events/cliff"
  and msg.state is CliffEvent.FLOOR
  and msg.sensor is CliffEvent.RIGHT
implies SafetyController.cliff_right_detected_ becomes False
```

12. Topic `events/bumper`
```python
receive on "events/bumper"
  and msg.state is BumperEvent.PRESSED
implies SafetyController.last_event_time_ becomes Time.now()
```

13. Topic `events/bumper`
```python
receive on "events/bumper"
  and msg.state is BumperEvent.PRESSED
  and msg.bumper is BumperEvent.LEFT
implies SafetyController.bumper_left_pressed_ becomes True
```

14. Topic `events/bumper`
```python
receive on "events/bumper"
  and msg.state is BumperEvent.PRESSED
  and msg.bumper is BumperEvent.CENTER
implies SafetyController.bumper_center_pressed_ becomes True
```

15. Topic `events/bumper`
```python
receive on "events/bumper"
  and msg.state is BumperEvent.PRESSED
  and msg.bumper is BumperEvent.RIGHT
implies SafetyController.bumper_right_pressed_ becomes True
```

16. Topic `events/bumper`
```python
receive on "events/bumper"
  and msg.state is BumperEvent.RELEASED
  and msg.bumper is BumperEvent.LEFT
implies SafetyController.bumper_left_pressed_ becomes False
```

17. Topic `events/bumper`
```python
receive on "events/bumper"
  and msg.state is BumperEvent.RELEASED
  and msg.bumper is BumperEvent.CENTER
implies SafetyController.bumper_center_pressed_ becomes False
```

18. Topic `events/bumper`
```python
receive on "events/bumper"
  and msg.state is BumperEvent.RELEASED
  and msg.bumper is BumperEvent.RIGHT
implies SafetyController.bumper_right_pressed_ becomes False
```

19. Topic `events/wheel_drop`
```python
receive on "events/wheel_drop"
  and msg.state is WheelDropEvent.DROPPED
  and msg.wheel is WheelDropEvent.LEFT
implies SafetyController.wheel_left_dropped_ becomes True
```

20. Topic `events/wheel_drop`
```python
receive on "events/wheel_drop"
  and msg.state is WheelDropEvent.DROPPED
  and msg.wheel is WheelDropEvent.RIGHT
implies SafetyController.wheel_right_dropped_ becomes True
```

21. Topic `events/wheel_drop`
```python
receive on "events/wheel_drop"
  and msg.state is WheelDropEvent.RAISED
  and msg.wheel is WheelDropEvent.LEFT
implies SafetyController.wheel_left_dropped_ becomes False
```

22. Topic `events/wheel_drop`
```python
receive on "events/wheel_drop"
  and msg.state is WheelDropEvent.RAISED
  and msg.wheel is WheelDropEvent.RIGHT
implies SafetyController.wheel_right_dropped_ becomes False
```

23. Topic `events/wheel_drop`
```python
receive on "reset"
implies SafetyController.wheel_left_dropped_ becomes False
  and SafetyController.wheel_right_dropped_ becomes False
  and SafetyController.bumper_left_pressed_ becomes False
  and SafetyController.bumper_center_pressed_ becomes False
  and SafetyController.bumper_right_pressed_ becomes False
  and SafetyController.cliff_left_detected_ becomes False
  and SafetyController.cliff_center_detected_ becomes False
  and SafetyController.cliff_right_detected_ becomes False
```

# Publishers

1. Topic `cmd_vel`
```python
call on SafetyController.spin()
  and SafetyController.getState() is True
  and SafetyController.wheel_left_dropped_ is True
      or SafetyController.wheel_right_dropped_ is True
implies SafetyController.msg_ becomes new geometry_msgs.Twist
  and SafetyController.msg_.linear.x becomes 0.0
  and SafetyController.msg_.angular.z becomes 0.0
  and publish on "cmd_vel"
      with SafetyController.msg_ as msg
```

2. Topic `cmd_vel`
```python
call on SafetyController.spin()
  and SafetyController.getState() is True
  and SafetyController.wheel_left_dropped_ is False
  and SafetyController.wheel_right_dropped_ is False
  and SafetyController.bumper_center_pressed_ is True
      or SafetyController.cliff_center_detected_ is True
implies SafetyController.msg_ becomes new geometry_msgs.Twist
  and SafetyController.msg_.linear.x becomes -0.1
  and SafetyController.msg_.angular.z becomes 0.0
  and publish on "cmd_vel"
      with SafetyController.msg_ as msg
```

3. Topic `cmd_vel`
```python
call on SafetyController.spin()
  and SafetyController.getState() is True
  and SafetyController.wheel_left_dropped_ is False
  and SafetyController.wheel_right_dropped_ is False
  and SafetyController.bumper_center_pressed_ is False
  and SafetyController.cliff_center_detected_ is False
  and SafetyController.bumper_left_pressed_ is True
      or SafetyController.cliff_left_detected_ is True
implies SafetyController.msg_ becomes new geometry_msgs.Twist
  and SafetyController.msg_.linear.x becomes -0.1
  and SafetyController.msg_.angular.z becomes -0.4
  and publish on "cmd_vel"
      with SafetyController.msg_ as msg
```

4. Topic `cmd_vel`
```python
call on SafetyController.spin()
  and SafetyController.getState() is True
  and SafetyController.wheel_left_dropped_ is False
  and SafetyController.wheel_right_dropped_ is False
  and SafetyController.bumper_center_pressed_ is False
  and SafetyController.cliff_center_detected_ is False
  and SafetyController.bumper_left_pressed_ is False
  and SafetyController.cliff_left_detected_ is False
  and SafetyController.bumper_right_pressed_ is True
      or SafetyController.cliff_right_detected_ is True
implies SafetyController.msg_ becomes new geometry_msgs.Twist
  and SafetyController.msg_.linear.x becomes -0.1
  and SafetyController.msg_.angular.z becomes 0.4
  and publish on "cmd_vel"
      with SafetyController.msg_ as msg
```

5. Topic `cmd_vel`
```python
call on SafetyController.spin()
  and SafetyController.getState() is True
  and SafetyController.wheel_left_dropped_ is False
  and SafetyController.wheel_right_dropped_ is False
  and SafetyController.bumper_center_pressed_ is False
  and SafetyController.cliff_center_detected_ is False
  and SafetyController.bumper_left_pressed_ is False
  and SafetyController.cliff_left_detected_ is False
  and SafetyController.bumper_right_pressed_ is False
  and SafetyController.cliff_right_detected_ is False
  and SafetyController.time_to_extend_bump_cliff_events_ > 1e-10
  and SafetyController.time_to_extend_bump_cliff_events
      < Time.now() - SafetyController.last_event_time_
implies publish on "cmd_vel"
    with SafetyController.msg_ as msg
```

6. Topic `cmd_vel`
```python
publish on "cmd_vel"
implies msg.linear.x >= -0.1
  and msg.linear.x <= 0.0
  and msg.angular.z >= -0.4
  and msg.angular.z <= 0.4
```

# Composite Properties

1. Topic `cmd_vel` relative to subscribers
```python
let event be last of
    (last receive on "events/bumper"
      where msg.state is BumperEvent.PRESSED
      and msg.bumper is BumperEvent.LEFT
    or last receive on "events/cliff"
      where msg.state is CliffEvent.CLIFF
      and msg.sensor is CliffEvent.LEFT)
publish on "cmd_vel"
  and msg.linear.x is -0.1
  and msg.angular.z is -0.4
implies SafetyControllerNodelet.shutdown_requested_ is False
  and ROS.ok() is True
  and last receive on "disable" precedes last receive on "enable"
  and last receive on "enable" precedes call on SafetyController.spin()
  and last receive on "reset" precedes event
  and event precedes call on SafetyController.spin()
  and last receive on "events/wheel_drop"
      where msg.state is WheelDropEvent.RAISED
      and msg.wheel is WheelDropEvent.LEFT
    precedes call on SafetyController.spin()
  and last receive on "events/wheel_drop"
      where msg.state is WheelDropEvent.RAISED
      and msg.wheel is WheelDropEvent.RIGHT
    precedes call on SafetyController.spin()
  and no receive on "events/wheel_drop"
      where msg.state is WheelDropEvent.DROPPED
      and msg.wheel is WheelDropEvent.LEFT
    since last receive on "events/wheel_drop"
      where msg.state is WheelDropEvent.RAISED
      and msg.wheel is WheelDropEvent.LEFT
  and no receive on "events/wheel_drop"
      where msg.state is WheelDropEvent.DROPPED
      and msg.wheel is WheelDropEvent.RIGHT
    since last receive on "events/wheel_drop"
      where msg.state is WheelDropEvent.RAISED
      and msg.wheel is WheelDropEvent.RIGHT
  and no receive on "events/bumper"
      where msg.state is BumperEvent.PRESSED
      and msg.bumper is BumperEvent.CENTER
    since last receive on "events/bumper"
      where msg.state is BumperEvent.RELEASED
      and msg.bumper is BumperEvent.CENTER
  and no receive on "events/cliff"
      where msg.state is CliffEvent.CLIFF
      and msg.sensor is CliffEvent.CENTER
    since last receive on "events/cliff"
      where msg.state is CliffEvent.FLOOR
      and msg.sensor is CliffEvent.CENTER
```

# Multiplexer Subscribers

1. Message forwarding
```python
let allowed be CmdVelMuxNodelet.cmd_vel_subs.allowed
call on CmdVelMuxNodelet.cmdVelCallback(geometry_msgs/Twist message, int index)
  and allowed is CmdVelMuxNodelet.VACANT
      or allowed is index
      or CmdVelMuxNodelet.cmd_vel_subs[index].priority
          > CmdVelMuxNodelet.cmd_vel_subs[allowed].priority
implies publish on "output"
    with msg as message
```

2. Topic `active`
```python
let allowed be CmdVelMuxNodelet.cmd_vel_subs.allowed
call on CmdVelMuxNodelet.cmdVelCallback(geometry_msgs/Twist message, int index)
  and allowed is CmdVelMuxNodelet.VACANT
      or allowed is index
      or CmdVelMuxNodelet.cmd_vel_subs[index].priority
          > CmdVelMuxNodelet.cmd_vel_subs[allowed].priority
  and not allowed is index
implies allowed becomes index
  and publish on "active"
      where msg.data is CmdVelMuxNodelet.cmd_vel_subs[index].name
```
