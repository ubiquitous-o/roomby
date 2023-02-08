# mofidied_teleop_twist_keyboard
Added song function to [telepo_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard)

**This package depends on [create_robot kinetic ver.](https://github.com/AutonomyLab/create_robot/tree/kinetic)**

# Launch
## ROS launch.
```
roslaunch ca_driver create_2.launch
```

## Run.
```
rosrun modified_teleop_twist_keyboard teleop_twist_keyboard.py _repeat_rate:=10.0 _key_timeout:=0.5
```

With custom values.
```
rosrun modified_teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.9 _turn:=0.8
```

Publishing to a different topic (in this case `my_cmd_vel`).
```
rosrun modified_teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=my_cmd_vel
```

# Usage
```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j         l
   m    k    . 

anything else : stop

q/z : high speed mode/ low speed mode
c: circle move mode

Playing song(select roomba talking word)
---------------------------
   1,2,3,4

talking word meanings
1 : "OK"
2 : "!!"
3 : "??"
4 : "yeah!"

CTRL-C to quit
```
## Change song set
If you want to change song set, see the lines 108-116.
Please see ["Song Chapter" of this specification](https://cdn-shop.adafruit.com/datasheets/create_2_Open_Interface_Spec.pdf) for more details about song definitions.


# Repeat Rate

If your mobile base requires constant updates on the cmd\_vel topic, teleop\_twist\_keyboard can be configured to repeat the last command at a fixed interval, using the `repeat_rate` private parameter.

For example, to repeat the last command at 10Hz:

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _repeat_rate:=10.0
```

It is _highly_ recommened that the repeat rate be used in conjunction with the key timeout, to prevent runaway robots.

# Key Timeout

Teleop\_twist\_keyboard can be configured to stop your robot if it does not receive any key presses in a configured time period, using the `key_timeout` private parameter.

For example, to stop your robot if a keypress has not been received in 0.6 seconds:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _key_timeout:=0.6
```

It is recommended that you set `key_timeout` higher than the initial key repeat delay on your system (This delay is 0.5 seconds by default on Ubuntu, but can be adjusted).

# Twist with header
Publishing a `TwistStamped` message instead of `Twist` can be enabled with the `stamped` private parameter. Additionally the `frame_id` of the `TwistStamped` message can be set with the `frame_id` private parameter.
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _stamped:=True _frame_id:=base_link
```
