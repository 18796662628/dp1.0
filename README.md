# Semi-Steer Chassis

STM32F407 firmware baseline for a two-steering-wheel, two-passive-caster
chassis. The active application is intentionally limited to remote control,
safety handling, chassis commands, CAN support, and DJI motor control.

The current baseline registers no motors and sends no motor commands. This is
intentional until the motor types, CAN IDs, steering encoder zero positions,
wheel radius, reduction ratio, and wheel coordinates are measured on hardware.

## Active Flow

```
remote control -> robot_cmd -> chassis_cmd -> chassis
                                            -> motor control (no motors yet)
```

`application/robot_def.h` contains command limits and coordinate conventions.
`application/chassis/` is the only application area intended for the upcoming
semi-steer motor registration and kinematics.
