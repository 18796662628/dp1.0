# Semi-Steer Chassis Integration

The mechanical layout has two powered steering modules and two passive caster
wheels. Only the powered steering modules need control software.

Before enabling motors, record these values in the chassis configuration:

1. Drive-motor type, CAN bus, CAN ID, reduction ratio, and wheel radius.
2. Steering-motor type, CAN bus, CAN ID, reduction ratio, and encoder zero.
3. Each powered wheel center in the body frame: `(x_i, y_i)` in meters.
4. Motor directions and the steering angle that corresponds to forward motion.

For a requested body velocity `(vx, vy, wz)`, the velocity at steering module
`i` is:

```
vix = vx - wz * yi
viy = vy + wz * xi
angle_i = atan2(viy, vix)
speed_i = sqrt(vix * vix + viy * viy)
```

The final implementation will add steering-angle optimization, drive-speed sign
selection, steering and drive motor PID settings, limits, and offline handling.
