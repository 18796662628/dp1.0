# Application Layer

Only two application modules are active:

- `cmd/robot_cmd.c` owns remote-control parsing, the safety state machine, and
  publishing `chassis_cmd`.
- `chassis/chassis.c` owns the future semi-steer motor configuration and
  kinematic allocation.

The command uses the chassis body frame: `+x` forward, `+y` left, and positive
`wz` counterclockwise. Translation is measured in m/s and yaw rate in rad/s.

The system starts in emergency stop. After valid remote data arrives, hold both
remote switches in the middle for 500 ms to arm. Both switches down, an invalid
switch value, or a remote timeout immediately disables chassis commands.
