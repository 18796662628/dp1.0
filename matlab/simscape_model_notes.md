# Simplified Simscape Multibody Model

Run `run_simscape_semi_steer_model` from the `matlab` folder to generate the
`semi_steer_multibody_model.slx` model and open it in Simulink.

The model contains a circular chassis connected to the world by a Planar Joint,
two active steering modules at the front-left and rear-right corners, and two
fixed passive support wheels at the other corners. Each active module has a
vertical Revolute Joint for steering and a second Revolute Joint for wheel spin.

Gravity is enabled. Estimated masses are 11 kg for the chassis, 0.25 kg per
steering column, 0.35 kg per active wheel, and 0.25 kg per support wheel.
Simscape calculates the matching moments of inertia from each cylinder's
geometry and uniform mass distribution.

Tire contact and friction are still disabled. The Planar Joint holds the chassis
at a fixed height. Small damping values are included in planar translation,
planar yaw, steering, and wheel-spin joints to approximate rolling and bearing
losses. This is still a gravity-and-inertia mechanism model rather than a
tire-force dynamics model. The existing `semi_steer_kinematic_model.slx`
remains the model that tests current velocity commands and steering kinematics.

## Wheel-Ground Contact Experiment

Run `run_simscape_contact_model` to create `semi_steer_contact_model.slx`.
This is a separate experimental model. It replaces the Planar Joint with a
6-DOF joint and adds four Spatial Contact Force blocks. The active wheels use
estimated static friction 0.80; the passive omni-wheel placeholders use 0.03
because their rollers are not modeled. Contact stiffness and damping are
provisional.

Before adding the drive input below, this version checks gravity, ground
support, and contact geometry without driving the chassis.

## M3508 Equivalent Drive Input

The contact model now adds a Constant block and Simulink-PS Converter for each
active wheel. These feed the torque input of `FrontLeftWheelSpin` and
`RearRightWheelSpin`. The default value is 0.30 N*m at each wheel axis. It is
an estimated equivalent wheel torque, not a measured M3508 motor-shaft value.
Change the values in the two `*DriveTorque` Constant blocks to reverse or tune
the test motion.

## GM6020 Steering Check

The two GM6020 steering joints use an internal torsional spring-damper as a
torque-based stand-in for position control. The front-left target is +20 degrees
and rear-right target is -20 degrees. Both drive-torque constants are 0 N*m for
this isolated steering test. A later version can replace this approximation with
encoder feedback and a Simulink PID that commands GM6020 torque.
