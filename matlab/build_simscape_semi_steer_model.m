function build_simscape_semi_steer_model(model_name)
% Build a simplified Simscape Multibody mechanism for the semi-steer chassis.
% This model includes estimated rigid-body mass and inertia. Tire contact and
% friction are still omitted so the mechanism remains easy to inspect.

if nargin < 1
    model_name = "semi_steer_multibody_model";
end
model_dir = fileparts(mfilename("fullpath"));
model_file = fullfile(model_dir, model_name + ".slx");
old_dir = pwd;
cleanup = onCleanup(@() cd(old_dir));
cd(model_dir);

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if isfile(model_file)
    delete(model_file);
end

% Estimated masses in kilograms. Replace with measured values on the real car.
chassis_mass_kg = 11.0;
steering_column_mass_kg = 0.25;
active_wheel_mass_kg = 0.35;
support_wheel_mass_kg = 0.25;

mb = simscape.multibody.Multibody;
mb.Gravity = simscape.Value([0 0 -9.81], "m/s^2");

addComponent(mb, "World", simscape.multibody.WorldFrame);
addComponent(mb, "PlanarBase", damped_planar_joint());
addComponent(mb, "ChassisPose", rigid_pose([0 0 0.13], 0, [0 0 1]));
addComponent(mb, "Chassis", solid_cylinder(0.30, 0.08, chassis_mass_kg, [0.68 0.75 0.90]));
addComponent(mb, "GroundPose", rigid_pose([0 0 -0.01], 0, [0 0 1]));
addComponent(mb, "Ground", solid_brick([4 4 0.02], 1, [0.82 0.82 0.82]));

connect(mb, "World/W", "PlanarBase/B");
connect(mb, "PlanarBase/F", "ChassisPose/B");
connect(mb, "ChassisPose/F", "Chassis/R");
connectVia(mb, "GroundPose", "World/W", "Ground/R");

half_base = 0.344 / 2;
half_track = 0.344 / 2;

add_steering_module(mb, "FrontLeft", +half_base, +half_track, ...
    steering_column_mass_kg, active_wheel_mass_kg);
add_steering_module(mb, "RearRight", -half_base, -half_track, ...
    steering_column_mass_kg, active_wheel_mass_kg);
add_support_wheel(mb, "FrontRightSupport", +half_base, -half_track, support_wheel_mass_kg);
add_support_wheel(mb, "RearLeftSupport", -half_base, +half_track, support_wheel_mass_kg);

makeBlockDiagram(mb, simscape.op.OperatingPoint, model_name);
save_system(model_name, model_file);
open_system(model_name);
end

function add_steering_module(mb, name, x, y, column_mass_kg, wheel_mass_kg)
% The steering joint sits directly above the chassis top surface.
addComponent(mb, name + "Anchor", rigid_pose([x y 0.02], 0, [0 0 1]));
addComponent(mb, name + "Steer", damped_revolute_joint(0.010));
% GM6020 visual: original radius with a shortened height.
addComponent(mb, name + "Column", solid_cylinder(0.035, 0.08, column_mass_kg, [0.35 0.42 0.55]));
% All four abstract wheels use radius 0.05 m.  The steering anchor is
% z=0.02 m above the chassis reference, so -0.10 m puts this axle at z=0.05 m.
addComponent(mb, name + "WheelOffset", rigid_pose([0 0 -0.10], 0, [0 0 1]));
% Rotate the local frame before the wheel-spin joint. This makes the
% WheelSpin axis horizontal (the M3508 wheel axle), not vertical.
addComponent(mb, name + "WheelAlign", rigid_pose([0 0 0], 90, [1 0 0]));
addComponent(mb, name + "WheelSpin", damped_revolute_joint(0.002));
addComponent(mb, name + "Wheel", solid_cylinder(0.050, 0.040, wheel_mass_kg, [0.08 0.08 0.08]));

connect(mb, "Chassis/R", name + "Anchor/B");
connect(mb, name + "Anchor/F", name + "Steer/B");
connect(mb, name + "Steer/F", name + "Column/R");
connect(mb, name + "Steer/F", name + "WheelOffset/B");
connect(mb, name + "WheelOffset/F", name + "WheelAlign/B");
connect(mb, name + "WheelAlign/F", name + "WheelSpin/B");
connect(mb, name + "WheelSpin/F", name + "Wheel/R");
end

function add_support_wheel(mb, name, x, y, wheel_mass_kg)
% With the chassis reference at 0.13 m, this puts the wheel center at 0.05 m.
addComponent(mb, name + "Pose", rigid_pose([x y -0.08], 90, [1 0 0]));
addComponent(mb, name + "Wheel", solid_cylinder(0.050, 0.030, wheel_mass_kg, [0.30 0.30 0.30]));
connect(mb, "Chassis/R", name + "Pose/B");
connect(mb, name + "Pose/F", name + "Wheel/R");
end

function component = rigid_pose(offset_m, angle_deg, axis)
rotation = simscape.multibody.ArbitraryAxisRotation( ...
    simscape.Value(angle_deg, "deg"), axis);
translation = simscape.multibody.CartesianTranslation( ...
    simscape.Value(offset_m, "m"));
component = simscape.multibody.RigidTransform(rotation, translation);
end

function joint = damped_planar_joint()
% A small linear and yaw damping approximates rolling resistance before
% wheel-ground contact forces are introduced.
joint = simscape.multibody.PlanarJoint;
joint.Px.ForceLaws = axial_damper(0.40);
joint.Py.ForceLaws = axial_damper(0.40);
joint.Rz.ForceLaws = torsional_damper(0.020);
end

function joint = damped_revolute_joint(damping_n_m_per_rad_s)
joint = simscape.multibody.RevoluteJoint;
joint.Rz.ForceLaws = torsional_damper(damping_n_m_per_rad_s);
end

function damper = axial_damper(damping_n_per_m_s)
damper = simscape.multibody.AxialSpringDamper;
damper.DampingCoefficient = simscape.Value(damping_n_per_m_s, "N/(m/s)");
end

function damper = torsional_damper(damping_n_m_per_rad_s)
damper = simscape.multibody.TorsionalSpringDamper;
damper.DampingCoefficient = simscape.Value(damping_n_m_per_rad_s, "N*m/(rad/s)");
end

function component = solid_cylinder(radius_m, length_m, mass_kg, color)
geometry = simscape.multibody.Cylinder(simscape.Value(radius_m, "m"), ...
    simscape.Value(length_m, "m"));
inertia = simscape.multibody.UniformMass(simscape.Value(mass_kg, "kg"));
visual = simscape.multibody.SimpleVisualProperties(color);
component = simscape.multibody.Solid(geometry, inertia, visual);
end

function component = solid_brick(size_m, mass_kg, color)
geometry = simscape.multibody.Brick(simscape.Value(size_m, "m"));
inertia = simscape.multibody.UniformMass(simscape.Value(mass_kg, "kg"));
visual = simscape.multibody.SimpleVisualProperties(color);
component = simscape.multibody.Solid(geometry, inertia, visual);
end
