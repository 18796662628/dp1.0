function build_simscape_contact_model()
% Build a separate 3-D wheel/ground contact experiment. The original model
% remains unchanged.

model_name = "semi_steer_contact_model";
build_simscape_semi_steer_model(model_name);

% The 6-DOF base lets gravity and tire contact support the chassis vertically.
delete_block(model_name + "/PlanarBase");
add_block("sm_lib/Joints/6-DOF Joint", model_name + "/FreeBase", ...
    "Position", [130 400 180 450]);
add_line(model_name, first_port(model_name + "/World", "RConn"), ...
    first_port(model_name + "/FreeBase", "LConn"), "autorouting", "on");
add_line(model_name, first_port(model_name + "/FreeBase", "RConn"), ...
    first_port(model_name + "/ChassisPose", "LConn"), "autorouting", "on");

solid_names = ["Ground", "FrontLeftWheel", "RearRightWheel", ...
    "FrontRightSupportWheel", "RearLeftSupportWheel"];
for index = 1:numel(solid_names)
    set_param(model_name + "/" + solid_names(index), "ExportEntireGeometry", "on");
end
set_param(model_name + "/FrontLeftWheelSpin", "TorqueActuationMode", "InputTorque");
set_param(model_name + "/RearRightWheelSpin", "TorqueActuationMode", "InputTorque");
configure_steer_servo(model_name + "/FrontLeftSteer", +20);
configure_steer_servo(model_name + "/RearRightSteer", -20);
set_param(model_name, "Solver", "ode15s", "MaxStep", "1e-3");
set_param(model_name, "SimulationCommand", "update");

% Active wheels have ordinary tire friction; passive omni-wheel placeholders
% have low friction because their rollers are not yet modeled.
add_wheel_contact(model_name, "FrontLeftContact", "FrontLeftWheel", 0.80, [870 570 930 630]);
add_wheel_contact(model_name, "RearRightContact", "RearRightWheel", 0.80, [870 160 930 220]);
add_wheel_contact(model_name, "FrontRightSupportContact", "FrontRightSupportWheel", 0.03, [520 190 580 250]);
add_wheel_contact(model_name, "RearLeftSupportContact", "RearLeftSupportWheel", 0.03, [520 470 580 530]);

% M3508 abstraction. Keep torque at zero while checking steering alone.
add_drive_torque(model_name, "FrontLeft", 0.00, [820 680 875 710], [920 680 965 710]);
add_drive_torque(model_name, "RearRight", 0.00, [820 80 875 110], [920 80 965 110]);

save_system(model_name);
open_system(model_name);
end

function add_drive_torque(model_name, wheel_name, torque_n_m, source_position, converter_position)
source_path = model_name + "/" + wheel_name + "DriveTorque";
converter_path = model_name + "/" + wheel_name + "TorquePS";
joint_path = model_name + "/" + wheel_name + "WheelSpin";

add_block("simulink/Sources/Constant", source_path, ...
    "Value", num2str(torque_n_m), "Position", source_position);
add_block("nesl_utility/Simulink-PS Converter", converter_path, ...
    "Unit", "N*m", "Position", converter_position);
add_line(model_name, first_port(source_path, "Outport"), ...
    first_port(converter_path, "Inport"), "autorouting", "on");
add_line(model_name, first_port(converter_path, "RConn"), ...
    port_at(joint_path, "LConn", 2), "autorouting", "on");
end

function configure_steer_servo(joint_path, target_deg)
% Torque-based stand-in for a GM6020 position loop. Unlike prescribed motion,
% it remains physically compatible with a free chassis and wheel contact.
set_param(joint_path, "EquilibriumPosition", num2str(target_deg));
set_param(joint_path, "SpringStiffness", "0.04");
set_param(joint_path, "DampingCoefficient", "0.05");
end

function add_wheel_contact(model_name, contact_name, wheel_name, friction, position)
contact_path = model_name + "/" + contact_name;
add_block("sm_lib/Forces and Torques/Spatial Contact Force", contact_path, ...
    "Position", position, ...
    "NormalForceType", "SmoothSpringDamper", ...
    "NormalStiffness", "5e4", ...
    "NormalDamping", "600", ...
    "NormalTransitionRegionWidth", "1e-4", ...
    "FrictionType", "SmoothStickSlip", ...
    "CoefficientOfStaticFriction", num2str(friction), ...
    "CoefficientOfDynamicFriction", num2str(friction * 0.75), ...
    "FrictionalCriticalVelocity", "1e-3");

ground_geometry = exported_geometry_port(model_name + "/Ground");
wheel_geometry = exported_geometry_port(model_name + "/" + wheel_name);
base_geometry = first_port(contact_path, "LConn");
follower_geometry = first_port(contact_path, "RConn");
add_line(model_name, ground_geometry, base_geometry, "autorouting", "on");
add_line(model_name, wheel_geometry, follower_geometry, "autorouting", "on");
end

function handle = exported_geometry_port(block_path)
% In this MATLAB release, RConn is the solid reference frame and LConn is
% the geometry port created by ExportEntireGeometry.
port_handles = get_param(block_path, "PortHandles");
if ~isfield(port_handles, "LConn") || isempty(port_handles.LConn)
    error("Geometry port was not exported from '%s'.", block_path);
end
handle = port_handles.LConn(1);
end

function handle = first_port(block_path, side)
handle = port_at(block_path, side, 1);
end

function handle = port_at(block_path, side, index)
port_handles = get_param(block_path, "PortHandles");
handle = port_handles.(side)(index);
end
