function probe_motor_actuation_ports()
% Inspect the exact physical-signal port created by wheel torque actuation.
% No contact model is changed and no simulation is run.

model_name = "motor_port_probe";
build_simscape_semi_steer_model(model_name);

joint_names = ["FrontLeftWheelSpin", "RearRightWheelSpin"];
for index = 1:numel(joint_names)
    set_param(model_name + "/" + joint_names(index), ...
        "TorqueActuationMode", "InputTorque");
end
add_block("nesl_utility/Simulink-PS Converter", ...
    model_name + "/TorqueConverterProbe", "Position", [820 680 880 720]);
set_param(model_name, "SimulationCommand", "update");

for index = 1:numel(joint_names)
    block_path = model_name + "/" + joint_names(index);
    fprintf("\n%s\n", block_path);
    disp(get_param(block_path, "PortHandles"));
end
fprintf("\nTorqueConverterProbe\n");
disp(get_param(model_name + "/TorqueConverterProbe", "PortHandles"));
end
