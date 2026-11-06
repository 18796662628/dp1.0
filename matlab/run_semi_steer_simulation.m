function run_semi_steer_simulation()
% Build, simulate, and animate the stage-one semi-steer model.

build_semi_steer_simulink_model();
simulation_output = sim("semi_steer_kinematic_model", ...
    "ReturnWorkspaceOutputs", "on");
pose = simulation_output.get("pose");
steering_actual = simulation_output.get("steering_actual");
animate_semi_steer_3d(pose, steering_actual);
end
