function run_simscape_semi_steer_model()
% Generate, simulate, and open the Mechanics Explorer for the 3-D mechanism.

build_simscape_semi_steer_model();
sim("semi_steer_multibody_model", "StopTime", "1");
end
