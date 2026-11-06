function run_simscape_contact_model()
% Generate and run the independent 3-D wheel-ground contact experiment.

build_simscape_contact_model();
sim("semi_steer_contact_model", "StopTime", "2");
end
