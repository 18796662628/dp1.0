function probe_contact_geometry_ports()
% Inspect the geometry ports without altering or simulating a contact model.

model_name = "contact_port_probe";
build_simscape_semi_steer_model(model_name);

solid_names = ["Ground", "FrontLeftWheel", "RearRightWheel", ...
    "FrontRightSupportWheel", "RearLeftSupportWheel"];
for index = 1:numel(solid_names)
    block_path = model_name + "/" + solid_names(index);
    set_param(block_path, "ExportEntireGeometry", "on");
end
set_param(model_name, "SimulationCommand", "update");

for index = 1:numel(solid_names)
    block_path = model_name + "/" + solid_names(index);
    fprintf("\n%s\n", block_path);
    fprintf("ExportEntireGeometry = %s\n", ...
        get_param(block_path, "ExportEntireGeometry"));
    disp(get_param(block_path, "PortHandles"));
end
end
