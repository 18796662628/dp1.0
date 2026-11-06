function probe_contact_library()
log_file = fullfile(fileparts(mfilename("fullpath")), "contact_library_probe.txt");
fid = fopen(log_file, "w");
cleanup = onCleanup(@() fclose(fid));

write_line(fid, "start");
try
    load_system("sm_lib");
    write_line(fid, "library loaded");
    blocks = find_system("sm_lib", "LookUnderMasks", "all", ...
        "FollowLinks", "on", "Name", "Planar Contact Force");
    write_line(fid, "block path:");
    for index = 1:numel(blocks)
        write_line(fid, blocks{index});
    end

    if isempty(blocks)
        error("Planar Contact Force block was not found.");
    end

    if bdIsLoaded("contact_probe")
        close_system("contact_probe", 0);
    end
    new_system("contact_probe");
    add_block(blocks{1}, "contact_probe/Contact", "Position", [100 100 250 230]);

    write_line(fid, "");
    write_line(fid, "port connectivity:");
    ports = get_param("contact_probe/Contact", "PortConnectivity");
    for index = 1:numel(ports)
        write_line(fid, sprintf("port %d: type=%s", index, ports(index).Type));
    end

    write_line(fid, "");
    write_line(fid, "dialog parameters:");
    parameters = get_param("contact_probe/Contact", "DialogParameters");
    names = fieldnames(parameters);
    for index = 1:numel(names)
        write_line(fid, names{index});
    end
    close_system("contact_probe", 0);
    write_line(fid, "probe complete");
catch problem
    if bdIsLoaded("contact_probe")
        close_system("contact_probe", 0);
    end
    write_line(fid, "");
    write_line(fid, "ERROR:");
    write_line(fid, getReport(problem, "extended", "hyperlinks", "off"));
end

fprintf("Contact probe complete. Read: %s\n", log_file);
end

function write_line(fid, text)
fprintf("%s\n", text);
fprintf(fid, "%s\n", text);
end
