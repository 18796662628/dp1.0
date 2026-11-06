function animate_semi_steer_3d(pose, steering_actual)
% Animate the abstract semi-steer chassis in 3-D from Simulink results.

t = pose.Time;
state = squeeze(pose.Data);
steer_deg = squeeze(steering_actual.Data);

wheel_base = 0.344;
track_width = 0.344;
visual_scale = 2.0;
active_positions = visual_scale * [wheel_base/2, track_width/2; -wheel_base/2, -track_width/2];
support_positions = visual_scale * [wheel_base/2, -track_width/2; -wheel_base/2, track_width/2];

figure("Name", "Semi-steer chassis 3-D animation", "Color", "w");
axis equal
axis([-1.0 3.0 -1.0 3.0 0 0.9])
grid on
view(42, 25)
xlabel("World x (m)")
ylabel("World y (m)")
zlabel("Height (m)")
hold on

for k = 1:10:numel(t)
    cla
    x = state(k,1);
    y = state(k,2);
    yaw = state(k,3);
    body_rotation = rotz3(yaw);

    draw_cylinder([x; y; 0.20], body_rotation, 0.60, 0.12, [0.68 0.75 0.90]);
    draw_arrow([x; y; 0.32], body_rotation * [0.55; 0; 0], [0.90 0.12 0.10]);

    for i = 1:2
        local_position = [active_positions(i,:), 0.16]';
        center = [x; y; 0] + body_rotation * local_position;
        steer_rotation = body_rotation * rotz3(deg2rad(steer_deg(k,i)));

        % Green arrow marks the mechanical zero direction of each steering module.
        draw_arrow(center + [0; 0; 0.38], body_rotation * [0.28; 0; 0], ...
            [0.10 0.55 0.20]);
        % Keep only the upper half of the steering column above the wheel.
        draw_cylinder(center + [0; 0; 0.30], steer_rotation, 0.07, 0.20, ...
            [0.35 0.42 0.55]);
        draw_wheel(center + [0; 0; 0.11], steer_rotation);
    end

    for i = 1:2
        local_position = [support_positions(i,:), 0.10]';
        center = [x; y; 0] + body_rotation * local_position;
        support_rotation = body_rotation * rotz3(deg2rad(steer_deg(k,i)));
        draw_wheel(center + [0; 0; 0.11], support_rotation);
    end

    plot3(state(1:k,1), state(1:k,2), 0.01*ones(k,1), ...
        "Color", [0.45 0.45 0.45], "LineWidth", 1);
    title(sprintf("Semi-steer chassis 3-D: t = %.2f s", t(k)));
    drawnow
    pause(0.015)
end
end

function draw_wheel(center, steering_rotation)
% A cylinder's default axis is z. Rotate it once into the local y axis;
% after that it only follows the steering-column rotation around local z.
wheel_mount_rotation = steering_rotation * rotx3(-pi/2);
draw_cylinder(center, wheel_mount_rotation, 0.11, 0.080, ...
    [0.08 0.08 0.08]);
end

function draw_cylinder(center, rotation, radius, height, color)
[cx, cy, cz] = cylinder(radius, 20);
cz = (cz - 0.5) * height;
points = rotation * [cx(:)'; cy(:)'; cz(:)'];
sx = reshape(points(1,:) + center(1), size(cx));
sy = reshape(points(2,:) + center(2), size(cy));
sz = reshape(points(3,:) + center(3), size(cz));
surf(sx, sy, sz, "FaceColor", color, "EdgeColor", "none");
end

function draw_arrow(origin, vector, color)
quiver3(origin(1), origin(2), origin(3), vector(1), vector(2), vector(3), ...
    0, "Color", color, "LineWidth", 2, "MaxHeadSize", 0.55);
end

function rotation = rotz3(angle)
rotation = [cos(angle), -sin(angle), 0; sin(angle), cos(angle), 0; 0, 0, 1];
end

function rotation = rotx3(angle)
rotation = [1, 0, 0; 0, cos(angle), -sin(angle); 0, sin(angle), cos(angle)];
end
