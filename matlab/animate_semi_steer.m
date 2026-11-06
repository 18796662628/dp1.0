function animate_semi_steer(pose, steering_actual)
% Animate the abstract semi-steer chassis after running the Simulink model.

t = pose.Time;
state = squeeze(pose.Data);
steer_deg = squeeze(steering_actual.Data);

wheel_base = 0.344;
track_width = 0.344;
wheel_points = [wheel_base/2, track_width/2; -wheel_base/2, -track_width/2];
support_points = [wheel_base/2, -track_width/2; -wheel_base/2, track_width/2];
chassis_radius = 0.30;
circle_angle = linspace(0, 2*pi, 48);

figure("Name", "Semi-steer chassis animation", "Color", "w");
axis equal
axis([-1 3 -1 3])
grid on
hold on

for k = 1:10:numel(t)
    cla
    x = state(k,1);
    y = state(k,2);
    yaw = state(k,3);
    rotation = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    chassis = rotation * [chassis_radius*cos(circle_angle); chassis_radius*sin(circle_angle)];
    fill(x + chassis(1,:), y + chassis(2,:), [0.75 0.80 0.90], "EdgeColor", "k");
    plot(state(1:k,1), state(1:k,2), "Color", [0.55 0.55 0.55], "LineWidth", 1);

    for i = 1:2
        location = rotation * wheel_points(i,:)';
        world_angle = yaw + deg2rad(steer_deg(k,i));
        direction = [cos(world_angle); sin(world_angle)];
        center = [x; y] + location;
        zero_direction = rotation * [0.12; 0];
        quiver(center(1), center(2), zero_direction(1), zero_direction(2), 0, ...
            "Color", [0.1 0.55 0.2], "LineWidth", 1.5, "MaxHeadSize", 0.8);
        plot(center(1) + [-0.10 0.10] * direction(1), ...
            center(2) + [-0.10 0.10] * direction(2), "k", "LineWidth", 7);
    end

    for i = 1:2
        location = rotation * support_points(i,:)';
        center = [x; y] + location;
        plot(center(1), center(2), "o", "MarkerFaceColor", [0.4 0.4 0.4], ...
            "MarkerEdgeColor", "k", "MarkerSize", 10);
    end

    quiver(x, y, 0.25*cos(yaw), 0.25*sin(yaw), 0, "r", "LineWidth", 2);
    title(sprintf("t = %.2f s", t(k)));
    drawnow
    pause(0.015)
end
end
