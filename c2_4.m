clc;
clear;
close all;

%% Robot arm parameters
L1 = 2; % Length of the first arm segment
L2 = 2; % Length of the second arm segment
L3 = 2; % Length of the third arm segment
gripper_width = 2; % Distance between gripper fingers
gripper_length = 1; % Length of the gripper fingers
dt = 0.1; % Time step
t_total = 10; % Time for a single transport task
num_steps = t_total / dt;
num_targets = 10; % Number of target points

%% Object and initial position
obj_start = [5, 0, 0]; % Initial position of the object
pickup_height = 8; % Initial lifting height of the robotic arm

%% Generate 5 random target points within the (-10,10) interval
targets = [randi([-10, 10], num_targets, 1), ...
           randi([-10, 10], num_targets, 1), ...
           zeros(num_targets, 1)]; % Target points are on the xy-plane

%% Initialize 3D scene
figure;
hold on;
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
xlim([-15, 15]);
ylim([-15, 15]);
zlim([0, 15]);
view(3);

%% Mark all target points
for t = 1:num_targets
    draw_sphere(targets(t, :), 0.5, 'g'); % Green spheres to mark target points
end

%% Carry out transport tasks to each target point
for t = 1:num_targets
    target_point = targets(t, :); % Select the current target point
    disp(['Transporting to target point: ', num2str(target_point)]);

    %% Trajectory planning
    pickup_phase = round(num_steps / 4);
    transfer_phase = round(num_steps / 3);
    release_phase = round(num_steps / 6);
    reset_phase = num_steps - (pickup_phase + transfer_phase + release_phase);

    theta1 = [linspace(pi/6, 0, pickup_phase), ...
              linspace(0, atan2(target_point(2), target_point(1)), transfer_phase), ...
              linspace(atan2(target_point(2), target_point(1)), atan2(target_point(2), target_point(1)), release_phase), ...
              linspace(atan2(target_point(2), target_point(1)), pi/6, reset_phase)];

    theta2 = [linspace(-pi/4, -pi/2, pickup_phase), ...
              linspace(-pi/2, -pi/3, transfer_phase), ...
              linspace(-pi/3, -pi/6, release_phase), ...
              linspace(-pi/6, -pi/4, reset_phase)];

    theta3 = [linspace(pi/6, -pi/3, pickup_phase), ...
              linspace(-pi/3, 0, transfer_phase), ...
              linspace(0, -pi/6, release_phase), ...
              linspace(-pi/6, pi/6, reset_phase)];

    trajectory = [];
    obj_current_pos = obj_start;
    is_object_released = false;

    %% Animation loop
    for i = 1:num_steps
        % Calculate joint angles of the robotic arm
        t1 = theta1(i);
        t2 = theta2(i);
        t3 = theta3(i);

        % Calculate key points of the robotic arm
        p1 = [0, 0, 0]; % Base
        p2 = [L1*cos(t1), L1*sin(t1), pickup_height]; % First joint
        p3 = p2 + [L2*cos(t1)*cos(t2), L2*sin(t1)*cos(t2), L2*sin(t2)]; % Second joint
        p4 = p3 + [L3*cos(t1)*cos(t2+t3), L3*sin(t1)*cos(t2+t3), L3*sin(t2+t3)]; % End effector
        gripper_pos = p4; % Gripper position

        % Record trajectory
        trajectory = [trajectory; gripper_pos];

        % Determine holding state
        holding = i >= pickup_phase && i < pickup_phase + transfer_phase + release_phase - 5;

        % Object movement logic
        if i < pickup_phase
            obj_current_pos = obj_start;
        elseif holding
            obj_current_pos = gripper_pos;
        elseif ~is_object_released
            obj_current_pos = gripper_pos;
            is_object_released = true;
        end

        %% Plot
        cla;

        % Robotic arm
        draw_cylinder(p1, p2, 0.5, 'r');
        draw_cylinder(p2, p3, 0.4, 'g');
        draw_cylinder(p3, p4, 0.3, 'b');

        % Joint spheres
        draw_sphere(p1, 0.6, 'k');
        draw_sphere(p2, 0.6, 'k');
        draw_sphere(p3, 0.6, 'k');
        draw_sphere(p4, 0.6, 'k');

        % Gripper
        draw_gripper(gripper_pos, gripper_width, gripper_length, holding);

        % Object
        draw_cube(obj_current_pos, 1);

        % Motion trajectory
        plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'c--', 'LineWidth', 1);

        pause(dt);
    end
end

disp('All target point transport tasks completed!');

%% Draw gripper
function draw_gripper(pos, width, length, holding)
    if holding
        gap = width / 2;
    else
        gap = width;
    end
    
    X = [pos(1)-gap, pos(1)+gap];
    Y = [pos(2), pos(2)];
    Z = [pos(3), pos(3)];
    plot3(X, Y, Z, 'LineWidth', 4, 'Color', 'k');
    
    for i = 1:2
        plot3([X(i), X(i)], [Y(i), Y(i)], [Z(i), Z(i) - length], 'LineWidth', 4, 'Color', 'k');
    end
end

%% Draw sphere (for target points and joints)
function draw_sphere(center, radius, color)
    [X, Y, Z] = sphere(20);
    X = X * radius + center(1);
    Y = Y * radius + center(2);
    Z = Z * radius + center(3);
    surf(X, Y, Z, 'FaceColor', color, 'EdgeColor', 'none');
end

%% Draw cylinder (for robotic arm segments)
function draw_cylinder(p1, p2, radius, color)
    [X, Y, Z] = cylinder(radius, 20);
    Z = Z * norm(p2 - p1);
    
    dir = (p2 - p1) / norm(p2 - p1);
    R = vrrotvec2mat(vrrotvec([0 0 1], dir));
    
    XYZ = R * [X(:) Y(:) Z(:)]';
    X(:) = XYZ(1, :) + p1(1);
    Y(:) = XYZ(2, :) + p1(2);
    Z(:) = XYZ(3, :) + p1(3);
    
    surf(X, Y, Z, 'FaceColor', color, 'EdgeColor', 'none');
end

%% Draw cube (for the object)
function draw_cube(center, size)
    vertices = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1;
                -1 -1 1; 1 -1 1; 1 1 1; -1 1 1] * (size/2);
    vertices = vertices + center;
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'y', 'EdgeColor', 'k');
end
