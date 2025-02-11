clc;
clear;
close all;

%% Robot Arm Parameters
L1 = 5; % First segment length
L2 = 4; % Second segment length
L3 = 3; % Third segment length
gripper_width = 2; % Gripper width
gripper_length = 1; % Gripper length
dt = 0.1; % Time step
t_total = 10; % Total time per task
num_steps = t_total / dt;
num_targets = 100; % Number of target points

%% Object and Initial Position
obj_start = [5, 0, 0]; % Initial object position
pickup_height = 8; % Initial lift height

%% Generate 100 Random Target Points in Range (-10,10)
targets = [randi([-10, 10], num_targets, 1), ...
           randi([-10, 10], num_targets, 1), ...
           zeros(num_targets, 1)];

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

% Mark Target Points
for t = 1:num_targets
    plot3(targets(t,1), targets(t,2), targets(t,3), 'go', 'MarkerSize', 8, 'LineWidth', 2);
end

%% Metrics Recording
actual_trajectories = cell(num_targets,1);
errors = zeros(num_targets, 1);
completion_times = zeros(num_targets, 1);

%% Perform Pick-and-Place Tasks
for t = 1:num_targets
    target_point = targets(t, :);
    trajectory = [];
    obj_current_pos = obj_start;
    is_object_released = false;
    
    % Trajectory Planning
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
    
    tic;
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
    
    for i = 1:num_steps
        t1 = theta1(i);
        t2 = theta2(i);
        t3 = theta3(i);
        
        p1 = [0, 0, 0];
        p2 = [L1*cos(t1), L1*sin(t1), pickup_height];
        p3 = p2 + [L2*cos(t1)*cos(t2), L2*sin(t1)*cos(t2), L2*sin(t2)];
        p4 = p3 + [L3*cos(t1)*cos(t2+t3), L3*sin(t1)*cos(t2+t3), L3*sin(t2+t3)];
        gripper_pos = p4;
        
        trajectory = [trajectory; gripper_pos];
        holding = i >= pickup_phase && i < pickup_phase + transfer_phase + release_phase - 5;
        
        if i < pickup_phase
            obj_current_pos = obj_start;
        elseif holding
            obj_current_pos = gripper_pos;
        elseif ~is_object_released
            obj_current_pos = gripper_pos;
            is_object_released = true;
        end
        
        cla;
        plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'c--', 'LineWidth', 1);
        pause(dt/5);
    end
    completion_times(t) = toc;
    actual_trajectories{t} = trajectory;
    errors(t) = norm(trajectory(end,:) - target_point);
end

%% Evaluation Metrics Visualization
figure;
subplot(3,1,1);
bar(completion_times);
title('Task Completion Time');
xlabel('Target Point Index');
ylabel('Time (s)');

subplot(3,1,2);
bar(errors);
title('End Effector Position Error');
xlabel('Target Point Index');
ylabel('Error (m)');

subplot(3,1,3);
bar([mean(completion_times), std(completion_times), mean(errors), std(errors)]);
set(gca, 'XTickLabel', {'Mean Time', 'Std Time', 'Mean Error', 'Std Error'});
title('Statistical Summary');
ylabel('Value');

disp('Pick-and-place task completed, evaluation metrics generated.');
