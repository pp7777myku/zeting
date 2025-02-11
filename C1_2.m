clc;
clear;
close all;

%% 机械臂参数
L1 = 5; % 第一节臂长
L2 = 4; % 第二节臂长
L3 = 3; % 第三节臂长
gripper_width = 2; % 夹持器夹指间距
gripper_length = 1; % 夹指长度
dt = 0.1; % 时间步长
t_total = 10; % 单次搬运任务时间
num_steps = t_total / dt;
num_targets = 5; % 目标点数量

%% 物体与初始位置
obj_start = [5, 0, 0]; % 物体初始位置
pickup_height = 8; % 机械臂初始上扬高度

%% 生成 5 个随机目标点 (-10,10) 区间内
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

% 标记目标点
for t = 1:num_targets
    plot3(targets(t,1), targets(t,2), targets(t,3), 'go', 'MarkerSize', 8, 'LineWidth', 2);
end

%% 记录指标
actual_trajectories = cell(num_targets,1);
errors = zeros(num_targets, 1);
completion_times = zeros(num_targets, 1);

%% 搬运任务
for t = 1:num_targets
    target_point = targets(t, :);
    trajectory = [];
    obj_current_pos = obj_start;
    is_object_released = false;
    
    % 轨迹规划
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
        
        plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'c--', 'LineWidth', 1);
        pause(dt/5);
    end
    completion_times(t) = toc;
    actual_trajectories{t} = trajectory;
    errors(t) = norm(trajectory(end,:) - target_point);
end

%% 评估指标可视化
figure;
subplot(2,1,1);
bar(completion_times);
title('搬运时间');
xlabel('目标点编号');
ylabel('时间 (秒)');

subplot(2,1,2);
bar(errors);
title('末端误差');
xlabel('目标点编号');
ylabel('误差 (m)');

figure;
hold on;
for t = 1:num_targets
    plot3(actual_trajectories{t}(:,1), actual_trajectories{t}(:,2), actual_trajectories{t}(:,3));
end
title('机械臂运动轨迹');
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on;
hold off;

disp('搬运任务完成，评估指标已生成。');
