% Define Robot Arm Parameters
L1 = 5; % Length of the first arm segment
L2 = 3; % Length of the second arm segment
L3 = 2; % Length of the third arm segment
dt = 0.1; % Time step for simulation
num_steps = 100; % Number of steps in each experiment
num_targets = 5; % Number of targets to reach

% Generate random target points in 3D space
targets = [randi([-10, 10], num_targets, 3)]; % Include Z coordinate

% Experiment setup
num_experiments = 20; % Number of experiments to run
errors = zeros(num_experiments, 1); % Store position errors
times = zeros(num_experiments, 1); % Store times for each experiment

% Setup figure for animation
figure;
hold on;
axis equal;
grid on;
xlim([-15, 15]);
ylim([-15, 15]);
zlim([-15, 15]);
xlabel('X');
ylabel('Y');
zlabel('Z');

% Simulation loop
for exp = 1:num_experiments
    tic; % Start timer
    target_idx = mod(exp - 1, num_targets) + 1; % Cycle through targets
    target = targets(target_idx, :);
    
    % Initialize arm position
    theta1 = linspace(0, atan2(target(2), target(1)), num_steps);
    theta2 = linspace(0, asin(min(1, max(-1, target(3) / sqrt(target(1)^2 + target(2)^2 + target(3)^2)))), num_steps); % Include vertical movement
    theta3 = linspace(0, pi/4, num_steps); % Simplified angle for demonstration
    
    % Animation and computation loop
    for step = 1:num_steps
        % Add noise to simulate real-world inaccuracies
        noisy_theta1 = theta1(step) + randn * 0.05;
        noisy_theta2 = theta2(step) + randn * 0.05;
        noisy_theta3 = theta3(step) + randn * 0.05;
        
        % Calculate positions
        p1 = [L1 * cos(noisy_theta1) * cos(noisy_theta2), L1 * sin(noisy_theta1) * cos(noisy_theta2), L1 * sin(noisy_theta2)];
        p2 = p1 + [L2 * cos(noisy_theta1) * cos(noisy_theta2 + noisy_theta3), L2 * sin(noisy_theta1) * cos(noisy_theta2 + noisy_theta3), L2 * sin(noisy_theta2 + noisy_theta3)];
        p3 = p2 + [L3 * cos(noisy_theta1) * cos(noisy_theta2 + 2 * noisy_theta3), L3 * sin(noisy_theta1) * cos(noisy_theta2 + 2 * noisy_theta3), L3 * sin(noisy_theta2 + 2 * noisy_theta3)];
        
        % Update plot
        cla;
        plot3([0 p1(1)], [0 p1(2)], [0 p1(3)], 'r-'); % Arm segment 1
        plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'g-'); % Arm segment 2
        plot3([p2(1) p3(1)], [p2(2) p3(2)], [p2(3) p3(3)], 'b-'); % Arm segment 3
        plot3(target(1), target(2), target(3), 'ko'); % Target point
        drawnow;
        
        % Calculate error at the last step
        if step == num_steps
            errors(exp) = norm(p3 - target); % Calculate final position error
        end
    end

    % Record time for the experiment
    times(exp) = toc;
end

% Create a table with the results
Results = table((1:num_experiments)', times, errors, 'VariableNames', {'Experiment', 'CompletionTime', 'PositionError'});
disp(Results);

% Calculate and display summary statistics
SummaryStats = table(mean(times), std(times), mean(errors), std(errors), 'VariableNames', {'MeanTime', 'StdTime', 'MeanError', 'StdError'});
disp(SummaryStats);
