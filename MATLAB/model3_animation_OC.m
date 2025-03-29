% Animation to display the results of the Optimal Control (OC).
% Extract simulation data
close all

which_controller = "MPC";
%---OPTIMAL CONTROLLER - data from MATLAB simulation
if which_controller == "OC_SIMULINK"
    time = ySim(:, 1);   % Time vector
    x = ySim(:, 2);      % x position (output(1))
    y = ySim(:, 3);      % y position (output(2))
    theta = ySim(:, 4);  % Orientation angle (output(3))
end
%---OPTIMAL CONTROLLER - data from SIMULINK
if which_controller == "OC_SIMULINK"
    time = ySim(:, 1);   % Time vector
    x = out.simout1.Data(:, 1);      % x position (output(1))
    y = out.simout1.Data(:, 2);      % y position (output(2))
    theta = out.simout1.Data(:, 3);  % Orientation angle (output(3))
end
%---MPC
if which_controller == "MPC"
    time = out.MPCsim.Time;         % Time vector
    x = out.MPCsim.Data(:, 1);      % x position (output(1))
    y = out.MPCsim.Data(:, 2);      % y position (output(2))
    theta = out.MPCsim.Data(:, 3);  % Orientation angle (output(3))
    uMPC_T = out.MPCsim.Data(:,4);    % Thrust force
    uMPC_angle = out.MPCsim.Data(:,5);% Thrust angle
end

% Define rocket shape: rectangle with a triangle on top
rocket_length = 0.5; % Total length of the rocket (body + triangle)
body_length = 0.35;  % Length of the rectangular body
body_width = 0.15;   % Width of the rectangular body
triangle_height = rocket_length - body_length; % Height of the triangle
% Define the shape (vertices) of the rocket:
% Rectangle (body) + Triangle (tip)
rocket_shape = [
    -body_width/2, -body_length/2;  % Bottom-left of the body
    -body_width/2, body_length/2;   % Top-left of the body
    0, body_length/2 + triangle_height; % Tip of the triangle
    body_width/2, body_length/2;    % Top-right of the body
    body_width/2, -body_length/2;   % Bottom-right of the body
];

% Define target position
target_x = xf(1); % Target x-coordinate
target_y = xf(3); % Target y-coordinate

% Initialize video writer
video = VideoWriter('rocket_animation.avi'); % Video file
video.FrameRate = 1 / Ts; % Set frame rate based on sampling time
open(video);

% Set up figure
figure('Position', [100, 100, 1200, 900]); % Large figure window
subplot(5, 1, 1:3); % Top subplot for the animation
hold on;
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
xlim([min(x)-1, max(x)+1]);
ylim([min(y)-1, max(y)+1]);

% Plot the target position
plot(target_x, target_y, 'bx', 'MarkerSize', 10, 'LineWidth', 2); % Blue "x" for target

% Initialize the rocket plot
% rocket_plot = fill(0, 0, 'r'); % Red triangle for the rocket
rocket_plot = fill(rocket_shape(:, 1), rocket_shape(:, 2), 'r'); % Red rocket (rectangle + triangle)
% Define thrust line length
thrust_line_length = 0.3; % Length of the thrust direction line
% Initialize thrust direction line
thrust_line = plot([0, 0], [0, 0], 'b', 'LineWidth', 2); % Blue line for thrust direction
% Initialize the trajectory line
trajectory_plot = plot(x(1), y(1), 'b--'); % Dashed blue line for trajectory

% Create subplot for uMPC_T over time
subplot(5, 1, 4); % Second-to-last subplot
uMPC_T_plot = plot(NaN, NaN, 'g', 'LineWidth', 1.5); % Empty plot for uMPC_T
grid on;
xlabel('Time (s)');
ylabel('u_{MPC_T}');
title('Thrust Force (u_{MPC_T}) Over Time');
legend('u_{MPC_T}', 'Location', 'best');
xlim([time(1), time(end)]);
ylim([min(uMPC_T) - 0.1, max(uMPC_T) + 0.1]); % Adjust Y-limits if needed

% Create subplot for uMPC_angle over time
subplot(5, 1, 5); % Bottom subplot
uMPC_angle_plot = plot(NaN, NaN, 'm', 'LineWidth', 1.5); % Empty plot for uMPC_angle
grid on;
xlabel('Time (s)');
ylabel('u_{MPC\_angle} (rad)');
title('Thrust Angle (u_{MPC\_angle}) Over Time');
legend('u_{MPC\_angle}', 'Location', 'best');
xlim([time(1), time(end)]);
ylim([min(uMPC_angle) - 0.1, max(uMPC_angle) + 0.1]); % Adjust Y-limits if needed

%% Animation loop
for i = 1:length(time)
    fprintf("Time: %f\r", time(i))
    % Update rocket position and orientation
    R = [cos(theta(i)), -sin(theta(i)); sin(theta(i)), cos(theta(i))]; % Rotation matrix
    transformed_rocket = (R * rocket_shape')';                        % Rotate rocket
    transformed_rocket(:, 1) = transformed_rocket(:, 1) + x(i);       % Translate in x
    transformed_rocket(:, 2) = transformed_rocket(:, 2) + y(i);       % Translate in y

    % Update rocket plot
    set(rocket_plot, 'XData', transformed_rocket(:, 1), ...
                     'YData', transformed_rocket(:, 2));
    
    % Update trajectory line
    set(trajectory_plot, 'XData', x(1:i), 'YData', y(1:i));

     % Calculate thrust direction
    if which_controller == "MPC"
        thrust_angle = theta(i) + uMPC_angle(i); % Thrust angle relative to the rocket
    else
        thrust_angle = theta(i) + uSim(i, 3); % Thrust angle relative to the rocket
    end
    thrust_vector = thrust_line_length * [sin(thrust_angle), -cos(thrust_angle)];
    thrust_start = [(transformed_rocket(1,1)+transformed_rocket(end,1))/2, (transformed_rocket(1,2)+transformed_rocket(end,2))/2];                              % Starting point of thrust line
    thrust_end = thrust_start + thrust_vector;                % Ending point of thrust line

    % Update thrust direction line
    set(thrust_line, 'XData', [thrust_start(1), thrust_end(1)], ...
                     'YData', [thrust_start(2), thrust_end(2)]);

    % Update uMPC_T plot dynamically
    subplot(5, 1, 4); % Switch to the uMPC_T plot
    set(uMPC_T_plot, 'XData', time(1:i), 'YData', uMPC_T(1:i)); % Update only up to current time

    % Update uMPC_angle plot dynamically
    subplot(5, 1, 5); % Switch to the uMPC_angle plot
    set(uMPC_angle_plot, 'XData', time(1:i), 'YData', uMPC_angle(1:i)); % Update only up to current time

    % Capture frame
    frame = getframe(gcf);
    writeVideo(video, frame);

    % Pause for animation (optional)
    pause(0.01);
end

% Close video writer
close(video);

disp('Animation complete! Saved as rocket_animation.avi');
