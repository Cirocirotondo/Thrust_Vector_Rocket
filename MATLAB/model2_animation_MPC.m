% Animation to display the results of the control of the rocket that is
% going up in the sky, controlled with MPC
% Extract simulation data

close all

% PARAMETERS
video_name = 'rocket_animation_no_thrust.avi';


time = out.simout1.Time;        % Time vector
x = out.simout1.Data(:, 1);     % x position (output(1))
y = out.simout1.Data(:, 2);     % y position (output(2))
theta = out.simout1.Data(:, 3); % Orientation angle (output(3))
u_sim = out.simout1.Data(:,4);   % Thrust angle (control of the rocket)

% Initialize video writer
video = VideoWriter(video_name); % Video file
video.FrameRate = 1 / Ts; % Set frame rate based on sampling time
open(video);


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



% Set up figure
figure('Position', [100, 100, 1200, 900]); % Large figure window

% Create subplot for the animation
subplot(5, 1, 1:3); % Top subplot for the animation
hold on;
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Rocket Animation');
xlim([min(x) - 1, max(x) + 1]);
ylim([min(y) - 1, max(y) + 1]);
% Initialize the rocket plot
rocket_plot = fill(rocket_shape(:, 1), rocket_shape(:, 2), 'r'); % Red rocket (rectangle + triangle)
% Define thrust line length
thrust_line_length = 0.3; % Length of the thrust direction line
% Initialize thrust direction line
thrust_line = plot([0, 0], [0, 0], 'b', 'LineWidth', 2); % Blue line for thrust direction
% Initialize the trajectory line
trajectory_plot = plot(x(1), y(1), 'b--'); % Dashed blue line for trajectory

% Create subplot for theta over time
subplot(5, 1, 4); % Middle subplot
theta_plot = plot(NaN, NaN, 'r', 'LineWidth', 1.5); % Empty plot for theta
grid on;
xlabel('Time (s)');
ylabel('\theta (rad)');
title('Rocket Orientation (\theta) Over Time');
legend('\theta', 'Location', 'best');
xlim([time(1), time(end)]);
ylim([min(theta) - 0.1, max(theta) + 0.1]); % Adjust Y-limits if needed

% Create subplot for u_sim over time
subplot(5, 1, 5); % Bottom subplot
u_sim_plot = plot(NaN, NaN, 'b', 'LineWidth', 1.5); % Empty plot for u_sim
grid on;
xlabel('Time (s)');
ylabel('u_{Sim} (rad)');
title('Control Input (u_{Sim} = thrust angle) Over Time');
legend('u_{Sim}', 'Location', 'best');
xlim([time(1), time(end)]);
ylim([min(u_sim) - 0.1, max(u_sim) + 0.1]); % Adjust Y-limits if needed


% Animation loop
for i = 1:length(time)
    fprintf("Time: %f\r", Ts*i)

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
    thrust_angle = theta(i) + u_sim(i); % Thrust angle relative to the rocket
    thrust_vector = thrust_line_length * [sin(thrust_angle), -cos(thrust_angle)];
    thrust_start = [(transformed_rocket(1,1)+transformed_rocket(end,1))/2, (transformed_rocket(1,2)+transformed_rocket(end,2))/2];                              % Starting point of thrust line
    thrust_end = thrust_start + thrust_vector;                % Ending point of thrust line

    % Update thrust direction line
    set(thrust_line, 'XData', [thrust_start(1), thrust_end(1)], ...
                     'YData', [thrust_start(2), thrust_end(2)]);

    % Update theta plot dynamically
    subplot(5, 1, 4); % Switch to the theta plot
    set(theta_plot, 'XData', time(1:i), 'YData', theta(1:i)); % Update only up to current time
    % Update u_sim plot dynamically
    subplot(5, 1, 5); % Switch to the u_sim plot
    set(u_sim_plot, 'XData', time(1:i), 'YData', u_sim(1:i)); % Update only up to current time

    % Capture frame
    frame = getframe(gcf);
    writeVideo(video, frame);

    % Pause for animation (optional)
    pause(Ts);
end

% Close video writer
close(video);

fprintf('Animation complete! Saved as %s', video_name);
