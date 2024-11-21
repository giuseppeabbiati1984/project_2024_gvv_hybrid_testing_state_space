% This model represents two springs (one linear, one Bouc-Wen)
clc; clear;

% Define model name
modelname = 'model_2springs';

%% Define elements and properties
% Linear spring
element{1}.m = 1e2 * eye(2); % Mass matrix for element 1
element{1}.k = 100 * [1 -1; -1 1]; % Stiffness matrix for element 1
element{1}.type = 'linear';
element{1}.dofs = [1,1; 2,1]; % Degrees of freedom mapping

% Bouc-Wen spring
element{2}.m = 2e2 * [1 0; 0 1]; % Mass matrix for element 2
element{2}.k = 1 * [1 -1; -1 1]; % Stiffness matrix for element 2
element{2}.type = 'boucwen';
element{2}.alpha = 1;
element{2}.beta = 0.1;
element{2}.gamma = 0.05;
element{2}.n = 1;
element{2}.dofs = [2,1; 3,1]; % Degrees of freedom mapping

% Model parameters
model.dofs_f = [2,1;3,1];
model.dofs_d = [1,1];

% Bouc-Wen parameters
alpha = 1; beta = 0.1; gamma = 0.05; n = 1; % Parameters for Bouc-Wen

% Rayleigh damping coefficients
a0 = 0.00; % Mass-proportional damping
a1 = 0.00; % Stiffness-proportional damping

%% Assemble global mass, stiffness, and damping matrices
num_dofs = 3; % Total degrees of freedom in the system
M_global = zeros(num_dofs);
K_global = zeros(num_dofs);

% Add contributions from element 1 (linear spring)
M_global(1:2, 1:2) = M_global(1:2, 1:2) + element{1}.m; 
K_global(1:2, 1:2) = K_global(1:2, 1:2) + element{1}.k;

% Add contributions from element 2 (Bouc-Wen spring)
M_global(2:3, 2:3) = M_global(2:3, 2:3) + element{2}.m;
% K_global(2:3, 2:3) = K_global(2:3, 2:3) + element{2}.k;

% Damping matrix (Rayleigh damping)
C_global = a0 * M_global + a1 * K_global;

%% Boundary Condition for Node 1
omega_u = 2 * pi / 5; % Frequency of imposed displacement
imposed_disp = @(t) 0.25*cos(omega_u * t); % Imposed displacement at Node 1

% Remove DOF corresponding to Node 1
M_reduced = M_global(2:end, 2:end);
C_reduced = C_global(2:end, 2:end);
K_reduced = K_global(2:end, 2:end);

%% Initial conditions and time span
tspan = [0 5]; % Time span
u0_reduced = [0; 0]; % Initial displacement (Node 2, Node 3)
v0_reduced = [0; 0]; % Initial velocity (Node 2, Node 3)
z0 = 0; % Initial Bouc-Wen force variable
IC = [u0_reduced; v0_reduced; z0]; % Combine initial conditions

% Ground motion force
omega_g = 2 * pi; % Frequency of sinusoidal ground motion
ground_motion = @(t) 0.015*sin(omega_g * t); % Imposed force at Node 3

%% Solve the system
[t, y] = ode45(@(t, y) EOM(t, y, M_reduced, C_reduced, K_reduced, imposed_disp(t), ground_motion(t), alpha, beta, gamma, n), tspan, IC);

%% Extract results
displacement_reduced = y(:, 1:2); % Node 2 and Node 3
velocity_reduced = y(:, 3:4); % Node 2 and Node 3
boucwen_force = y(:, end);

% Include Node 1 displacement
displacement = [imposed_disp(t), displacement_reduced];

%% Create GIF for displacement over time
% filename = 'displacement_animation.gif';
% 
% % Create the figure for plotting displacement
% figure;
% 
% % Initial node positions (x, y)
% node_positions = [0, 0; 2, 0; 4, 0];  % Node 1 [0,0], Node 2 [1,0], Node 3 [2,0]
% x_positions = node_positions(:, 1); % Initial x-positions
% 
% for i = 1:length(t)
%     % Update the x-positions with the displacement at each time step
%     current_x_positions = x_positions + displacement(i, :)'; % Add displacement to the x-coordinate
% 
%     % Plot the updated node positions
%     plot(current_x_positions, node_positions(:, 2), 'ko', 'MarkerFaceColor', 'k'); % Node 1 in black
%     hold on;
%     plot(current_x_positions(2), node_positions(2, 2), 'ro', 'MarkerFaceColor', 'r'); % Node 2 in red
%     plot(current_x_positions(3), node_positions(3, 2), 'bo', 'MarkerFaceColor', 'b'); % Node 3 in blue
% 
%     % Labels and title
%     title('Displacement of Nodes Over Time');
%     xlabel('X-Position (m)');
%     ylabel('Y-Position (m)');
% 
%     % Check and adjust axis limits
%     x_min = min(current_x_positions) - 0.5;
%     x_max = max(current_x_positions) + 0.5;
% 
%     % Ensure x_min is less than x_max
%     if x_min >= x_max
%         x_min = x_max - 1; % Adjust if necessary
%     end
% 
%     axis([-5, 10, -1, 1]); % Adjust axis limits for better view
%     legend('Node 1', 'Node 2', 'Node 3', 'Location', 'Best');
% 
%     % Capture the plot as an image for the GIF
%     drawnow;
%     frame = getframe(gcf);
%     im = frame2im(frame);
%     [imind, cm] = rgb2ind(im, 256);
% 
%     % Write to GIF
%     if i == 1
%         imwrite(imind, cm, filename, 'gif', 'LoopCount', inf, 'DelayTime', 0.1);
%     else
%         imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
%     end
% 
%     % Clear the plot for the next frame
%     clf;
% end

%% Create Plots for Displacement, Velocity, and Restoring Force
% Plot displacement vs time for Node 2 and Node 3
figure;
subplot(3,1,1);
plot(t, displacement(:, 1), 'k', 'LineWidth', 2, 'DisplayName', 'Node 1'); % Node 1 displacement
hold on;
plot(t, displacement(:, 2), 'r', 'LineWidth', 2, 'DisplayName', 'Node 2'); % Node 2 displacement
hold on;
plot(t, displacement(:, 3), 'b', 'LineWidth', 2, 'DisplayName', 'Node 3'); % Node 3 displacement
title('Displacement vs Time');
xlabel('Time (s)');
ylabel('Displacement (m)');
legend('show');
grid on;

% Plot velocity vs time for Node 2 and Node 3
subplot(3,1,2);
plot(t, velocity_reduced(:, 1), 'r', 'LineWidth', 2, 'DisplayName', 'Node 2'); % Node 2 velocity
hold on;
plot(t, velocity_reduced(:, 2), 'b', 'LineWidth', 2, 'DisplayName', 'Node 3'); % Node 3 velocity
title('Velocity vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('show');
grid on;

% Plot Bouc-Wen restoring force vs time (using Bouc-Wen force variable)
subplot(3,1,3);
plot(t, boucwen_force, 'g', 'LineWidth', 2, 'DisplayName', 'Bouc-Wen Force'); % Bouc-Wen force
title('Restoring Force vs Time');
xlabel('Time (s)');
ylabel('Force (N)');
legend('show');
grid on;

%% Create Hysteresis Plot (Force vs Displacement for Bouc-Wen)
figure;

% Plot Bouc-Wen force vs displacement for Node 2
plot(displacement(:, 2), boucwen_force, 'LineWidth', 2);
title('Bouc-Wen Hysteresis Loop');
xlabel('Displacement (m)');
ylabel('Bouc-Wen Force (N)');
grid on;

%% Function Definitions
function dydt = EOM(t, y, M_reduced, C_reduced, K_reduced, u1, f3, alpha, beta, gamma, n)
    % Extract variables
    u = y(1:2); % Displacements (Node 2, Node 3)
    v = y(3:4); % Velocities (Node 2, Node 3)
    z = y(end); % Bouc-Wen force state variable

    % Bouc-Wen force
    du = v(1) - v(2); % Relative velocity (Node 2 - Node 3)
    dz = - (alpha - (beta * sign(z*du) + gamma) * abs(z^n) * du);

    % External forcing term from imposed displacement u1
    F_ext = -K_reduced(:, 1) * u1; % Contribution of Node 1 displacement

    % Restoring force
    F_boucwen = -z;
    F_restoring = [F_boucwen; -F_boucwen];

    % Equations of motion
    a = M_reduced \ (f3 * [0; 1] - C_reduced * v - K_reduced * u - F_restoring + F_ext);
    
    % Assemble state derivatives
    dydt = [v; a; dz];
end
