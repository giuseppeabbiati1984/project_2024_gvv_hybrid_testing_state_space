% This model represents two springs (one linear, one Bouc-Wen)
clc; clear;

% Define model name
modelname = 'model_2springs';

%% Define elements and properties
% Linear spring
element{1}.m = 1e2 * eye(2); % Mass matrix for element 1
element{1}.k = 100 * [1,-1;-1,1]; % Stiffness matrix for element 1
element{1}.type = 'linear';
element{1}.dofs = [1,1;2,1]; % Degrees of freedom mapping

% Bouc-Wen spring
element{2}.m = 2e2 * eye(2); % Mass matrix for element 2
element{2}.k = zeros(2); % Stiffness matrix for element 2
element{2}.type = 'boucwen';
element{2}.alpha = 50;
element{2}.beta = 0.5;
element{2}.gamma = 0.0;
element{2}.n = 1;
element{2}.dofs = [2,1;3,1]; % Degrees of freedom mapping

% setting default values
element = element_processing(element) ;

% Model parameters
model.dofs_f = [2,1;3,1];
model.dofs_d = [1,1];

model.loads_f{1}.type = 'Constant';
model.loads_f{1}.Value = 0;

model.loads_f{2}.type = 'Sine Wave';
model.loads_f{2}.Amplitude = 5e2;
model.loads_f{2}.Bias = 0;
model.loads_f{2}.Frequency = 2*pi;
model.loads_f{2}.Phase = 0;

model.loads_d{1}.type = 'Constant';
model.loads_d{1}.Value = 0;

[element, model] = create_model(element, model);

return

%% Assemble global mass, stiffness, and damping matrices
ndofs = 3; % Total degrees of freedom in the system

M = zeros(ndofs);
K = zeros(ndofs);
C = zeros(ndofs);

% Add contributions from element 1 (linear spring)
M(1:2, 1:2) = M(1:2, 1:2) + element{1}.m; 
K(1:2, 1:2) = K(1:2, 1:2) + element{1}.k;
C(1:2, 1:2) = C(1:2, 1:2) + element{1}.c;

% Add contributions from element 2 (Bouc-Wen spring)
M(2:3, 2:3) = M(2:3, 2:3) + element{2}.m;
K(2:3, 2:3) = K(2:3, 2:3) + element{2}.k;
C(2:3, 2:3) = C(2:3, 2:3) + element{2}.c;

%% Boundary Condition for Node 1
wu = 2*pi*2; % Frequency of imposed displacement
gu = @(t) 0.00*cos(wu*t); % Imposed displacement at Node 1

% Ground motion force
wf = 2*pi; % Frequency of sinusoidal ground motion
gf = @(t) 5e2*sin(wf*t); % Imposed force at Node 3

% Remove DOF corresponding to Node 1
Mr = M(2:end, 2:end);
Cr = C(2:end, 2:end);
Kr = K(2:end, 2:end);
Ku = K(2:3,1) ;

%% Initial conditions and time span
tspan = [0 20]; % Time span
u0 = [0; 0]; % Initial displacement (Node 2, Node 3)
v0 = [0; 0]; % Initial velocity (Node 2, Node 3)
r0 = 0; % Initial Bouc-Wen force variable
y0 = [u0; v0; r0]; % Combine initial conditions

%% Solve the system
[t, y] = ode45(@(t, y) EOM(t, y, Mr, Cr, Kr, Ku, ...
    element{2}.alpha,element{2}.beta,element{2}.gamma,element{2}.n,gu,gf), tspan, y0);

%% Extract results
u = y(:,1:2); % Node 2 and Node 3
v = y(:,3:4); % Node 2 and Node 3
r = y(:,5);

% Include Node 1 displacement
displacement = [gu(t), u];

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
plot(t, u(:, 1), 'r', 'LineWidth', 2, 'DisplayName', 'Node 2'); % Node 1 displacement
hold on;
plot(t, u(:, 2), 'b', 'LineWidth', 2, 'DisplayName', 'Node 3'); % Node 2 displacement
hold on;
% plot(t, displacement(:, 3), 'b', 'LineWidth', 2, 'DisplayName', 'Node 3'); % Node 3 displacement
title('Displacement vs Time');
xlabel('Time (s)');
ylabel('Displacement (m)');
legend('show');
grid on;

% Plot velocity vs time for Node 2 and Node 3
subplot(3,1,2);
plot(t, v(:, 1), 'r', 'LineWidth', 2, 'DisplayName', 'Node 2'); % Node 2 velocity
hold on;
plot(t, v(:, 2), 'b', 'LineWidth', 2, 'DisplayName', 'Node 3'); % Node 3 velocity
title('Velocity vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('show');
grid on;

% Plot Bouc-Wen restoring force vs time (using Bouc-Wen force variable)
subplot(3,1,3);
plot(t,r, 'g', 'LineWidth', 2, 'DisplayName', 'Bouc-Wen Force'); % Bouc-Wen force
title('Restoring Force vs Time');
xlabel('Time (s)');
ylabel('Force (N)');
legend('show');
grid on;

% Create Hysteresis Plot (Force vs Displacement for Bouc-Wen)
figure;

% Plot Bouc-Wen force vs displacement for Node 2
plot(u(:,1)-u(:,2),r, 'LineWidth', 2);
title('Bouc-Wen Hysteresis Loop');
xlabel('Displacement (m)');
ylabel('Bouc-Wen Force (N)');
grid on;

%% Function Definitions
function yd = EOM(t, y, Mr, Cr, Kr, Ku, alpha, beta, gamma, n, gu, gf)

    % Extract variables
    u = y(1:2); % displacements
    v = y(3:4); % velocities
    r = y(5); % Bouc-Wen restoring force

    % Bouc-Wen force
    dv = v(1) - v(2); % Relative velocity (Node 2 - Node 3)
    rd = (alpha - (beta * sign(r*dv) + gamma) * abs(r)^n) * dv;

    % External loading from imposed displacement
    F_ext = gf(t) * [0;1] - Ku * gu(t);

    % Internal restoring force from bouc-wen
    F_int = Cr*v + Kr*u + [r;-r];

    % Equations of motion
    ud = v ;
    vd = Mr \ (F_ext - F_int);
    
    % Assemble state derivatives
    yd = [ud; vd; rd];

end
