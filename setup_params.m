% --- System Parameters for the "Carelli 1928" Tram ---

% --- Given Physical Parameters ---
V_dc = 600;      % Line Voltage (V)
Ra = 0.39;       % Armature Resistance (Ohm)
Re = 12;         % Excitation Resistance (Ohm)
Kt_rated = 1.06; % Torque constant at rated current (Nm/A)
Ier = 5;         % Excitation Rated Current (A)
Iar = 156;       % Excitation Rated Current (A)
rho = 13/74;     % Gearbox Ratio (-)
beta = 0.81;     % Friction Coefficient (Nms)
g = 9.81;        % Gravity (m/s^2)
M = 25400;       % Total Vehicle Mass (kg)
r = 0.34;        % Wheel Radius (m)
La = 0.0039;     % Armature Inductance (H)
Le = 1.2;        % Excitation Inductance (H)
Jeq = 90.56;     % Equivalent Inertia (kg*m^2)

% --- Given Speed Specifications ---
Omega_r_rpm = 970; % Rated motor speed (rpm)
vmax_kmh = 42.0;   % Maximum vehicle speed (km/h)

% --- CALCULATED Speed Parameters ---
Omega_r_rads = Omega_r_rpm * (2*pi/60);
vr_ms = Omega_r_rads * rho * r; % Rated speed in m/s
vr_kmh = vr_ms * 3.6;           % Rated speed in km/h
En = V_dc - (Ra * Iar);         % Nominal Back EMF

% --- Travel Profile Data for Lookup Tables ---

% Add a small distance after 10km for the final stop
final_stop_dist = 10.001;
tiny_step = 1e-5;

% Speed Profile
dist_profile_speed = [0, 1, 1 + tiny_step, 4, 4 + tiny_step, 6, 6 + tiny_step, 9, 9 + tiny_step, 10, 10 + tiny_step, final_stop_dist]; % Add final points
speed_req_kmh =      [vr_kmh/2, vr_kmh/2, vr_kmh, vr_kmh, vmax_kmh, vmax_kmh, vr_kmh, vr_kmh, vr_kmh/2, vr_kmh/2, 0, 0]; % Add final drop to zero

% Slope Profile (can remain the same)
dist_profile_slope = [0, 3, 3 + tiny_step, 4, 4 + tiny_step, 8, 8 + tiny_step, 9, 9 + tiny_step, 10];
slope_profile =      [0, 0, 5, 5, 0, 0, -5, -5, 0, 0];

% --- PID Tuning via MATLAB Script ---

% 1. Model the behavior of the tuned inner current loop as a simple, fast filter.
%    The time constant is the response time we set in its tuner (e.g., 0.01s).
tau_current_loop = 0.01; 
G_current_loop = tf(1, [tau_current_loop 1]);

% 2. Model the mechanical part of the plant.
G_mechanical = tf(1, [Jeq beta]);

% 3. The total simplified plant seen by the outer speed controller is the
%    combination of the inner loop, the flux constant, and the mechanics.
G_outer_plant = G_current_loop * Kt_rated * G_mechanical;

% 4. Set the desired performance for the outer speed loop.
%    We want a response time of 1.0 seconds, which corresponds to a
%    bandwidth (crossover frequency) of approximately 2 rad/s.
wc_speed = 2; % Target crossover frequency in rad/s for the speed loop
phase_margin = 90;
% 5. Use pidtune to calculate the gains.
[C_speed,info] = pidtune(G_outer_plant, 'PI', wc_speed);

% Display the calculated gains in the command window so you can see them.
fprintf('Calculated Speed Controller Gains:\n');
fprintf('Kp = %f\n', C_speed.Kp);
fprintf('Ki = %f\n', C_speed.Ki);

%% --- SCRIPT FOR SIMULATION AND DETAILED PLOTTING ---


disp('Running Simulink simulation...');
sim('model.slx'); % <-- IMPORTANT: REPLACE WITH YOUR SIMULINK FILE NAME
disp('Simulation complete.');

%% --- Post-Processing and Data Extraction ---

% Extract the logged data from the Simulink output variable
speed_data = out.logsout.get('Omega').Values; % Replace with your logged speed signal name
torque_data = out.logsout.get('Te').Values; % Replace with your logged torque signal name
power_data = out.logsout.get('Power').Values; % Replace with your logged power signal name

% Get the speed in rad/s from the timeseries object
speed_rad_s = speed_data.Data;

% Create a new figure
figure;

% --- Plot Torque on the Left Y-Axis ---
yyaxis left;
plot(speed_rad_s, torque_data.Data, 'b-', 'LineWidth', 2);
ylabel('Torque (Nm)');
ylim([0, 900]); % Set torque limits, e.g., 0 to slightly above nominal

% --- Plot Power on the Right Y-Axis ---
yyaxis right;
plot(speed_rad_s, power_data.Data / 1000, 'r--', 'LineWidth', 2); % Convert power to kW
ylabel('Power (kW)');
ylim([0, 100]); % Set power limits, e.g., 0 to slightly above rated

% --- Final Touches ---
grid on;
title('Motor Operating Regions');
xlabel('Speed (rad/s)');
legend('Torque', 'Power', 'Location', 'best');