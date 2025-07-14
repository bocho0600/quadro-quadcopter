clc;
clear;

%% --- Parameters ---
Ts = 0.004;                 % Time step (same as Arduino loop rate)
T_end = 10;                 % Total simulation time
time = 0:Ts:T_end;
n = length(time);

% PID Gains (Rate)
Kp_rate = [0.6, 0.6, 2];
Ki_rate = [3.5, 3.5, 12];
Kd_rate = [0.03, 0.03, 0];

% PID Gains (Angle)
Kp_angle = [6, 6, 4];

% Quad physical model (simplified)
A = -diag([5, 5, 6]);       % Damping
B = eye(3);                 % Input matrix

mass = 1.0;                 % kg
g = 9.81;                   % m/s²
thrust_per_pwm = 0.005;     % Simplified: thrust per PWM unit

% Motor mixing matrix (layout 4 1 / 3 2)
Mix = 1.024 * [
    1, -1, -1, -1;  % M1 - Front-right
    1,  1, -1,  1;  % M2 - Rear-right
    1,  1,  1, -1;  % M3 - Rear-left
    1, -1,  1,  1;  % M4 - Front-left
];

%% --- Memory Allocation ---
angle = zeros(3,n);           % [roll; pitch; yaw] in deg
rate = zeros(3,n);            % Angular velocity (deg/s)
desired_angle = zeros(3,n);   % Desired angles from joystick
desired_rate = zeros(3,n);    % Desired rates from outer PID
U = zeros(3,n);               % Inner loop control output
Error = zeros(3,n);           % Rate error
Integral = zeros(3,n);        % Rate integral term
Derivative = zeros(3,n);      % Rate derivative term
MotorInputs = zeros(4,n);     % Motor PWM outputs

pos = zeros(3,n);             % Position (m)
vel = zeros(3,n);             % Velocity (m/s)
acc = zeros(3,n);             % Acceleration (m/s²)

%% --- Joystick Input (simulate stick motion) ---
desired_angle(1,:) = 10 * sin(2*pi*0.3*time);  % Roll (deg)
desired_angle(2,:) = 5 * sin(2*pi*0.2*time);   % Pitch (deg)
desired_angle(3,:) = 0;                        % Yaw (deg)

%% --- Simulation Loop ---
for k = 2:n
    % --- Angle PID (outer loop): angle → desired rate ---
    angle_error = desired_angle(:,k) - angle(:,k-1);
    desired_rate(:,k) = Kp_angle' .* angle_error;
    desired_rate(:,k) = max(min(desired_rate(:,k), 200), -200);

    % --- Rate PID (inner loop): desired rate → torque command (U) ---
    Error(:,k) = desired_rate(:,k) - rate(:,k-1);
    Integral(:,k) = Integral(:,k-1) + 0.5 * (Error(:,k) + Error(:,k-1)) * Ts;
    Derivative(:,k) = (Error(:,k) - Error(:,k-1)) / Ts;

    U(:,k) = Kp_rate' .* Error(:,k) + Ki_rate' .* Integral(:,k) + Kd_rate' .* Derivative(:,k);
    U(:,k) = max(min(U(:,k), 400), -400);

    % --- Rate dynamics update ---
    rate(:,k) = rate(:,k-1) + Ts * (A * rate(:,k-1) + B * U(:,k));
    angle(:,k) = angle(:,k-1) + Ts * rate(:,k-1);  % Integrate to get angle

    % --- Motor mixing (Throttle + corrections) ---
    throttle_base = 1500;  % Simulated throttle
    ctrl = [throttle_base; U(:,k)];
    MotorInputs(:,k) = Mix * ctrl;
    MotorInputs(:,k) = max(min(MotorInputs(:,k), 1900), 1100);  % Clamp

    % --- Position dynamics (simplified translational model) ---
    phi = deg2rad(angle(1,k));
    theta = deg2rad(angle(2,k));

    thrust_total = sum(MotorInputs(:,k) - 1100) * thrust_per_pwm;
    Tz = thrust_total * cos(phi) * cos(theta);

    acc(:,k) = [
        thrust_total * sin(theta);
        -thrust_total * sin(phi);
        Tz - mass * g
    ] / mass;

    vel(:,k) = vel(:,k-1) + Ts * acc(:,k);
    pos(:,k) = pos(:,k-1) + Ts * vel(:,k);
end

%% --- Plots ---

% 1. Attitude Response
figure;
labels = {'Roll (°)', 'Pitch (°)', 'Yaw (°)'};
for i = 1:3
    subplot(3,1,i);
    plot(time, desired_angle(i,:), 'k--', 'LineWidth', 1.2); hold on;
    plot(time, angle(i,:), 'b', 'LineWidth', 1.4);
    ylabel(labels{i});
    legend('Desired', 'Actual');
    grid on;
end
xlabel('Time (s)');
sgtitle('Attitude Control Response');

% 2. Motor Outputs
figure;
for i = 1:4
    subplot(4,1,i);
    plot(time, MotorInputs(i,:), 'LineWidth', 1.2);
    ylabel(['Motor ', num2str(i)]);
    grid on;
end
xlabel('Time (s)');
sgtitle('Motor PWM Outputs');

% 3. 3D Flight Path
figure;
plot3(pos(1,:), pos(2,:), -pos(3,:), 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
grid on; axis equal;
title('3D Quadcopter Flight Path');
