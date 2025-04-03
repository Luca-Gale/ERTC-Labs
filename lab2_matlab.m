%DATA ACQUISITION:


%Serial Port Number: use 'seriallist' routine to identify,
%the second entry is what gives the communication port

COM_port = ''; 
packet_specification = {'double', 'double'}; % Define expected data format

% Start serial data logging
data = serial_datalog(COM_port, packet_specification);

time = (0:length(data(:,1))-1) * 0.01; % Time vector (assuming 100Hz sampling rate)

theta_acc = atan2(data(:,2), data(:,3)) * 180 / pi; % Compute tilt angle from accelerometer (atan2 ensures real result)

% Plot IMU tilt estimation
figure;
subplot(2,1,1);
plot(time, theta_acc, 'r', 'LineWidth', 1.5); hold on;
plot(time, data(:,4), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Tilt Angle (°)');
legend('Accelerometer Estimate', 'Control Signal');
title('Camera Stabilization Performance');
grid on;

% Plot control signal (PWM command)
subplot(2,1,2);
plot(time, data(:,4), 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Servo PWM Command');
title('Control Signal Over Time');
grid on;