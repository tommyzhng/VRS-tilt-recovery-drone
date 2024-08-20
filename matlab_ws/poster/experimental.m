% Load the data
load('data_for_matlab_final.mat');

% Create the figure
figure;

% Subplot 1: Local Position Z
subplot(2, 1, 1);
plot(notilt_time_data_position, notilt_local_position_data, 'b', 'DisplayName', '0° Tilt', 'LineWidth', 1);
hold on;
plot(tilt15_time_data_position, tilt15_local_position_data, 'color', [1 0.5 0], 'DisplayName', '15° Tilt', 'LineWidth', 1);
xline(recovery_time_seconds, 'k', 'LineWidth', 1, 'DisplayName', '20m Recovery');
xlabel('Time (seconds)');
ylabel('Altitude (meters)');
legend;
grid on;
hold off;

% Subplot 2: Yaw Error with a darker green color
subplot(2, 1, 2);
plot(notilt_time_data_yaw, notilt_yaw_error_data, 'r', 'DisplayName', '0° Tilt', 'LineWidth', 1);
hold on;
plot(tilt15_time_data_yaw, tilt15_yaw_error_data, 'color', [0 0.5 0], 'DisplayName', '15° Tilt', 'LineWidth', 1);
xline(recovery_time_seconds, 'k', 'LineWidth', 1, 'DisplayName', '20m Recovery');
xlabel('Time (seconds)');
ylabel('Yaw Error (deg)');
legend;
grid on;
hold off;

% Add shortened title to the figure
sgtitle('Altitude and Yaw Error Comparison (Experimental)');

f = gcf;
exportgraphics(f, 'Experimental.pdf', 'ContentType', 'vector');