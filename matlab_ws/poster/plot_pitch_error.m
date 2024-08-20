% Load the data
load('pitch_error_data.mat');

% Create a figure with two subplots
figure;

% First subplot (empty or placeholder)
subplot(2, 1, 1);
title('Top Subplot (Placeholder)');
xlabel('Time (seconds)');
ylabel('Data');
% You can add your desired content here for the first subplot

% Second subplot - Tilted and Non-Tilted Pitch Error
subplot(2, 1, 2);
hold on;

% Plot tilted pitch error
plot(tilted_timestamp, tilted_pitch_error, 'color', [0 0.5 0], 'LineWidth', 1);

% Plot non-tilted pitch error
plot(non_tilted_timestamp, non_tilted_pitch_error, 'r-', 'LineWidth', 1);

% Add markers
scatter(stop_timestamp, 0, 100, 'rx', 'LineWidth', 1); % Ground Contact
xline(recovery_marker_timestamp, 'k-', 'LineWidth', 1); % 20m Recovery

% Add labels and title
title('Tilted and Non-Tilted Pitch Error (Degrees) with Markers');
xlabel('Time (seconds)');
ylabel('Pitch Error (degrees)');
legend('Tilt Control', 'No Tilt Control', 'Ground Contact', '20m Recovery', 'Location','best');

hold off;
f = gcf;
exportgraphics(f, 'pitch.pdf', 'ContentType', 'vector');