% Retrieve data from the base workspace
rpm_values = evalin('base', 'rpm_values');
Vh_values = evalin('base', 'Vh_values');
coefficients = evalin('base', 'coefficients');

% Generate fitted values using the polynomial coefficients
fitted_Vh_values = polyval(coefficients, rpm_values);

% Plot the original data points
figure;
plot(rpm_values, Vh_values, 'bo', 'DisplayName', 'Original Data');
hold on;

% Plot the fitted curve
plot(rpm_values, fitted_Vh_values, 'r-', 'DisplayName', 'Fitted Curve');
hold off;

% Add labels and title
xlabel('RPM');
ylabel('Induced Velocity (Vh)');
title('Induced Velocity vs. RPM with Curve Fit');
legend show;
grid on;
