%% combined data
Vz_Vh = [0, -0.2, -0.4, -0.6, -0.8, -0.9, -0.95, -1, -1.2, -1.4, -1.6, -1.8, -2];
Ct_Ct0 = [1, 1.018, 1.091, 1.140, 1.237, 1.294, 1.339, 1.404, 1.619, 1.697, 1.772, 1.862, 2.011];
Ct_Ct0_1 = [1, 1.035, 1.087, 1.164, 1.255, 1.312, 1.338, 1.370, 1.531, 1.689, 1.835, 1.969, 2.134];
Ct_Ct0_2 = [1, 1.036, 1.085, 1.155, 1.244, 1.288, 1.308, 1.335, 1.484, 1.651, 1.802, 1.917, 2.033];


%% fit
p = polyfit(Vz_Vh, Ct_Ct0, 4);
Vz_Vh_range = linspace(-2.5, 0, 100);
fitted_curve = polyval(p, Vz_Vh_range);

%% plot
figure;
hold on;
scatter(Vz_Vh, Ct_Ct0, 'blue', 'DisplayName', '2000 RPM');
scatter(Vz_Vh, Ct_Ct0_1, 'cyan','DisplayName', '6000 RPM');
scatter(Vz_Vh, Ct_Ct0_2, 'MarkerEdgeColor', "[0 0.4470 0.7410]", 'DisplayName', '10000 RPM');

plot(Vz_Vh_range, fitted_curve, 'r', 'DisplayName', 'Fitted curve');
xlabel('Vz/Vi_0');
ylabel('Ct/Ct_0');
legend('show');
title('45° Tilt Ct/Ct_0 vs Vz/Vi_0 with Quartic Polynomial Fit');
grid on;
hold off;

f = gcf;
exportgraphics(f, 'VzVh Tilted.pdf', 'ContentType', 'vector');


%% disp
disp('Fitted parameters 1 (quartic polynomial):');
disp(p);
