%% combined data
Vz_Vh = [0, -0.2828427125, -0.5656854249, -0.8485281374, -1.13137085, -1.272792206, -1.343502884, -1.414213562, -1.697056275, -1.979898987, -2.2627417, -2.545584412, -2.828427125,...
         0, -0.2828427125, -0.5656854249, -0.8485281374, -1.13137085, -1.272792206, -1.343502884, -1.414213562, -1.697056275, -1.979898987, -2.2627417, -2.545584412, -2.828427125,...
         0, -0.2828427125, -0.5656854249, -0.8485281374, -1.13137085, -1.272792206, -1.343502884, -1.414213562, -1.697056275, -1.979898987, -2.2627417, -2.545584412, -2.828427125];
Ct_Ct0 = [1, 1.018, 1.091, 1.140, 1.237, 1.294, 1.339, 1.404, 1.619, 1.697, 1.772, 1.862, 2.011,...
          1, 1.035, 1.087, 1.164, 1.255, 1.312, 1.338, 1.370, 1.531, 1.689, 1.835, 1.969, 2.134,...
          1, 1.036, 1.085, 1.155, 1.244, 1.288, 1.308, 1.335, 1.484, 1.651, 1.802, 1.917, 2.033];

RPM = [2000, 6000, 10000];
Ct0 = [0.007804640048, 0.007095629221, 0.006776539105];

%% fit
p = polyfit(Vz_Vh, Ct_Ct0, 4);
Vz_Vh_range = linspace(-3, 0, 100);
fitted_curve = polyval(p, Vz_Vh_range);

p1 = polyfit(RPM, Ct0, 2);
RPM_range = linspace(0, 10000, 1000);
fitted_curve1 = polyval(p1, RPM_range);

%% plot
figure;
scatter(Vz_Vh, Ct_Ct0, 'b', 'DisplayName', 'Data');
hold on;
plot(Vz_Vh_range, fitted_curve, 'r', 'DisplayName', 'Fitted curve');
xlabel('Vz/Vh');
ylabel('Ct/Ct0');
legend('show');
title('Ct/Ct0 vs Vz/Vh with Quartic Polynomial Fit');
grid on;
hold off;

figure;
scatter(RPM, Ct0, 'b', 'DisplayName', 'Data');
hold on;
plot(RPM_range, fitted_curve1, 'r', 'DisplayName', 'Fitted curve');
xlabel('RPM');
ylabel('Ct0');
legend('show');
title('Ct at hover vs RPM with Quadratic Polynomial Fit');
grid on;
hold off;

%% disp
disp('Fitted parameters 1 (quartic polynomial):');
disp(p);

disp('Fitted parameters 2: ');
format long
disp(p1);