%% combined data
Vz_Vh = [0, -0.2, -0.4, -0.6, -0.8, -0.9, -0.95, -1, -1.2, -1.4, -1.6, -1.8, -2,
         0, -0.2, -0.4, -0.6, -0.8, -0.9, -0.95, -1, -1.2, -1.4, -1.6, -1.8, -2,
         0, -0.2, -0.4, -0.6, -0.8, -0.9, -0.95, -1, -1.2, -1.4, -1.6, -1.8, -2];
Ct_Ct0 = [1.01, 1.033561256, 1.02574431, 1.016905479, 0.9886233356, 0.8970075219, 0.8777239092, 0.8355080194, 0.6365858976, 0.8193207782, 0.9771476477, 1.205411694, 1.479053926,
          1.01, 1.019862598, 1.022487682, 1.036115946, 1.007347708, 0.9839842248, 0.9786538544, 0.9802922029, 0.9320008793, 0.8571452231, 0.8878504896, 1.11776332, 1.286220286,
          1.01, 1.012799538, 1.010993231, 1.005584499, 0.9918431202, 0.9742899559, 0.966401586, 0.9580817711, 0.9240400687, 0.8932144368, 0.8924124454, 1.097631645, 1.364297782];

RPM = [2000, 6000, 10000];
Ct0 = [0.007804640048, 0.007095629221, 0.006776539105];

%% fit
p = polyfit(Vz_Vh, Ct_Ct0, 4);
Vz_Vh_range = linspace(-2.2, 0, 100);
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
format long
disp(p);

disp('Fitted parameters 2: ');
format long
disp(p1);