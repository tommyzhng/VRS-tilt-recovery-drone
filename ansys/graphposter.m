Vz_Vh_Sweep = [0,
-0.2,
-0.4,
-0.6,
-0.8,
-0.9,
-0.95,
-1,
-1.2,
-1.4,
-1.6,
-1.8,
-2];

%% Tilted Prop
% Data for RPM 2000
Tilt_Vh_2000 = 1.66158964;
Tilt_Ct0_2000 = 0.007804640048;
Tilt_Ct_2000 = [1, 1.018189915, 1.090540468, 1.140172693, 1.236948411,1.294126366 ,1.338782061, 1.403502185, 1.618923155, 1.69657135, 1.772131462, 1.86169902, 2.011279236];

% Data for RPM 6000
Tilt_Vh_6000 = 4.752958784;
Tilt_Ct0_6000 = 0.007095629221;
Tilt_Ct_6000 = [1, 1.03536684, 1.086897991, 1.163847095, 1.255382818, 1.31169761, 1.338264834, 1.369676708, 1.531358299, 1.689285604, 1.834652481, 1.969320206, 2.133656805];

% Data for RPM 10000
Tilt_Vh_10000 = 7.741432217;
Tilt_Ct0_10000 = 0.006776539105;
Tilt_Ct_10000 = [1, 1.035587192, 1.084898379, 1.154714436, 1.244372259, 1.288208642, 1.308213597, 1.335046672, 1.484462851, 1.651201255, 1.802341965, 1.916672143, 2.032526001];


%% Non Tilted Prop

NoTilt_Vh_2000 = 1.633948495;
NoTilt_Ct0_2000 = 0.007547133843;
NoTilt_Ct_2000 = [1, 1.033561256, 1.02574431, 1.016905479, 0.9886233356, 0.8970075219, 0.8777239092, 0.8355080194, 0.6365858976, 0.8193207782, 0.9771476477, 1.205411694, 1.479053926];

NoTilt_Vh_6000 = 4.7505755;
NoTilt_Ct0_6000 = 0.007088515058;
NoTilt_Ct_6000 = [1, 1.019862598, 1.022487682, 1.036115946, 1.007347708, 0.9839842248, 0.9786538544, 0.9802922029, 0.9320008793, 0.8571452231, 0.8878504896, 1.11776332, 1.286220286];

NoTilt_Vh_10000 = 7.767194431;
NoTilt_Ct0_10000 = 0.006821716568;
NoTilt_Ct_10000 = [1, 1.012799538, 1.010993231, 1.005584499, 0.9918431202, 0.9742899559, 0.966401586, 0.9580817711, 0.9240400687, 0.8932144368, 0.8924124454, 1.097631645, 1.364297782];

%% Plotting Organized by RPM with Correct Legend Grouping
figure;

% Define colors for non-tilted and tilted props
color_no_tilt = [0.8500 0.3250 0.0980]; % Red for non-tilted (0°)
color_tilt = [0 0.4470 0.7410]; % Blue for tilted (45°)

% Plot all 0° Tilt first
plot(Vz_Vh_Sweep, NoTilt_Ct_2000, '-x', 'Color', color_no_tilt, 'DisplayName', '0° Tilt - RPM 2000');
hold on;
plot(Vz_Vh_Sweep, NoTilt_Ct_6000, '-o', 'Color', color_no_tilt, 'DisplayName', '0° Tilt - RPM 6000');
plot(Vz_Vh_Sweep, NoTilt_Ct_10000, '-d', 'Color', color_no_tilt, 'DisplayName', '0° Tilt - RPM 10000');

% Plot all 45° Tilt second
plot(Vz_Vh_Sweep, Tilt_Ct_2000, '-x', 'Color', color_tilt, 'DisplayName', '45° Tilt - RPM 2000');
plot(Vz_Vh_Sweep, Tilt_Ct_6000, '-o', 'Color', color_tilt, 'DisplayName', '45° Tilt - RPM 6000');
plot(Vz_Vh_Sweep, Tilt_Ct_10000, '-d', 'Color', color_tilt, 'DisplayName', '45° Tilt - RPM 10000');

% Title and labels
title('Normalized Thrust Coefficients in Descent: 0° vs 45° at different RPMs');
xlabel('Vz/Vi_0');
ylabel('Ct/Ct_0');
legend;
grid on;
ylim([0 2.5]);

legend({'0° Tilt - RPM 2000', '0° Tilt - RPM 6000', '0° Tilt - RPM 10000', ...
        '45° Tilt - RPM 2000', '45° Tilt - RPM 6000', '45° Tilt - RPM 10000'});


% Save the figure
saveas(gcf,'rpm_comparison.pdf')
