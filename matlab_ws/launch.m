m = 1;
R = 0.127;
CDval = 0.008;
Arm = [
0.2*(1/sqrt(2)) -0.2*(1/sqrt(2)) 0.2*(1/sqrt(2)) -0.2*(1/sqrt(2))
0.2*(1/sqrt(2)) -0.2*(1/sqrt(2)) -0.2*(1/sqrt(2)) 0.2*(1/sqrt(2))
0 0 0 0
];
prop_inertia = (1/12) * 0.015 * 0.127^2;
tilt_speed = [0; pi/4; 0];
SampleTime = 0.01;

m_to_mm = 1000;

%% environmental variables
g = 9.81;
open('rotor.slx')
