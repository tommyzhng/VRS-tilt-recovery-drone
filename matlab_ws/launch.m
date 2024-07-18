%% environmental variables
g = 9.81;

%% drone variables
m = 1;
R = 0.127;
CDval = 0.008;
CDDrone = 0.01;
Arm = [
0.2*(1/sqrt(2)) -0.2*(1/sqrt(2)) 0.2*(1/sqrt(2)) -0.2*(1/sqrt(2))
0.2*(1/sqrt(2)) -0.2*(1/sqrt(2)) -0.2*(1/sqrt(2)) 0.2*(1/sqrt(2))
0 0 0 0
];
Ixx = 0.0126; %Kgm^2
Iyy = 0.0126;
Izz = 0.0253;

inertia = diag([Ixx, Iyy, Izz]);

rotorRPMMax = 10000;
rotorTimeConst = 0.05;
prop_inertia = (1/12) * 0.015 * 0.127^2;
tilt_speed = [0; pi/4; 0];

% Contact model
contact.translation.spring = 3100;
contact.translation.damper = 100;
contact.translation.friction = 0.5;
contact.translation.vd = 0.02;
contact.translation.maxFriction = 20;
contact.translation.maxNormal = 80;

contact.rotation.spring = 2;
contact.rotation.damper = 1;
contact.rotation.friction = 0.03;
control.rotation.maxMoment = 0.1;
control.rotation.friction = 0.025;
control.rotation.vd = 0.2;

%%initial states
init.equilibriumZ =  m*g/contact.translation.spring;
init.posNED = [0, 0, init.equilibriumZ]; % m
init.vb = [0 0 0]'; %m/s
init.euler = [0, 0, 0]'; %Roll Pitch Yaw Rads
init.angRates = [0, 0, 0]; %rad/s
ref_lat = 437818446e-7;
ref_lon = -794679715e-7;
ref_height = 76;

%Gain to convert m to mm
m_to_mm = 1000;

%Gain to convert uT to Gauss
uT_to_gauss = 0.01;

%Gain to convert m/s^2 to mg
ms2_to_mg = (1/9.80665)*1000;

%Gain to convert m/s to cm/s
ms_to_cms = 100;

% Maximum Serial data read size from Pixhawk
MAVLink_Input_Read_Size = 1024;

% Sample Time of Plant and Controller (100 Hz)
SampleTime = 0.004;

%open('SILsim.slx')