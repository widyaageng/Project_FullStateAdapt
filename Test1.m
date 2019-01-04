clearvars; clc;
%calibration

%calibration for position Ktheta
xdeg = [258 288 318 348 378 408];
xrad = xdeg/(180/pi);
x1 = [0 0.84 1.734 2.642 3.516 4.445];
Ktheta = polyfit(xrad,x1,1);
disp(['K theta is equal to ', num2str(Ktheta(1))])

%calibration for velocity Komega
rpm = [-162 -104 -46 0 47 104 162];
radsec = rpm*2*pi/60;
u = [-3 -2 -1 0 1 2 3];
x2 = [-3.39 -2.17 -0.95 0 0.95 2.12 3.39];
Komega = polyfit(radsec,x2,1);
disp(['K omega is equal to ', num2str(Komega(1))])
Kstatic = polyfit(u,radsec,1);
disp(['K static is equal to ', num2str(Kstatic(1))])

%Empirical plant
tau = 0.31;
DCmotor = tf([Kstatic(1)],[tau 1]);

% Ref model

%choose wm and cWW
c = 1;
wm = 2;

Am = [0 1; -(wm^2) -2*c*wm];
gm = [0;wm^2];

%choose Q
Q = [4 0; 0 .5];

%calculate ARE, P
P = lyap(Am',Q)

% Adaptive law