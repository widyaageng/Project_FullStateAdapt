%--------------------------------------------------------
% Adaptive Control,Full State Measurable
%--------------------------------------------------------
clear;clc;

%% Labview Sensor constants calibration, Ktheta, Kw

%Ktheta calibration using angle position, shaft dial
pos_degree = [258 288 318 348 378 408];
pos_radiant = pos_degree/(180/pi); % unit angle position to be used in the control algorithm structure
x1 = [0 0.84 1.734 2.642 3.516 4.445]; % Labview readouts
Fittheta = polyfit(pos_radiant,x1,1);
Ktheta = Fittheta(1); % its reciprocal to be entered into labview block

%Fitting plot of K theta
figure;hold on;grid;
plot(pos_radiant,x1,'.r','MarkerSize',10);
minplot_theta = ((min(x1)-Fittheta(2))/Fittheta(1))-1;
maxplot_theta = ((max(x1)-Fittheta(2))/Fittheta(1))+1;
plot(minplot_theta:0.01:maxplot_theta,Fittheta(1)*(minplot_theta:0.01:maxplot_theta)+Fittheta(2),'-b');
axis tight; legend('data','fitting line');
xlabel('Angular Position (rad)');
ylabel('Potentiometer output (Volts)');
title('K_{\theta} calibration');hold off;

%Kw calibration using digital tachometer on the module
w_rpm = [-162 -104 -46 0 47 104 162];
w_radps = w_rpm*2*pi/60; % unit angle velocity to be used in the control algorithm structure
x2 = [-3.39 -2.17 -0.95 0 0.95 2.12 3.39];
Fitw = polyfit(w_radps,x2,1);
Kw = Fitw(1); % its reciprocal to be entered into labview block

%Fitting plot of Kw
figure;hold on;grid;
plot(w_radps,x2,'.r','MarkerSize',10);
minplot_w= ((min(x2)-Fitw(2))/Fitw(1))-1;
maxplot_w = ((max(x2)-Fitw(2))/Fitw(1))+1;
plot(minplot_w:0.01:maxplot_w,Fitw(1)*(minplot_w:0.01:maxplot_w)+Fitw(2),'-b');
axis tight; legend('data','fitting line');
xlabel('Angular velocity (rads/sec)');
ylabel('Tachogenerator output (Volts)');
title('K_{\omega} calibration');hold off;


%% Reference model selection

%second order parameter
damping_rat = 1;
nat_freq = 2;

%state space representation of reference model
Am = [0 1; -(nat_freq^2) -2*damping_rat*nat_freq]; % to be entered into labview block
B = [0;1];% to be entered into labview block
gm = nat_freq^2;% to be entered into labview block


%% Other Adaptive Controller Gain Matrices Setup & Plant Characterisation

% Adaptive Gain Gamma
Gamma = [3 0 0;0 0.5 0;0 0 2]; % to be entered into labview block
%Gamma = [1 0 0;0 1 0;0 0 1];

% Q +ve definite matrix
Q = [4 0; 0 0.5];
%Q = [1 0; 0 1];

% Calculating ARE equation Am'P + PAm = -Q
P = lyap(Am',Q); % to be entered into labview block

%Pretest to determine DC gain of motor system, Kp
u_labview = [-3 -2 -1 0 1 2 3]; % Experiment
Kpvec = polyfit(u_labview,w_radps,1); % Fitting
Kp = Kpvec(1);

%Fitting plot of Kp
figure;hold on;grid;
plot(u_labview,w_radps,'.r','MarkerSize',10);
minplot_u = ((min(w_radps)-Kpvec(2))/Kpvec(1))-1;
maxplot_u = ((max(w_radps)-Kpvec(2))/Kpvec(1))+1;
plot(minplot_u:0.01:maxplot_u,Kpvec(1)*(minplot_u:0.01:maxplot_u)+Kpvec(2),'-b');
axis tight; legend('data','fitting line');
xlabel('Input Voltage (Volts)');
ylabel('Angular velocity (rads/sec)');
title('K_{p} calibration');hold off;

% Plant's parameter empirical determination through step response test
tau = 0.31; % Experiment
Mot_system = tf(Kp,[tau 1]);

% Plant's state space representation
Ap = [0 1;0 -1/tau];
g = Kp/tau;

% Approximation of perfect controller gain Theta star
% from Matching Condition
Theta_x_star = (B'*(Am-Ap)/(g*(B'*B)))';
Theta_r_star = gm/g;

%augmented theta
aug_theta_star = [Theta_x_star;Theta_r_star];

