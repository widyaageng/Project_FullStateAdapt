
%% Plotting theta trajectories and exact theta
%% Calculating exact theta

%Reference Model
%second order parameter
damping_rat = 1;
nat_freq = 2;

%state space representation of reference model
Am = [0 1; -(nat_freq^2) -2*damping_rat*nat_freq]; % to be entered into labview block
B = [0;1];% to be entered into labview block
gm = nat_freq^2;% to be entered into labview block

% Calibration data
w_rpm = [-162 -104 -46 0 47 104 162];
w_radps = w_rpm*2*pi/60; % unit angle velocity to be used in the control algorithm structure
u_labview = [-3 -2 -1 0 1 2 3]; % Experiment
Kpvec = polyfit(u_labview,w_radps,1); % Fitting
Kp = Kpvec(1);

% Plant's parameter empirical determination through step response test
tau = 0.31; % Experiment
Mot_system = tf(Kp,[tau 1]);

% Plant's state space representation
Ap = [0 1;0 -1/tau];
g = Kp/tau;

% Approximation of perfect controller gain Theta star
% from Matching Condition
Theta_x_star_plot = [(Theta.Time>=0) (Theta.Time>=0)]*diag((B'*(Am-Ap)/(g*(B'*B)))');
Theta_r_star_plot = (Theta.Time>=0)*gm/g;

% finding axes bounds
min_y_plot1 = min(min(Theta.Data(:,1)),min(Theta_x_star_plot(:,1)));
max_y_plot1 = max(max(Theta.Data(:,1)),max(Theta_x_star_plot(:,1)));
min_y_plot2 = min(min(Theta.Data(:,2)),min(Theta_x_star_plot(:,2)));
max_y_plot2 = max(max(Theta.Data(:,2)),max(Theta_x_star_plot(:,2)));
min_y_plot3 = min(min(Theta.Data(:,3)),min(Theta_r_star_plot(:,1)));
max_y_plot3 = max(max(Theta.Data(:,3)),max(Theta_r_star_plot(:,1)));

figure;grid;
subplot(3,1,1);hold on;grid;title('\Theta vs \Theta^*');
plot(Theta.Time,Theta.Data(:,1),'-r');
plot(Theta.Time,Theta_x_star_plot(:,1),'-.b');
axis([min(Theta.Time) max(Theta.Time) min_y_plot1-0.5 max_y_plot1+0.5]);
legend('\theta_{x1}','\theta_{x1} exact');hold off;

subplot(3,1,2);hold on;grid;
plot(Theta.Time,Theta.Data(:,2),'-r');
plot(Theta.Time,Theta_x_star_plot(:,2),'-.b');
axis([min(Theta.Time) max(Theta.Time) min_y_plot2-0.5 max_y_plot2+0.5]);
legend('\theta_{x2}','\theta_{x2} exact');hold off;

subplot(3,1,3);hold on;grid;
plot(Theta.Time,Theta.Data(:,3),'-r');
plot(Theta.Time,Theta_r_star_plot(:,1),'-.b');
axis([min(Theta.Time) max(Theta.Time) min_y_plot3-0.5 max_y_plot3+0.5]);
legend('\theta_r','\theta_r exact');hold off;
