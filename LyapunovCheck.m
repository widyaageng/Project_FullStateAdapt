%-----------------------------------------------------
%-- LYAPUNOV CHECK & BOUNDEDNESS OF VARIOUS SIGNALS---
%-----------------------------------------------------

clear;clc;

%% Initiating state error and parameter error vectors
state_err = [0;0];
param_err = [0;0;0];

% x uniform stability theorem, page 15 Lect Note part 1
aug_err = [state_err;param_err];

P = eye(2);
Gamma = eye(3);
aug_Ly_mat = [P zeros(2,3);zeros(3,2) Gamma];


%% Bounding Lyapunov function with scalar functions
% increasing element in state vector x, means the L2 norm (||x||) increases
t = (0:0.01:2.5/sqrt(5))';
abs_t = sqrt(5)*t;
state_inc_mat = ones(size(aug_err,1),1)*t';

%Lower bound non-decreasing scalar function, a(x)=Lyap*(1-exp(-e'e));
low_a_vec = state_inc_mat(:,1)'*aug_Ly_mat*state_inc_mat(:,1)*(1-exp((-1)*state_inc_mat(:,1)'*state_inc_mat(:,1)));

%Lyapunov function value, x'Px + phi'(Gamma^-1)phi
Lyap_val = state_inc_mat(:,1)'*aug_Ly_mat*state_inc_mat(:,1);

%Upper bound non-decreasing scalar function, b(x)=Lyap*(1+exp(-e'e));
upp_b_vec = state_inc_mat(:,1)'*aug_Ly_mat*state_inc_mat(:,1)*(1+exp((-1)*state_inc_mat(:,1)'*state_inc_mat(:,1)));

%iterating along state axis,|x|
for i=2:length(t)
    low_a_vec = [low_a_vec;(state_inc_mat(:,i)'*aug_Ly_mat*state_inc_mat(:,i))*(1-exp((-1)*state_inc_mat(:,i)'*state_inc_mat(:,i)))];
    Lyap_val = [Lyap_val;state_inc_mat(:,i)'*aug_Ly_mat*state_inc_mat(:,i)];
    upp_b_vec = [upp_b_vec;state_inc_mat(:,i)'*aug_Ly_mat*state_inc_mat(:,i)*(1+exp((-1)*state_inc_mat(:,i)'*state_inc_mat(:,i)))];
end

% Plotting of bounding functions and Lyapunov function
figure;
hold on;grid;
plot(abs_t,low_a_vec,'-r');
plot(abs_t,Lyap_val,'-.k');
plot(abs_t,upp_b_vec,'-b');
hold off;axis tight;
xlabel('|x|');
title('Bounds of Lyapunov function');
legend('a(x)','V(x)','b(x)');

%% Example of error spike during asymptotic tracking against zero which can be avoided using adaptive law

% create exponential graph with spikes
sig_t = (0:0.005:10)';
sig_e = exp((-1)*sig_t);
noi_e = (0.1*sin(2*pi*1*sig_t)).*(exp(-0.9*sig_t));
noi_e = noi_e + 0.005*sin(2*pi*(10*rand()*sig_t)).*(exp(-0.9*sig_t));
spikes = 0.1*(sig_t==8)-0.3*(sig_t==9)+0.2*(abs(sig_t-7.7)<1e-10);
sig_e = sig_e + spikes + noi_e;
figure;hold on;grid;
plot(sig_t,sig_e,'-r');
ylabel('e');
xlabel('t');
title('example of state error convergence towards 0');
legend('state error, x_1');
hold off;
