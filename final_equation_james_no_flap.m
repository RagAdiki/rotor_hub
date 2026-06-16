%5cm

% plot_lag_beta_pitch_vs_tau.m
% Simulate the 3-DOF rotor example (lag, flap(beta), pitch) and plot
% lag, beta, pitch vs tau (time variable named tau).
% Uses the same example matrices as the demo script. Replace M,C,K,B with
% your exact matrices if you have them.

clear; close all; clc;

%% Parameters (example / from screenshots)
e      = 0.09;        % hinge eccentricity
l      = 0.607;
X_Ih   = 0.147;
gamma  = 1.56;
c_zeta = 0.0751;
c_beta = 0.0381;
cd0    = 0.06;
a_lift = 6.28;
theta0 = deg2rad(9);
k_drive = 9e-5;
c_drive = 9e-2;
k = 0.426;
sigmma=0.0746;


K_e=0.0025;
R_ohm=1.4;
I_beta=1.8*10^(-7);
phi34=a_lift*sigmma/12*(sqrt(1+24*theta0/a_lift/sigmma)-1);
zeta0=1/8*(l/e)*(1-4*e/3)*gamma*(theta0*phi34-phi34^2+cd0/a_lift);
u= a_lift*sigmma/(2*I_beta*gamma*40000)*K_e/R_ohm;
disp(zeta0);

beta0=1/8*(1/(1+e/l))*(1-4*e/3)*gamma*(theta0-phi34-cd0/a_lift*phi34);
%% Example M, C, K, B (replace with exact derived expressions if available)
M = zeros(3,3);
M(1,1) = 1 + X_Ih + 2*e/l + (e^2)/(k^2);

M(2,1) = -1 - e/l; 
M(1,2) = -1 - e/l; 
M(2,2) = 1;
M(3,3) = 1;



%C matrix

A = 2*cd0/a_lift + theta0*phi34;                 % (2*c_d0/a + θ0 φ3/4)
B = 1 - (4/3)*e;                             % (1 - 4/3 e)
C1 = 1 - (8/3)*e + 2*e^2;                      % (1 - 8/3 e + 2 e^2)
D = theta0 - 2*phi34;                         % (θ0 - 2 φ3/4)
E = 2*theta0 - (1 + cd0/a_lift)*phi34;             % (2θ0 - (1 + c_d0/a) φ3/4)

C = zeros(3,3);

% Row 1
C(1,1) =  (1/8) * gamma *A +c_drive/2;
C(1,2) = -(1/8) * gamma *A * B-2*e*zeta0/l;
C(1,3) = (1/8) * gamma * D * B-2*(1 + e/l)*beta0;

% Row 2
C(2,1) = -(1/8) * gamma *A * B+2*e*zeta0/l;
C(2,2) =  (1/8) * gamma *A * C1+c_zeta;
C(2,3) = -(1/8) * gamma *D * C1+2*beta0;

% Row 3
C(3,1) = -(1/8) * gamma *E * B+2*(1 + e/l)*beta0;
C(3,2) =  (1/8) * gamma *E * C1-2*beta0;
C(3,3) = (1/8) * gamma *(1 + cd0/a_lift) * C1+c_beta;
C=C*200

K = zeros(3,3);
K(1,1) = k_drive/2; K(1,2) = (gamma/8)*phi34; K(1,3) = 0;
K(2,1) = 0; K(2,2) = e/l-(gamma/8)*phi34*(1-4*e/3); K(2,3) = 0;
K(3,1) = 0; K(3,2) = (gamma/8)*(4*e/3-1); K(3,3) = 1+e/l;
K=K*40000;


B = [ gamma / (a_lift*0.0746); 0; 0 ];
B=B*40000;

%% Convert to first-order system: M x'' + C x' + K x = B u(t)
Minv = inv(M);
A = [ zeros(3), eye(3);
     -Minv*K,  -Minv*C ];
Bfull = [ zeros(3,1); Minv*B ];

%% Define input u(t) = tau_time_profile(tau)
% Choose the torque/time-profile you want. Examples below:
%tau_step = 1.0;               % constant torque amplitude for step
% Option 1: unit step (tau = constant)
%u_fun = @(tau) tau_step;

% Option 2: uncomment for a sinusoidal torque: tau(t) = 0.5*sin(10*t)
 u_fun = @(tau) u*0.175*sin(200*tau);

% Option 3: uncomment for a ramp torque: tau(t) = 0.5 * (tau/MaxTime)
% MaxTime = 2; u_fun = @(tau) 0.5 * (tau/MaxTime);

%% Simulation time (tau)
tau0 = 0;
tau_end = 1.0;       % seconds (rename of time variable to tau)
npts = 1000;
tau = linspace(tau0, tau_end, npts);

%% Integrate with ode45
y0 = zeros(6,1);
odefun = @(t,y) A*y + Bfull * u_fun(t);

opts = odeset('RelTol',1e-6,'AbsTol',1e-9);
[tt, yy] = ode45(odefun, tau, y0, opts);

% rename time vector to tau_out to match user notation


% extract generalized coordinates:
tau_out = tt
virtual_angle   = yy(:,1);   % x1 (lag)
lag  = yy(:,2);   % x2 (flap / beta)
flap = yy(:,3);   % x3 (pitch)
u_vals = arrayfun(u_fun, tau_out);


%% Plot lag, beta, pitch vs tau (4 subplots)
figure('Color','w','Units','normalized','Position',[0.1 0.1 0.7 0.7]);

subplot(4,1,1);
plot(tau_out, virtual_angle, 'LineWidth', 1.6);
xlabel('\tau (rad)'); ylabel('virtual_angle (x_1)');
title('virtual_angle vs \tau');
grid on;

subplot(4,1,2);
plot(tau_out, lag, 'LineWidth', 1.6);
xlabel('\tau (rad)'); ylabel('ζ (lag, x_2)');
title('ζ (lag) vs \tau');
grid on;

subplot(4,1,3);
plot(tau_out, flap, 'LineWidth', 1.6);
xlabel('\tau (rad)'); ylabel('flap (β)');
title('flap vs \tau');
grid on;

subplot(4,1,4);
plot(tau_out, u_vals, 'LineWidth', 1.6);
xlabel('\tau (rad)'); ylabel('u(\tau)');
title('Input u(\tau) vs \tau');
grid on;
% Optional: single-plot overlay (comment/uncomment as desired)
figure('Color','w','Position',[200 200 700 450]);
plot(tau_out, virtual_angle,'LineWidth',1.4); hold on;
plot(tau_out, lag,'LineWidth',1.4);
plot(tau_out, flap,'LineWidth',1.4);
xlabel('\tau (s)'); ylabel('Displacement (nondim)');
legend('virtual_angle (x_1)','lag (x_2)','beta (x_3)','Location','northwest');
title('virtual_angle, lag, flap vs \tau');
grid on;


% End of script