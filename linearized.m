
clear; close all; clc;

%% ---------------- GLOBAL VISUAL SETTINGS ----------------
set(groot,'defaultFigureColor','w');
set(groot,'defaultAxesColor','w');
set(groot,'defaultAxesGridColor',[0.85 0.85 0.85]);
set(groot,'defaultAxesXColor',[0.1 0.1 0.1]);
set(groot,'defaultAxesYColor',[0.1 0.1 0.1]);

col_virtual = [0.00 0.32 0.60];
col_speed   = [0.00 0.00 0.00];
col_lag     = [0.80 0.40 0.00];
col_flap    = [0.20 0.55 0.25];
col_eq      = [0.55 0.55 0.55];

%% ---------------- PARAMETERS ----------------

%% ---------------- PARAMETERS ----------------
R= 0.1;
m=0.39e-3;
e=0.09;
I_h=5.1*10^(-7);
theta0=deg2rad(8);
cd0=0.06;
a_lift=6.28;
K_e=0.0025;
K_p= 0.03;
R_ohm=1.4;
K_I= 0.03;
gamma=2.18;
c_zeta=0.0751;
c_beta=0.0381; 
c = 5.9e-3;



sigmma=0.0746;
theta0_etta0=3;

Omega0 = 200;
V = 0.25;


l=2*(1-e)/3;
I_beta=((1-e)^2)*m*R*R/3;

k=(1-e)/(3)^0.5;
k_drive=K_I*K_e/(R_ohm*Omega0*Omega0*I_beta);
c_drive=(K_p+K_e)*K_e/(R_ohm*Omega0*I_beta); 
X_Ih=I_h/(2*I_beta);
u= a_lift*sigmma/(2*I_beta*gamma*40000)*K_e/R_ohm;


phi34 = a_lift*sigmma/12*(sqrt(1+24*theta0/(a_lift*sigmma))-1);
zeta0 = (1/8)*(l/e)*(1-4*e/3)*gamma*(theta0*phi34-phi34^2+cd0/a_lift);
beta0 = (1/8)*(1/(1+e/l))*(1-4*e/3)*gamma*(theta0-phi34-cd0/a_lift*phi34);

%% ---------------- MATRICES ----------------
M = [1+X_Ih+2*e/l+e^2/k^2  -1-e/l  0;
     -1-e/l               1       0;
      0                   0       1];



A_C=2*cd0/a_lift+theta0*phi34;
B_C=1-(4/3)*e;
C1_C=1-(8/3)*e+2*e^2;
D_C=theta0-2*phi34;
E_C=2*theta0-(1+cd0/a_lift)*phi34;

C=zeros(3);
C(1,1)=(1/8)*gamma*A_C+c_drive/2;
C(1,2)=-(1/8)*gamma*A_C*B_C-2*e*zeta0/l;
C(1,3)=(1/8)*gamma*D_C*B_C-2*(1+e/l)*beta0;
C(2,1)=-(1/8)*gamma*A_C*B_C+2*e*zeta0/l;
C(2,2)=(1/8)*gamma*A_C*C1_C+c_zeta;
C(2,3)=-(1/8)*gamma*D_C*C1_C+2*beta0;
C(3,1)=-(1/8)*gamma*E_C*B_C+2*(1+e/l)*beta0;
C(3,2)=(1/8)*gamma*E_C*C1_C-2*beta0;
C(3,3)=(1/8)*gamma*(1+cd0/a_lift)*C1_C+c_beta;
C=C*Omega0;

K=zeros(3);
K(1,1)=k_drive/2;
K(1,2)=(gamma/8)*phi34*theta0_etta0;
K(2,2)=e/l-(gamma/8)*phi34*(1-4*e/3)*theta0_etta0;
K(3,2)=(gamma/8)*(4*e/3-1)*theta0_etta0;
K(3,3)=1+e/l;
K=K*Omega0^2;

B=[gamma/(a_lift*sigmma);0;0]*Omega0^2;

Minv=inv(M);
A_sys=[zeros(3) eye(3); -Minv*K -Minv*C];
B_sys=[zeros(3,1); Minv*B];

Q_fun=@(t)u*V*sin(Omega0*t);

% 
% % % % % 
% % Indices to KEEP (remove beta = 3 and beta_dot = 6)
% keep = [1 2 4 5];
% 
% A_sys_r = A_sys(keep, keep);
% B_sys_r = B_sys(keep);
% 
% % New initial condition
% y0_r = zeros(4,1);
% t=linspace(0,0.5,25000);
% 
% [t_out, y_r] = ode45(@(t,y) A_sys_r*y + B_sys_r*Q_fun(t), t, y0_r);
% 
% % Extract reduced states
% x1      = y_r(:,1);
% lag     = y_r(:,2) + zeta0;
% x1_dot  = y_r(:,3);
% lag_dot = y_r(:,4);
% 
% % Define beta as identically zero for plotting
% flap = zeros(size(t_out));

% % % % 
% ---------------- INTEGRATION ----------------
t=linspace(0,0.5,25000);

[t_out,y]=ode45(@(t,y)A_sys*y+B_sys*Q_fun(t),t,zeros(6,1));

x1=y(:,1); 
x1_dot=y(:,4);
lag=y(:,2)+zeta0; 
flap=y(:,3)+beta0;

 psi_deg=Omega0*t_out*180/pi;

%% ---------------- FIGURE 1: VIRTUAL ANGLE (LONG TIME) ----------------
figure;
plot(psi_deg,x1,'Color',col_virtual,'LineWidth',2.2); hold on;
plot([psi_deg(1) psi_deg(end)],[0 0],'--','Color',col_eq,'LineWidth',1.4);
xlabel('Azimuth \psi (deg)');
ylabel('Virtual angle x_1');
title('Virtual Angle – Long Time (Including Transient)');
grid on; box on;

%% ---------------- STEADY STATE: ONE REV ----------------
idx_ss=round(0.6*numel(psi_deg)):numel(psi_deg);
psi_ss=psi_deg(idx_ss);
psi1=psi_ss(end)-360;

idx1=psi_ss>=psi1;
psi1rev=psi_ss(idx1)-psi1;
dOm1=x1_dot(idx_ss); dOm1=dOm1(idx1)+200;
lag1=lag(idx_ss); lag1=lag1(idx1);
flap1=flap(idx_ss); flap1=flap1(idx1);

%% ---------------- FIGURE 2: ONE-REV STACK (WITH TORQUE) ----------------
Q1  = 0.037241173 * V * sind(psi1rev);
Q1n = Q1 / max(abs(Q1));   % normalized torque

figure;

subplot(5,1,1);
plot(psi1rev, Q1n, 'k', 'LineWidth', 2); hold on;
plot([0 360],[0 0],'--','Color',col_eq,'LineWidth',1.2);
ylabel('Torque (norm)');
title('One-Revolution Steady Response');
grid on; xlim([0 360]);

subplot(5,1,2);
plot(psi1rev, dOm1,'Color',col_speed,'LineWidth',2); hold on;
plot([0 360],[200 200],'--','Color',col_eq,'LineWidth',1.2);
ylabel('\Delta\Omega');
grid on; xlim([0 360]);

subplot(5,1,3);
plot(psi1rev, lag1,'Color',col_lag,'LineWidth',2); hold on;
plot([0 360],[zeta0 zeta0],'--','Color',col_eq,'LineWidth',1.2);
ylabel('\zeta');
grid on; xlim([0 360]);

subplot(5,1,4);
plot(psi1rev, flap1*57.32,'Color',col_flap,'LineWidth',2); hold on;
plot([0 360],[beta0 beta0],'--','Color',col_eq,'LineWidth',1.2);
xlabel('Azimuth \psi (deg)');
ylabel('\beta');
grid on; xlim([0 360]);


subplot(5,1,5);
plot(psi1rev, (theta0 + theta0_etta0*lag1)*57.32,'Color',col_lag,'LineWidth',2); hold on;
ylabel('\pitch');
grid on; xlim([0 360]);

