clear; clc;

%% ---------------- PARAMETERS ----------------
e       = 0.09;
l       = 0.607;
X_Ih    = 0.147;

c_zeta  = 0.0751;
c_beta  = 0.0381;
cd0     = 0.06;

a_lift  = 6.28;
a       = 6.28;

theta0  = deg2rad(9);
sigmma  = 0.0746;

rho     = 1.225;
c       = 5.9e-3;

Omega   = 200;
R       = 0.05;

m       = 0.39e-3;
theta_zeta =0.1 ;


V       = 20;
Rohm    = 1.4;
Ke      = 0.0025;
Nb      = 2;

Ibeta   = 1/3*m*R^2*(1-e)^2;
gamma   = 1.56;
k_drive = 9e-5;
c_drive = 9e-2;

% %% ---------------- AERODYNAMIC INTEGRALS ----------------
% e_int = 0.09;
% r = e:0.01:1;
% 
% iflow = a_lift*sigmma./(16*r) .* ...
%         (sqrt(1 + 32*theta*r./(a_lift*sigmma)) - 1);
% 
% prefZ = (rho*a_lift*c*Omega^2*R^3)/2;
% prefY = -(rho*a*c*Omega^2*R^3)/2;
% 
% Fy = cell(5,1);
% Fz = cell(5,1);
% 
% Fz{1} = -prefZ*(2*r.^2*theta - (1+cd0/a).*r.*iflow);
% Fz{2} =  prefZ*(2*theta.*r.*(r-e_int) - (1+cd0/a).*(r-e_int).*iflow);
% Fz{3} =  prefZ*((1+cd0/a).*r.*(r-e_int));
% Fz{4} = -prefZ*(r.^2);
% Fz{5} = -prefZ*((1+cd0/a).*r.*iflow - theta*r.^2);
% 
% Fy{1} = -prefY*(2*(cd0/a).*r.^2 + r.*iflow.*theta);
% Fy{2} =  prefY*(2*(cd0/a).*r.*(r-e_int) + theta*(r-e_int).*iflow);
% Fy{3} = -prefY*(theta*r.*(r-e_int) - 2*(r-e_int).*iflow);
% Fy{4} = -prefY*(r.*iflow);
% Fy{5} = -prefY*((cd0/a).*r.^2 - iflow.^2 + theta*r.*iflow);
% 
% C_aero = zeros(3);
% C_const = zeros(3,1);
% K_aero = zeros(3);
% 
% for i = 1:3
%     C_aero(1,i) =  R*trapz(r, r.*Fy{i});
%     C_aero(2,i) = -R*trapz(r,(r-e_int).*Fy{i});
%     C_aero(3,i) =  R*trapz(r,(r-e_int).*Fz{i});
% end
% 
% C_const(2) = -R*trapz(r,(r-e_int).*Fy{5});
% C_const(3) =  R*trapz(r,(r-e_int).*Fz{5});
% 
% K_aero(1,2) =  R*trapz(r, r.*Fy{4}*theta_zeta);
% K_aero(2,2) = -R*trapz(r,(r-e_int).*Fy{4})*theta_zeta;
% K_aero(3,2) =  R*trapz(r,(r-e_int).*Fz{4})*theta_zeta;
% 
% C_aero = C_aero/(Ibeta*Omega);
% K_aero = K_aero/Ibeta;
% 
% beta0 = C_const(3)/(Ibeta*Omega^2*(1 + e_int/l));
% zeta0 = C_const(2)/(Ibeta*Omega^2*(e_int/l));
% 
% %% ---------------- CONSTANT MATRICES ----------------
% e_int = R*0.09;
% 
% 
% 
% C = zeros(3);
% C(1,2) =  e_int*m*(e_int-R)*Omega*zeta0;
% C(1,3) = (1/3)*m*(e_int-R)*Omega*(-2*(e_int-R)+3*e_int)*beta0;
% C(2,1) = -C(1,2);
% C(2,3) = (2/3)*m*(e_int-R)^2*Omega*beta0;
% C(3,1) = -C(1,3);
% C(3,2) = -C(2,3);
% 
% Fmat = diag([Ibeta*c_drive*Omega/2, Ibeta*c_zeta*Omega, Ibeta*c_beta*Omega]);
% C = (C + Fmat)/Ibeta + C_aero;
% 
% K = zeros(3);
% K(1,1) = (Ibeta*k_drive*Omega^2)/2;
% K(2,2) = -(1/2)*e_int*m*(e_int-R)*Omega^2;
% K(3,3) = (1/6)*m*(e_int-R)*(2*(e_int-R)-3*e_int)*Omega^2;
% K(2,3) = (1/2)*e_int*m*(e_int-R)*beta0*zeta0*Omega^2;
% K(3,2) = K(2,3);
% K = K/Ibeta + K_aero;

u = a_lift*sigmma/(2*Ibeta*gamma)*Ke/Rohm;


phi34 = a_lift*sigmma/12*(sqrt(1+24*theta0/(a_lift*sigmma))-1);
zeta0 = (1/8)*(l/e)*(1-4*e/3)*gamma*(theta0*phi34-phi34^2+cd0/a_lift);
beta0 = (1/8)*(1/(1+e/l))*(1-4*e/3)*gamma*(theta0-phi34-cd0/a_lift*phi34);

odefun = @(t,x) odefun_safe(t,x,Omega);

x0 = [0,beta0,zeta0,0,0,0];

opts = odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',2*pi/Omega/50);

[t,x] = ode45(odefun,[0 40],x0,opts);

psi = Omega*t + x(:,1);     
psi_deg = psi*180/pi;

x1dot = x(:,4);
zeta  = x(:,2);
beta  = x(:,3);
u_t   = u*V*sin(Omega*t);   

x1_from_dot = cumtrapz(t, x1dot);


err = x(:,1) - x1_from_dot;
max_abs_err = max(abs(err));
rms_err = sqrt(mean(err.^2));

fprintf('Virtual-angle consistency check:\n');
fprintf('   max absolute error between x(:,1) and integral(x1dot): %.4e\n', max_abs_err);
fprintf('   RMS error: %.4e\n', rms_err);


rev = 360;
idx_tr = psi_deg <= 30*rev;
idx_ss = psi_deg >= psi_deg(end)-5*rev;
idx_1r = psi_deg >= psi_deg(end)-rev;

figure;
subplot(4,1,1); plot(psi_deg(idx_tr),x1dot(idx_tr)); grid on; ylabel('x_1 dot')
subplot(4,1,2); plot(psi_deg(idx_tr),zeta(idx_tr));  grid on; ylabel('\zeta')
subplot(4,1,3); plot(psi_deg(idx_tr),beta(idx_tr));  grid on; ylabel('\beta')
subplot(4,1,4); plot(psi_deg(idx_tr),u_t(idx_tr));   grid on; ylabel('u'); xlabel('\psi (deg)')
sgtitle('Transient (~5 revolutions)')

figure;
subplot(4,1,1); plot(psi_deg(idx_ss),x1dot(idx_ss)); grid on; ylabel('x_1 dot')
subplot(4,1,2); plot(psi_deg(idx_ss),zeta(idx_ss));  grid on; ylabel('\zeta')
subplot(4,1,3); plot(psi_deg(idx_ss),beta(idx_ss));  grid on; ylabel('\beta')
subplot(4,1,4); plot(psi_deg(idx_ss),u_t(idx_ss));   grid on; ylabel('u'); xlabel('\psi (deg)')
sgtitle('Steady (~5 revolutions)')

figure;
psi1 = psi_deg(idx_1r) - psi_deg(find(idx_1r,1));
subplot(4,1,1); plot(psi1,x1dot(idx_1r)); grid on; ylabel('x_1 dot')
subplot(4,1,2); plot(psi1,zeta(idx_1r));  grid on; ylabel('\zeta')
subplot(4,1,3); plot(psi1,beta(idx_1r));  grid on; ylabel('\beta')
subplot(4,1,4); plot(psi1,u_t(idx_1r));   grid on; ylabel('u'); xlabel('Azimuth (deg)')
sgtitle('One steady revolution')


figure;
subplot(3,1,1)
plot(t, x(:,1), 'k', 'LineWidth', 1.5); hold on
plot(t, x1_from_dot, '--r', 'LineWidth', 1.2);
grid on
xlabel('Time (s)')
ylabel('x_1')
legend('x_1 from ODE state','x_1 = \int x_1 dot dt','Location','Best')
title('Consistency check: virtual angle x_1')



subplot(3,1,2)
plot(t, beta); grid on
xlabel('Time (s)')
ylabel('error (beta')
title(sprintf('beeta', max_abs_err, rms_err))

subplot(3,1,3)
plot(t,zeta); grid on
xlabel('Time (s)')
ylabel('error (zeta')
title(sprintf('zeta', max_abs_err, rms_err))



theta_max_deg = cummax(abs(zeta));

figure;
plot(t, theta_max_deg, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('max | \theta | (deg)');
title('Running maximum of pitch angle \theta vs time');



function dx = odefun_safe(t,x,Omega)

    a_lift  = 6.28;
    a       = 6.28;
    cd0 = 0.06; 
    sigmma  = 0.0746;
    l=0.607;
    rho     = 1.225;
    c       = 5.9e-3;    
    Omega   = 200;
    R       = 0.05;
    m       = 0.39e-3;
    theta_zeta = 0.25 ;
    c_zeta  = 0.0751;
    c_beta  = 0.0381;    
    V       = 0.25;
    Rohm    = 1.4;
    Ke      = 0.0025;
    Nb      = 2;
    e=0.09;
    Ibeta   = 1/3*m*R^2*(1-e)^2;
    gamma   = 1.56*1.8e-7/Ibeta;
    k_drive = 9e-5*1.8e-7/Ibeta;
    c_drive = 9e-2*1.8e-7/Ibeta;
    e_int=0.09*R;
    e=e_int;
    x1d=x(4);
    x1=x(1);
    zetad=x(5);
    betad=x(6);
    zeta = x(2);
    beta=x(3);
    
    

    u = a_lift*sigmma/(2*Ibeta*gamma)*Ke/Rohm;
theta0  = deg2rad(9);

theta  = theta0+theta_zeta*zeta;


    %% ---------------- AERODYNAMIC INTEGRALS ----------------
    e_int = 0.09;
    r = e_int:0.01:1;
    
    iflow = a_lift*sigmma./(16*r) .* ...
            (sqrt(1 + 32*theta*r./(a_lift*sigmma)) - 1);
    
    prefZ = (rho*a_lift*c*Omega^2*R^3)/2;
    prefY = -(rho*a*c*Omega^2*R^3)/2;
    
    Fy = cell(5,1);
    Fz = cell(5,1);
    
    Fz{1} = -prefZ*(2*r.^2*theta - (1+cd0/a).*r.*iflow);
    Fz{2} =  prefZ*(2*theta.*r.*(r-e_int) - (1+cd0/a).*(r-e_int).*iflow);
    Fz{3} =  prefZ*((1+cd0/a).*r.*(r-e_int));
    Fz{4} = -prefZ*(r.^2);
    Fz{5} = -prefZ*((1+cd0/a).*r.*iflow - (theta0)*r.^2);
    
    Fy{1} = -prefY*(2*(cd0/a).*r.^2 + r.*iflow.*theta);
    Fy{2} =  prefY*(2*(cd0/a).*r.*(r-e_int) + theta*(r-e_int).*iflow);
    Fy{3} = -prefY*(theta*r.*(r-e_int) - 2*(r-e_int).*iflow);
    Fy{4} = -prefY*(r.*iflow);
    Fy{5} = -prefY*((cd0/a).*r.^2 - iflow.^2 + (theta0)*r.*iflow);
    
    C_aero = zeros(3);
    C_const = zeros(3,1);
    K_aero = zeros(3);
    
    for i = 1:3
        C_aero(1,i) =  R*trapz(r, r.*Fy{i});
        C_aero(2,i) = -R*trapz(r,(r-e_int).*Fy{i});
        C_aero(3,i) =  R*trapz(r,(r-e_int).*Fz{i});
    end
   
 
    C_const(2) = -R*trapz(r,(r-e_int).*Fy{5});
    C_const(3) =  R*trapz(r,(r-e_int).*Fz{5});
    
    K_aero(1,2) =  R*trapz(r, r.*Fy{4}*theta_zeta);
    K_aero(2,2) = -R*trapz(r,(r-e_int).*Fy{4})*theta_zeta;
    K_aero(3,2) =  R*trapz(r,(r-e_int).*Fz{4})*theta_zeta;

    C_aero = C_aero/(Ibeta*Omega);
    K_aero = K_aero/Ibeta;
    
 
    Const=zeros(3,1);
    Const(2,1)=C_const(2)+(m/6)*(e-R)*cos(beta)*( -3*e*sin(zeta)*Omega);
    Const(3,1)=C_const(3)+(1/6)*m*(e - R)*(e - R).*sin(2*beta)*Omega^2;
    Const= Const/Ibeta/Omega^2;
    disp(Const)

    %% ---------------- CONSTANT MATRICES ----------------
    e_int = R*0.09;

     e = e_int;

    C = zeros(3);
    C(1,2) =  m*e*(e-R)*cos(beta)*sin(zeta)*(Omega+x1d);
    C(1,3) = m/3*(e-R)*(-2*(e-R)*cos(beta)+3*e*cos(zeta))*sin(beta)*(Omega+x1d-zetad);
   
    C(2,1) = (m/6)*(e-R)*cos(beta)* -3*e*sin(zeta)*(x1d+2*Omega);
    C(2,3) = (m/6)*(e-R)*cos(beta)*4*(e-R)*sin(beta)*(Omega-zetad+1);
    C(3,1) = -(1/6)*m*(e - R)*(3*e*(  cos(beta).*sin(zeta).*betad ...
         + cos(zeta).*sin(beta).*(-Omega + zetad) ...
            ))-(1/6)*m*(e - R)*3*e*(Omega + x1d)*(-cos(zeta)*sin(beta));
    C(3,2) =  -(1/6)*m*(e - R)*(-3*e*cos(zeta).*sin(beta).*(Omega + x1d))-(1/6)*m*(e - R)*3*e*(Omega + x1d)*cos(beta).*sin(zeta);
    C(3,3)= -(1/6)*m*(e - R)*(-3*e*cos(beta).*sin(zeta).*(Omega + x1d))-(1/6)*m*(e - R)*3*e*(Omega + x1d)*cos(zeta).*sin(beta);

    Fmat = diag([Ibeta*c_drive*Omega/2, Ibeta*c_zeta*Omega, Ibeta*c_beta*Omega]);
    C = (C + Fmat)/Ibeta + C_aero;
    C = C/Omega^2;

    K = zeros(3);
    K(1,1) = (Ibeta*k_drive*Omega^2)/2;
    K(2,2) = -(1/2)*e_int*m*(e_int-R)*Omega^2;
    K(3,3) = (1/6)*m*(e_int-R)*(2*(e_int-R)-3*e_int)*Omega^2;
    K(2,3) = 0;
    K(3,1)= 0;
    K(3,2) = 0;

  
    K = K/Ibeta + K_aero ;
    K=K/Omega^2;
   

    cb = cos(beta);
    sb = sin(beta);
    cz = cos(zeta);
    sz = sin(zeta);

   M11 = (6*e_int^2) +( 2*e_int^2*cb^2-4*e_int*R*cb^2)+ (2*R^2)*cb^2 - (6*e_int^2)*cz*cb + (6*e_int*R)*cb*cz+( 2.6*(10^-8)*6/m) ;
    
    
    
    M12 = -2*e^2*cb^2 ...
        + 4*e*R*cb^2 ...
        - 2*R^2*cb^2 ...
        + 3*e^2*cb*cz ...
        - 3*e*R*cb*cz;
    
    M13 = -3*e^2*sb*sz ...
        + 3*e*R*sb*sz;
    
    M21 = (e - R)*cb*(-2*e*cb + 2*R*cb + 3*e*cz);
    
    M22 = 2*(e - R)*cb*(e*cb-R*cb);
    
    M31 = -3*e*sb*sz*(e-R);
    
    M33 = 2*(e - R)^2;
    
    M = (m/6).*[
        M11  M12  M13;
        M21  M22   0 ;
        M31   0   M33
    ];
    
    
    
    M=M/Ibeta/Omega^2;
    Minv = M\eye(3);
    Bv = Minv*[gamma/(a_lift*sigmma);0;0]*u*V/Omega^2;


    MinvK = Minv * K;
    MinvC = Minv * C;
    MinvCon = Minv*Const;
  
    dx = zeros(6,1);
    dx(1:3) = x(4:6);
    dx(4:6) = -MinvK*x(1:3) - MinvC*x(4:6)-MinvCon + Bv*sin(Omega*t);
end