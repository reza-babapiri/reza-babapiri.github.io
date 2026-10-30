%% HW5 Part 1 - BMW 730 steering dynamics
% Report-matched MATLAB code
% Reza Babapiri
%
% This script reproduces the main numerical results and plots used in the
% BMW 730 steering dynamics report. It follows the notation of the report:
% x1 = lateral speed, x2 = yaw rate, and the input is the steering angle.

clear; close all; clc;

%% Vehicle parameters
m  = 1864;       % vehicle mass [kg]
J  = 3654;       % yaw inertia [kg m^2]
Cr = 213800;     % rear cornering stiffness [N/rad]
Cf = 101600;     % front cornering stiffness [N/rad]
Lr = 1.32;       % rear axle distance [m]
Lf = 1.51;       % front axle distance [m]

Alpha = Cf/m + Cr/m;
Beta  = -Cf*Lf + Cr*Lr;
Gamma = (Cf*Lf^2 + Cr*Lr^2)/J;

b1 = Cf/m;
b2 = Cf*Lf/J;

C_yaw = [0 1];
C_lat = [1 0];
D = 0;

outdir = fullfile(pwd,'HW5_part1_figures');
if ~exist(outdir,'dir'); mkdir(outdir); end

%% Helper velocity grids
Vf = 0.1:0.1:245/3.6;   % forward speed [m/s]
Vb = -0.1:-0.1:-36/3.6; % backward speed [m/s]

%% Figure 1.1 - Forward speed eigenvalues
E1_f = zeros(numel(Vf),3);
E2_f = zeros(numel(Vf),3);
for k = 1:numel(Vf)
    V = Vf(k);
    [A,B] = vehicle_matrices(V,1,Alpha,Beta,Gamma,Cf,Lf,m,J);
    e = eig(A);
    E1_f(k,:) = [real(e(1)), imag(e(1)), V];
    E2_f(k,:) = [real(e(2)), imag(e(2)), V];
end

figure('Name','Figure 1.1');
plot(E1_f(:,1),E1_f(:,2),'b','LineWidth',1.3); hold on;
plot(E2_f(:,1),E2_f(:,2),'r','LineWidth',1.3); grid on;
xlabel('Real part'); ylabel('Imaginary part');
title('Forward speed eigenvalues');
legend('Eigenvalue 1','Eigenvalue 2','Location','best');
save_figure(gcf,outdir,'fig_1_1_forward_eigenvalues');

%% Figure 1.2 - Backward speed eigenvalues
E1_b = zeros(numel(Vb),3);
E2_b = zeros(numel(Vb),3);
for k = 1:numel(Vb)
    V = Vb(k);
    [A,B] = vehicle_matrices(V,1,Alpha,Beta,Gamma,Cf,Lf,m,J);
    e = eig(A);
    E1_b(k,:) = [real(e(1)), imag(e(1)), V];
    E2_b(k,:) = [real(e(2)), imag(e(2)), V];
end

figure('Name','Figure 1.2');
plot(E1_b(:,1),E1_b(:,2),'b','LineWidth',1.3); hold on;
plot(E2_b(:,1),E2_b(:,2),'r','LineWidth',1.3); grid on;
xlabel('Real part'); ylabel('Imaginary part');
title('Backward speed eigenvalues');
legend('Eigenvalue 1','Eigenvalue 2','Location','best');
save_figure(gcf,outdir,'fig_1_2_backward_eigenvalues');

%% Figures 1.3 and 1.4 - Reachability and observability determinants, forward speed
R_det_f = zeros(numel(Vf),1);
O_det_f = zeros(numel(Vf),1);
for k = 1:numel(Vf)
    V = Vf(k);
    [A,B] = vehicle_matrices(V,1,Alpha,Beta,Gamma,Cf,Lf,m,J);
    R = [B A*B];
    O = [C_yaw; C_yaw*A];
    R_det_f(k) = det(R);
    O_det_f(k) = det(O);
end

figure('Name','Figure 1.3');
plot(Vf,R_det_f,'LineWidth',1.3); grid on;
xlabel('Velocity [m/s]'); ylabel('det(R)');
title('Reachability determinant for forward speed');
save_figure(gcf,outdir,'fig_1_3_reachability_forward');

figure('Name','Figure 1.4');
plot(Vf,O_det_f,'LineWidth',1.3); grid on;
xlabel('Velocity [m/s]'); ylabel('det(O)');
title('Observability determinant for forward speed');
save_figure(gcf,outdir,'fig_1_4_observability_forward');

%% Figures 1.5 and 1.6 - Reachability and observability determinants, backward speed
R_det_b = zeros(numel(Vb),1);
O_det_b = zeros(numel(Vb),1);
for k = 1:numel(Vb)
    V = Vb(k);
    [A,B] = vehicle_matrices(V,1,Alpha,Beta,Gamma,Cf,Lf,m,J);
    R = [B A*B];
    O = [C_yaw; C_yaw*A];
    R_det_b(k) = det(R);
    O_det_b(k) = det(O);
end

figure('Name','Figure 1.5');
plot(Vb,R_det_b,'LineWidth',1.3); grid on;
xlabel('Velocity [m/s]'); ylabel('det(R)');
title('Reachability determinant for backward speed');
save_figure(gcf,outdir,'fig_1_5_reachability_backward');

figure('Name','Figure 1.6');
plot(Vb,O_det_b,'LineWidth',1.3); grid on;
xlabel('Velocity [m/s]'); ylabel('det(O)');
title('Observability determinant for backward speed');
save_figure(gcf,outdir,'fig_1_6_observability_backward');

%% Figures 1.7 and 1.8 - Zeros and minimum-phase properties
Z1_f = zeros(numel(Vf),1);
Z2_f = zeros(numel(Vf),1);
for k = 1:numel(Vf)
    V = Vf(k);
    [A,B] = vehicle_matrices(V,1,Alpha,Beta,Gamma,Cf,Lf,m,J);
    a11=A(1,1); a12=A(1,2); a21=A(2,1); a22=A(2,2);
    b1m=B(1); b2m=B(2);
    Z1_f(k) = (a11*b2m - a21*b1m)/b2m;
    Z2_f(k) = (a22*b1m - a12*b2m)/b1m;
end

figure('Name','Figure 1.7');
plot(Vf,Z1_f,'LineWidth',1.3); hold on;
plot(Vf,Z2_f,'LineWidth',1.3); grid on;
xlabel('Velocity [m/s]'); ylabel('Zero location');
title('Zeros of transfer functions for forward speed');
legend('z_1: steering to yaw rate','z_2: steering to lateral speed','Location','best');
save_figure(gcf,outdir,'fig_1_7_zeros_forward');

Z1_b = zeros(numel(Vb),1);
Z2_b = zeros(numel(Vb),1);
for k = 1:numel(Vb)
    V = Vb(k);
    [A,B] = vehicle_matrices(V,1,Alpha,Beta,Gamma,Cf,Lf,m,J);
    a11=A(1,1); a12=A(1,2); a21=A(2,1); a22=A(2,2);
    b1m=B(1); b2m=B(2);
    Z1_b(k) = (a11*b2m - a21*b1m)/b2m;
    Z2_b(k) = (a22*b1m - a12*b2m)/b1m;
end

figure('Name','Figure 1.8');
plot(Vb,Z1_b,'LineWidth',1.3); hold on;
plot(Vb,Z2_b,'LineWidth',1.3); grid on;
xlabel('Velocity [m/s]'); ylabel('Zero location');
title('Zeros of transfer functions for backward speed');
legend('z_1: steering to yaw rate','z_2: steering to lateral speed','Location','best');
save_figure(gcf,outdir,'fig_1_8_zeros_backward');

%% Figures 1.9 and 1.10 - Bode diagrams for the two transfer functions
speeds = [-10 -5 5 10];
G_yaw = cell(1,numel(speeds));
G_lat = cell(1,numel(speeds));
for k = 1:numel(speeds)
    V = speeds(k);
    G_yaw{k} = yaw_tf(V,1,Alpha,Beta,Gamma,Cf,Lf,m,J);
    G_lat{k} = lateral_tf(V,1,Alpha,Beta,Gamma,Cf,Lf,m,J);
end

figure('Name','Figure 1.9');
bodeplot(G_yaw{:}); grid on;
legend('V = -10 m/s','V = -5 m/s','V = 5 m/s','V = 10 m/s','Location','best');
title('Bode plot of steering angle to yaw rate transfer function');
save_figure(gcf,outdir,'fig_1_9_bode_yaw_rate');

figure('Name','Figure 1.10');
bodeplot(G_lat{:}); grid on;
legend('V = -10 m/s','V = -5 m/s','V = 5 m/s','V = 10 m/s','Location','best');
title('Bode plot of steering angle to lateral speed transfer function');
save_figure(gcf,outdir,'fig_1_10_bode_lateral_speed');

%% Figure 1.11 - Step response at 30 m/s with steering angle 0.05 rad
V = 30;
G1 = 0.05*yaw_tf(V,1,Alpha,Beta,Gamma,Cf,Lf,m,J);
G2 = 0.05*lateral_tf(V,1,Alpha,Beta,Gamma,Cf,Lf,m,J);
figure('Name','Figure 1.11');
step(G1); hold on; step(G2); grid on;
legend('Yaw rate','Lateral speed','Location','best');
title('Step response for steering angle of 0.05 rad at V = 30 m/s');
save_figure(gcf,outdir,'fig_1_11_step_response_30ms');

%% Figure 1.12 - Oversteering, neutral steering, understeering
Kstatic = zeros(numel(Vf),1);
for k = 1:numel(Vf)
    V = Vf(k);
    [A,B] = vehicle_matrices(V,1,Alpha,Beta,Gamma,Cf,Lf,m,J);
    a11=A(1,1); a12=A(1,2); a21=A(2,1); a22=A(2,2);
    b1m=B(1); b2m=B(2);
    Kstatic(k) = (a12*b2m - a22*b1m)/(a11*a22 - a12*a21);
end

figure('Name','Figure 1.12');
plot(Vf,Kstatic,'LineWidth',1.3); hold on;
yline(0,'k--'); grid on;
xlabel('Velocity [m/s]'); ylabel('Static gain sign');
title('Oversteering, neutral steering and understeering regions');
save_figure(gcf,outdir,'fig_1_12_steering_regions');

%% Figures 1.13, 1.14, 1.15 - Damping factor, settling time and overshoot
zeta_v = zeros(numel(Vf),1);
wn_v = zeros(numel(Vf),1);
for k = 1:numel(Vf)
    V = Vf(k);
    G = yaw_tf(V,1,Alpha,Beta,Gamma,Cf,Lf,m,J);
    [wn,zeta] = damp(G);
    zeta_v(k) = min(zeta);
    wn_v(k) = min(wn);
end

figure('Name','Figure 1.13');
plot(Vf,zeta_v,'LineWidth',1.3); grid on;
xlabel('Velocity [m/s]'); ylabel('Damping factor');
title('Damping factor with respect to forward speed');
ylim([0 1.2]);
save_figure(gcf,outdir,'fig_1_13_damping_velocity');

Vc = 8.4:0.1:245/3.6;
settling = zeros(numel(Vc),1);
overshoot = zeros(numel(Vc),1);
for k = 1:numel(Vc)
    V = Vc(k);
    G = lateral_tf(V,1,Alpha,Beta,Gamma,Cf,Lf,m,J);
    [wn,zeta] = damp(G);
    z = min(zeta); wmin = min(wn);
    settling(k) = 3/(z*wmin);
    if z < 1
        overshoot(k) = exp(-z*pi/sqrt(1-z^2))*100;
    else
        overshoot(k) = 0;
    end
end

figure('Name','Figure 1.14');
plot(Vc,settling,'LineWidth',1.3); grid on;
xlabel('Velocity [m/s]'); ylabel('Settling time [s]');
title('Settling time with respect to velocity');
save_figure(gcf,outdir,'fig_1_14_settling_time');

figure('Name','Figure 1.15');
plot(Vc,overshoot,'LineWidth',1.3); grid on;
xlabel('Velocity [m/s]'); ylabel('Overshoot [%]');
title('Overshoot with respect to velocity');
save_figure(gcf,outdir,'fig_1_15_overshoot');

%% Figure 1.16 - Effect of road friction coefficient mu
mu_grid = 0.01:0.01:1;
speeds_mu = [5 25 50];
zeta_mu = zeros(numel(mu_grid),numel(speeds_mu));
for j = 1:numel(speeds_mu)
    V = speeds_mu(j);
    for k = 1:numel(mu_grid)
        mu = mu_grid(k);
        G = lateral_tf(V,mu,Alpha,Beta,Gamma,Cf,Lf,m,J);
        [~,zeta] = damp(G);
        zeta_mu(k,j) = min(zeta);
    end
end

figure('Name','Figure 1.16');
plot(mu_grid,zeta_mu(:,1),'LineWidth',1.3); hold on;
plot(mu_grid,zeta_mu(:,2),'LineWidth',1.3);
plot(mu_grid,zeta_mu(:,3),'LineWidth',1.3); grid on;
xlabel('\mu'); ylabel('Damping factor');
title('Damping factor with respect to \mu for different speeds');
legend('V = 5 m/s','V = 25 m/s','V = 50 m/s','Location','best');
ylim([0 1.2]);
save_figure(gcf,outdir,'fig_1_16_damping_mu');

%% Figure 1.17 - 3-D damping factor with respect to velocity and mu
mu_values = 0.01:0.01:1;
V_values = 0.1:0.1:50;
[MU,VV] = meshgrid(mu_values,V_values);
ZZ = zeros(size(MU));
for i = 1:numel(MU)
    G = lateral_tf(VV(i),MU(i),Alpha,Beta,Gamma,Cf,Lf,m,J);
    [~,zeta] = damp(G);
    ZZ(i) = min(zeta);
end
figure('Name','Figure 1.17');
surf(MU,VV,ZZ,'EdgeColor','none'); grid on;
xlabel('\mu'); ylabel('Velocity [m/s]'); zlabel('Damping factor');
title('Damping factor with respect to velocity and \mu');
view(135,30); colorbar;
save_figure(gcf,outdir,'fig_1_17_damping_surface');

%% Figures 1.18 to 1.21 - Feedback design for yaw-rate output
mu = 0.001; V = 10;
Gy = yaw_tf(V,mu,Alpha,Beta,Gamma,Cf,Lf,m,J);
figure('Name','Figure 1.18');
step(Gy); grid on;
title('Yaw-rate step response for \mu = 0.001 and V = 10 m/s');
save_figure(gcf,outdir,'fig_1_18_yaw_step_low_mu');

figure('Name','Figure 1.19');
pzmap(zpk(zero(Gy),[pole(Gy);0],1)); grid on;
title('Open-loop poles and zeros for the basic yaw-rate controller');
save_figure(gcf,outdir,'fig_1_19_yaw_pzmap_basic');

figure('Name','Figure 1.20');
rlocus(zpk(zero(Gy),[pole(Gy);0],1)); grid on;
title('Root locus for yaw-rate controller C(s) = K/s');
save_figure(gcf,outdir,'fig_1_20_yaw_root_locus_basic');

z_yaw = zero(Gy);
p_yaw = pole(Gy);
G_yaw_pi = zpk([z_yaw; -0.1],[p_yaw; 0],1);
figure('Name','Figure 1.21');
rlocus(G_yaw_pi); grid on;
title('Root locus for yaw-rate PI structure');
save_figure(gcf,outdir,'fig_1_21_yaw_root_locus_pi');

%% Figures 1.22 to 1.28 - Feedback design for lateral-speed output
Gl = lateral_tf(V,mu,Alpha,Beta,Gamma,Cf,Lf,m,J);
figure('Name','Figure 1.22');
step(Gl); grid on;
title('Lateral-speed step response for \mu = 0.001 and V = 10 m/s');
save_figure(gcf,outdir,'fig_1_22_lateral_step_low_mu');

z_lat = zero(Gl);
p_lat = pole(Gl);
figure('Name','Figure 1.23');
pzmap(zpk(z_lat,[p_lat;0],1)); grid on;
title('Open-loop poles and zeros for the basic lateral-speed controller');
save_figure(gcf,outdir,'fig_1_23_lateral_pzmap_basic');

figure('Name','Figure 1.24');
rlocus(zpk(z_lat,[p_lat;0],1)); grid on;
title('Root locus for C(s) = K/s in lateral-speed control');
save_figure(gcf,outdir,'fig_1_24_lateral_root_locus_basic');

figure('Name','Figure 1.25');
rlocus(zpk([z_lat; 3],[p_lat;0],1)); grid on;
title('Root locus after adding one controller zero');
save_figure(gcf,outdir,'fig_1_25_lateral_root_locus_zero');

figure('Name','Figure 1.26');
rlocus(zpk([z_lat; -5; -10],[p_lat;0;z_lat],1)); grid on;
title('Root locus for alternative pole-zero placement');
save_figure(gcf,outdir,'fig_1_26_lateral_root_locus_alt');

figure('Name','Figure 1.27');
rlocus(zpk([z_lat;3],[p_lat;0;-0.01],1)); grid on;
title('Root locus for lateral-speed controller with pole in region 3');
save_figure(gcf,outdir,'fig_1_27_lateral_root_locus_region3');

G_lat_final = zpk([z_lat;3],[p_lat;0;-0.03],1);
figure('Name','Figure 1.28');
rlocus(G_lat_final); grid on;
title('Root locus for final lateral-speed controller');
save_figure(gcf,outdir,'fig_1_28_lateral_root_locus_final');

%% Print final controller information
fprintf('\nFinal yaw-rate PI open-loop structur<local path redacted>');
fprintf('C_yaw(s) proportional to (s + 0.1)/s\n');
fprintf('\nFinal lateral-speed controller structur<local path redacted>');
fprintf('C_lat(s) proportional to (s - %.4g)(s - 3)/(s(s + 0.03))\n', z_lat(1));
fprintf('\nAll figures saved in: %s\n', outdir);

%% Local functions
function [A,B] = vehicle_matrices(V,mu,Alpha,Beta,Gamma,Cf,Lf,m,J)
    a11 = -mu*Alpha/V;
    a12 = -(V - mu*Beta/(m*V));
    a21 =  mu*Beta/(J*V);
    a22 = -mu*Gamma/V;
    b1  =  mu*Cf/m;
    b2  =  mu*Cf*Lf/J;
    A = [a11 a12; a21 a22];
    B = [b1; b2];
end

function G = yaw_tf(V,mu,Alpha,Beta,Gamma,Cf,Lf,m,J)
    [A,B] = vehicle_matrices(V,mu,Alpha,Beta,Gamma,Cf,Lf,m,J);
    C = [0 1]; D = 0;
    G = minreal(tf(ss(A,B,C,D)),1e-8);
end

function G = lateral_tf(V,mu,Alpha,Beta,Gamma,Cf,Lf,m,J)
    [A,B] = vehicle_matrices(V,mu,Alpha,Beta,Gamma,Cf,Lf,m,J);
    C = [1 0]; D = 0;
    G = minreal(tf(ss(A,B,C,D)),1e-8);
end

function save_figure(fig,outdir,name)
    set(fig,'Color','w');
    saveas(fig,fullfile(outdir,[name '.png']));
    saveas(fig,fullfile(outdir,[name '.fig']));
end
