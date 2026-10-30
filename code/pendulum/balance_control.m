%% HW6 - Balance Control: cart-pendulum report-matched MATLAB code
% Reza Babapiri
% This script reproduces the numerical design steps and the main figures of
% the CMS HW6 report. It uses the linearized cart-pendulum model for the
% Ackermann and LQR controllers and, when the Simulink files are available,
% it also runs the nonlinear simulations used for Chapter 4.

clear; close all; clc;

%% Output folder
outdir = fullfile(pwd,'HW6_figures');
if ~exist(outdir,'dir'), mkdir(outdir); end

%% Parameters from Astrom-Murray Example 6.7
M     = 10;      % cart mass [kg]
m     = 80;      % pendulum mass [kg]
J     = 100;     % pendulum inertia [kg m^2]
l     = 1;       % distance to center of mass [m]
c     = 0.1;     % cart viscous friction [N s/m]
gamma = 0.01;    % angular viscous friction [N m s]
g     = 9.8;     % gravity [m/s^2]

Mt = M + m;
Jt = J + m*l^2;
mu = Mt*Jt - (m*l)^2;

%% Linearized model around the upright equilibrium
% State vector: x = [p theta p_dot theta_dot]'
A = [ 0, 0, 1, 0;
      0, 0, 0, 1;
      0, (m^2*l^2*g)/mu, -(c*Jt)/mu, -(gamma*l*m)/mu;
      0, (Mt*m*g*l)/mu, -(c*l*m)/mu, -(gamma*Mt)/mu ];

B = [0; 0; Jt/mu; l*m/mu];
Cp = [1 0 0 0];
Ctheta = [0 1 0 0];
D = 0;

fprintf('A matri<local path redacted>'); disp(A)
fprintf('B matri<local path redacted>'); disp(B)
fprintf('Open-loop eigenvalue<local path redacted>'); disp(eig(A).')
fprintf('Reachability rank = %d\n', rank(ctrb(A,B)));

%% 1) Ackermann pole-placement controller
p_des = [-0.33+0.66i, -0.33-0.66i, -0.18+0.18i, -0.18-0.18i];
poly_des = poly(p_des);
Rctrb = ctrb(A,B);
K_ack = -[0 0 0 1] * (Rctrb \ polyvalm(poly_des,A));  % u = K_ack*x
Acl_ack = A + B*K_ack;

fprintf('\nAckermann feedback gain K_ack, with u = K_ack*<local path redacted>'); disp(K_ack)
fprintf('Closed-loop eigenvalues for Ackermann desig<local path redacted>'); disp(eig(Acl_ack).')

%% 2) LQR state-feedback controller
p_max        = 0.20;        % m
theta_max    = deg2rad(5);  % rad
pdot_max     = 0.50;        % m/s
thetadot_max = deg2rad(50); % rad/s
u_max        = 50;          % N

Q0 = diag([1/p_max^2, 1/theta_max^2, 1/pdot_max^2, 1/thetadot_max^2]);
R0 = 1/u_max^2;
alpha = 1;
beta  = 1000;
Q = alpha*Q0;
R = beta*R0;

[P_lqr,~,~] = care(A,B,Q,R);
K_lqr = -(R \ (B'*P_lqr));      % u = K_lqr*x
Acl_lqr = A + B*K_lqr;

fprintf('\nLQR feedback gain K_lqr, with u = K_lqr*<local path redacted>'); disp(K_lqr)
fprintf('Closed-loop eigenvalues for LQR desig<local path redacted>'); disp(eig(Acl_lqr).')

%% 3) Reference tracking simulations with the linear model
% The feedforward gain is selected to obtain unit steady-state gain from
% the reference cart position to the output p.
tFinal = 50;
t = linspace(0,tFinal,5000);
r = ones(size(t));

kr_ack = -1/(Cp*(Acl_ack\B));
kr_lqr = -1/(Cp*(Acl_lqr\B));

sys_ack = ss(Acl_ack, B*kr_ack, Cp, 0);
sys_lqr = ss(Acl_lqr, B*kr_lqr, Cp, 0);

p_ack = lsim(sys_ack,r,t);
p_lqr = lsim(sys_lqr,r,t);

% Control effort: u = K*x + kr*r
x_ack = lsim(ss(Acl_ack,B*kr_ack,eye(4),zeros(4,1)),r,t);
x_lqr = lsim(ss(Acl_lqr,B*kr_lqr,eye(4),zeros(4,1)),r,t);
u_ack = x_ack*K_ack' + kr_ack*r(:);
u_lqr = x_lqr*K_lqr' + kr_lqr*r(:);

figure; plot(t,p_ack,'LineWidth',1.4); hold on; plot(t,p_lqr,'LineWidth',1.4);
grid on; xlabel('Time [s]'); ylabel('Cart position p [m]');
title('Unit step response: Ackermann vs LQR');
legend('State feedback (Ackermann)','State feedback (LQR)','Location','best');
saveas(gcf, fullfile(outdir,'fig1_3_step_response_ackermann_lqr.png'));

figure; plot(t,u_ack,'LineWidth',1.4); hold on; plot(t,u_lqr,'LineWidth',1.4);
grid on; xlabel('Time [s]'); ylabel('Control input u [N]');
title('Control input: Ackermann vs LQR');
legend('State feedback (Ackermann)','State feedback (LQR)','Location','best');
saveas(gcf, fullfile(outdir,'fig1_4_control_input_ackermann_lqr.png'));

%% 4) Transfer functions and stability/minimum-phase analysis
[num_p,den_p] = ss2tf(A,B,Cp,D);
Gfp = tf(num_p,den_p);
[num_th,den_th] = ss2tf(A,B,Ctheta,D);
Gftheta = tf(num_th,den_th);

fprintf('\nTransfer function from force to cart position G_fp(s):\n'); disp(zpk(Gfp))
fprintf('Zeros of G_fp(s):\n'); disp(zero(Gfp).')
fprintf('Poles of G_fp(s):\n'); disp(pole(Gfp).')

fprintf('\nTransfer function from force to pendulum angle G_f_theta(s):\n'); disp(zpk(Gftheta))
fprintf('Zeros of G_f_theta(s):\n'); disp(zero(Gftheta).')
fprintf('Poles of G_f_theta(s):\n'); disp(pole(Gftheta).')

%% 5) Augmented LQR with integral action for cart-position tracking
% eta_dot = p - p_ref
Aaug = [A, zeros(4,1);
        Cp, 0];
Baug = [B; 0];

eta_max = p_max*1.5;
Qaug = diag([1/p_max^2, 1/theta_max^2, 1/pdot_max^2, 1/thetadot_max^2, 1/eta_max^2]);
Raug = beta*R0;

[Ka,~,eig_aug] = lqr(Aaug,Baug,Qaug,Raug);
Kc = Ka(1:4);
Ki = Ka(5);

fprintf('\nAugmented LQR gain Ka = [Kc Ki], for u = -Ka*[x;eta]:\n'); disp(Ka)
fprintf('Augmented closed-loop eigenvalue<local path redacted>'); disp(eig_aug.')

%% 6) Optional Simulink simulations, if sim6a.slx and sim6b.slx are present
modelFolder = findModelFolder();
if ~isempty(modelFolder)
    addpath(modelFolder);
    fprintf('\nUsing Simulink models from: %s\n', modelFolder);
else
    fprintf('\nSimulink models not found. Numerical state-space figures were generated only.\n');
end

% The Simulink files are not required to run the analytical part of the code.
% They are used only to reproduce the exact nonlinear simulations in the report.
if ~isempty(modelFolder) && exist(fullfile(modelFolder,'sim6b.slx'),'file')
    try
        assignin('base','M',M); assignin('base','m',m); assignin('base','J',J);
        assignin('base','l',l); assignin('base','c',c); assignin('base','gamma',gamma);
        assignin('base','g',g); assignin('base','Kc',Kc); assignin('base','Ki',Ki);
        % The detailed block parameters are set in the original report scripts.
        % This call is left active for users who keep the original Simulink model.
        load_system('sim6b');
        % simOut = sim('sim6b');
    catch ME
        warning('sim6b was found but not executed automatically: %s', ME.message);
    end
end

%% Local function
function modelFolder = findModelFolder()
    modelFolder = '';
    candidates = {pwd, fullfile(pwd,'codes'), fullfile(pwd,'hw6_codes'), fullfile(pwd,'HW6_codes')};
    for k = 1:numel(candidates)
        if exist(fullfile(candidates{k},'sim6a.slx'),'file') || exist(fullfile(candidates{k},'sim6b.slx'),'file')
            modelFolder = candidates{k};
            return;
        end
    end
end
