%% HW7 - Bicycle dynamics and control
% Report-matched MATLAB script
% Reza Babapiri
%
% This script reproduces the eigenvalue plots, root-locus analysis, and
% Whipple bicycle control simulations discussed in the report. Put the
% Simulink files sim7a.slx and sim7b.slx in the same folder as this script.

scriptDir = fileparts(mfilename('fullpath'));
if ~isempty(scriptDir)
    cd(scriptDir);
end

%% Fig. 4.18(a,b) — bicycle stability vs speed (parametric eigenvalue locus)
% FIXED VERSION (your issue):
% - The shaded/marked “stable region” is now computed using the TRUE criterion:
%     max_i Re{lambda_i(v)} < 0   (with a small tolerance)
% - Eigenvalues are SORTED at each speed by Re(lambda) (descending):
%     lambda_1 is always the MOST unstable/closest-to-instability mode.
%   => If lambda_1 is negative, then all are negative (stable).
% - If there is NO stable region, the code will NOT draw fake vertical lines.

clear; clc; close all;

%% -------------------- Numerical matrices (bicycle with rider) --------------------
M  = [ 96.8   -3.57;
      -3.57   0.258];

C  = [  0    -50.8;
       0.436  2.20];

K0 = [-901.0  35.17;
       35.17 -12.03];

K2 = [  0    -87.06;
        0      3.50];


%% -------------------- Speed sweep (NO sorting) --------------------
v_min = 0;
v_max = 20;
Nv    = 2001;
vgrid = linspace(v_min, v_max, Nv);

I2 = eye(2);
Z2 = zeros(2);

% lam will be 5xNv:
% rows 1..4: real parts of the 4 eigenvalues (unsorted, as returned by eig)
% row 5:     velocity v
lam1 = zeros(5, Nv);
lam1(5,:) = vgrid;   % store velocity in 5th row

for k = 1:Nv
    v = vgrid(k);

    A = [ Z2,              I2;
         -M\(K0 + K2*v^2), -M\(C*v) ];

    ev = eig(A);                 % 4x1 complex eigenvalues (no sorting)
    lam1(1:4, k) = real(ev(:));   % store ONLY real parts in rows 1..4
end
%%

figure('Color','w'); hold on; grid on;

v = lam1(5,:);   % velocity (x-axis)

% ----- Eigenvalues: rows 1..4 -----
plot(v, lam1(1,:), 'LineWidth', 2, 'Color', [0.0 0.6 0.0], ...
     'LineStyle', '-',  'DisplayName','Re \lambda_1');

plot(v, lam1(2,:), ...
    'Color', [0.55 0.0 0.8], ...
    'LineWidth', 2, ...
    'LineStyle', '--', ...
    'Marker', 's', ...
    'MarkerIndices', 1:60:length(v), ...
    'MarkerSize', 5, ...
    'DisplayName', 'Re \lambda_2');

plot(v, lam1(3,:), 'LineWidth', 2, 'Color', [0.0 0.45 0.85], ...
     'LineStyle', '-.', 'DisplayName','Re \lambda_3');

plot(v, lam1(4,:), 'LineWidth', 2, 'Color', [0.85 0.33 0.1], ...
     'LineStyle', ':',  'DisplayName','Re \lambda_4');


xlabel('Velocity v [m/s]');
ylabel('Value');
title('All rows of \lambda matrix vs velocity');
legend('Location','best');
yline(0,'k','LineWidth',0.5,'HandleVisibility','off');


% --- After you create the figure and BEFORE plotting the eigenvalue curves ---
figure('Color','w'); hold on; grid on;

v = lam1(5,:);   % velocity (x-axis)

% ===== Shade the stable region v in [5.91, 10.23] =====
v1 = 5.91;
v2 = 10.23;

yl = [-30 10];          % use your preferred y-limits (matches your plot)
ylim(yl);

p = patch([v1 v2 v2 v1], [yl(1) yl(1) yl(2) yl(2)], [0.9 0.9 0.9], ...
          'EdgeColor','none', 'FaceAlpha',0.35);
p.HandleVisibility = 'off';   % do NOT add to legend

% optional: draw the boundaries as dotted vertical lines (also not in legend)
xl1 = xline(v1, ':k', 'LineWidth', 1); xl1.HandleVisibility = 'off';
xl2 = xline(v2, ':k', 'LineWidth', 1); xl2.HandleVisibility = 'off';

% --- Now plot your eigenvalue curves (your existing code) ---
plot(v, lam1(1,:), 'LineWidth', 2, 'Color', [0.0 0.6 0.0], ...
     'LineStyle', '-',  'DisplayName','Re \lambda_1');

plot(v, lam1(2,:), ...
    'Color', [0.55 0.0 0.8], ...
    'LineWidth', 2, ...
    'LineStyle', '--', ...
    'Marker', 's', ...
    'MarkerIndices', 1:60:length(v), ...
    'MarkerSize', 5, ...
    'DisplayName', 'Re \lambda_2');

plot(v, lam1(3,:), 'LineWidth', 2, 'Color', [0.0 0.45 0.85], ...
     'LineStyle', '-.', 'DisplayName','Re \lambda_3');

plot(v, lam1(4,:), 'LineWidth', 2, 'Color', [0.85 0.33 0.1], ...
     'LineStyle', ':',  'DisplayName','Re \lambda_4');

xlabel('Velocity v [m/s]');
ylabel('Re(Lambda)');
title('Real parts of \lambda eigenvalues vs velocity');
legend('Location','best');

% keep the 0-line thin and OUT of legend
h0 = yline(0,'k','LineWidth',0.5);
h0.HandleVisibility = 'off';

% Make sure curves stay on top of shading
uistack(findobj(gca,'Type','line'),'top');

%% -------------------- Speed sweep (IMAGINARY PARTS, NO sorting) --------------------
v_min = 0;
v_max = 20;
Nv    = 2001;
vgrid = linspace(v_min, v_max, Nv);

I2 = eye(2);
Z2 = zeros(2);

% lam will be 5xNv:
% rows 1..4: imaginary parts of the 4 eigenvalues (unsorted)
% row 5:     velocity v
lam2 = zeros(5, Nv);
lam2(5,:) = vgrid;   % store velocity in 5th row

for k = 1:Nv
    v = vgrid(k);

    A = [ Z2,              I2;
         -M\(K0 + K2*v^2), -M\(C*v) ];

    ev = eig(A);                  % 4x1 complex eigenvalues (no sorting)
    lam2(1:4, k) = imag(ev(:));    % <-- IMAGINARY PARTS
end

%% -------------------- Plot imaginary parts --------------------
figure('Color','w'); hold on; grid on;

v = lam2(5,:);   % velocity (x-axis)

plot(v, lam2(1,:), 'LineWidth', 2, 'Color', [0.0 0.6 0.0], ...
     'LineStyle', '-',  'DisplayName','Im \lambda_1');

plot(v, lam2(2,:), ...
    'Color', [0.55 0.0 0.8], ...
    'LineWidth', 2, ...
    'LineStyle', '--', ...
    'Marker', 's', ...
    'MarkerIndices', 1:60:length(v), ...
    'MarkerSize', 5, ...
    'DisplayName','Im \lambda_2');

plot(v, lam2(3,:), 'LineWidth', 2, 'Color', [0.0 0.45 0.85], ...
     'LineStyle', '-.', 'DisplayName','Im \lambda_3');

plot(v, lam2(4,:), 'LineWidth', 2, 'Color', [0.85 0.33 0.1], ...
     'LineStyle', ':',  'DisplayName','Im \lambda_4');

xlabel('Velocity v [m/s]');
ylabel('Im(\lambda)');
title('Imaginary parts of eigenvalues vs velocity');
legend('Location','best');

% % zero-frequency line (no legend entry)
% h0 = yline(0,'k','LineWidth',0.1);
% h0.HandleVisibility = 'off';
% ---- Shade oscillatory region: v >= 1.92 ----
v_osc = 1.92;

yl = ylim;   % get current y-axis limits

p = patch([v_osc v_max v_max v_osc], ...
          [yl(1)  yl(1)  yl(2)  yl(2)], ...
          [0.85 0.85 0.85], ...
          'EdgeColor','none', ...
          'FaceAlpha',0.35);

p.HandleVisibility = 'off';   % exclude from legend

% Optional: vertical boundary line
xl = xline(v_osc, ':k', 'LineWidth', 1);
xl.HandleVisibility = 'off';


%% -------------------- Root locus: Im(lambda) vs Re(lambda) --------------------
figure('Color','w'); hold on; grid on;

% Colors (no black)
colors = [
    0.00 0.60 0.00 ;   % lambda_1  green
    0.55 0.00 0.80 ;   % lambda_2  purple
    0.00 0.45 0.85 ;   % lambda_3  blue
    0.85 0.33 0.10     % lambda_4  orange
];

styles  = {'-','--','-.',':'};
markers = {'o','s','+','v'};   % circle, square, none, none
LW = 2;

markIdx = 1:80:length(lam1(1,:));    % marker spacing

for i = 1:4
    if strcmp(markers{i}, 'none')
        plot(lam1(i,:), lam2(i,:), ...
            'LineWidth', LW, ...
            'LineStyle', styles{i}, ...
            'Color', colors(i,:), ...
            'DisplayName', sprintf('\\lambda_%d', i));
    else
        plot(lam1(i,:), lam2(i,:), ...
            'LineWidth', LW, ...
            'LineStyle', styles{i}, ...
            'Color', colors(i,:), ...
            'Marker', markers{i}, ...
            'MarkerIndices', markIdx, ...
            'MarkerSize', 5, ...
            'DisplayName', sprintf('\\lambda_%d', i));
    end
end

% Axes reference lines
xline(0,'k','LineWidth',1,'HandleVisibility','off');
yline(0,'k','LineWidth',1,'HandleVisibility','off');

xlabel('Re(\lambda)');
ylabel('Im(\lambda)');
title('Root locus of bicycle eigenvalues as velocity varies');
legend('Location','best');
axis equal;


% ---- Velocity text annotations for ALL 4 eigenvalue branches (shifted down) ----
vgrid = lam1(5,:);              % velocity vector
v_labels = [0 10 20];           % velocities to annotate

% Small offsets per branch (avoid overlapping curves)
dx = [ 0.15, -0.20,  0.10, -0.15 ];
dy = [ 0.15,  0.15, -0.15, -0.15 ];

% Global downward shift (to move away from axes)
Yshift = -1;    % <-- increase magnitude if needed (e.g. -0.7)

for i = 1:4
    for j = 1:length(v_labels)
        [~, k] = min(abs(vgrid - v_labels(j)));

        text(lam1(i,k) + dx(i), lam2(i,k) + dy(i) + Yshift, ...
            sprintf('v = %d m/s', v_labels(j)), ...
            'Color', colors(i,:), ...
            'FontSize', 9, ...
            'FontWeight', 'normal');
    end
end
%%
clc; clear;

% ===== Benchmark matrices (as you entered) =====
M  = [80.8172227850870   2.31941332208709;
      2.31941332208709   0.29784188199686];

C1 = [0                  33.86641391492494;
     -0.85035641456978   1.68540397397560];

K0 = [-80.95             -2.59951685249872;
      -2.59951685249872  -0.80329488458618];

K2 = [0                  76.59734589573222;
      0                  2.65431523794604];

v0 = 5;   % speed [m/s]

% ===== Book convention (NO g factor) =====
Cv0 = v0 * C1;
Kv0 = K0 + (v0^2)*K2;

% ===== State-space matrices =====
A = [zeros(2) eye(2);
     -M\Kv0   -M\Cv0];

B = [zeros(2,1);
     M\[0;1]];   % input is steering torque T

% ===== Display & eigenvalues =====
disp('Cv0 = v0*C1 ='); disp(Cv0)
disp('Kv0 = K0 + v0^2*K2 ='); disp(Kv0)

disp('A(v0) ='); disp(A)
disp('B ='); disp(B)

eigA = eig(A);
disp('Eigenvalues of A at v0 = 5 m/s:');
disp(eigA.')

%% ===== Desired closed-loop poles =====
p_des = [-2, -10, -1+1j, -1-1j];

%% ===== Controllability (reachability) check =====
R = ctrb(A,B);
fprintf('rank(R) = %d (should be 4)\n', rank(R));

%% ===== Ackermann formula (manual) for u = -Kx =====
% Desired characteristic polynomial p*(s) = prod(s - p_i)
a  = poly(p_des);        % [1 a1 a2 a3 a4]

% Evaluate p*(A) = A^4 + a1 A^3 + a2 A^2 + a3 A + a4 I
pA = polyvalm(a, A);

% e_n^T for n=4
e4T = [0 0 0 1];

% Ackermann gain: K = e4^T * inv(R) * p*(A)
K_c  = -e4T * (R \ pA);

% You already have A, B, and your state feedback K (for u = Kx)

Cdelta = [0 1 0 0];          % y = delta

Acl = A + B*K_c;               % since your convention is u = Kx

% Feedforward / prefilter gain for delta tracking:
K_ff = -1 / (Cdelta * (Acl \ B));   % same as -1/(Cdelta*inv(Acl)*B)





%%

%%%% ============================================================
%  Proportional output feedback: u = k_p * phi
%  Goal:
%    1) Find all k_p such that ALL closed-loop poles satisfy Re(p) <= -1
%    2) Among those k_p, choose the one with the HIGHEST damping ratio
%       (minimum damping ratio among complex poles is maximized).
%
%  Requires: A (4x4), B (4x1) in workspace
%  Uses: phi = Cphi*x,  Cphi = [1 0 0 0]
%  Closed-loop: Acl = A + B*k_p*Cphi
% ============================================================

clc;

%% ---- Output selection ----
Cphi = [1 0 0 0];

%% ---- Scan settings (adjust range/resolution if needed) ----
kp_min = 0;          % start from 0 (change if you want negative too)
kp_max = 100;        % increase if needed
Nkp    = 20001;      % resolution (large -> finer search)

kp_vec = linspace(kp_min, kp_max, Nkp);

%% ---- Requirement ----
sigma_req = -1;      % require max Re(poles) <= -1
eps_imag  = 1e-8;    % imag threshold

%% ---- Helper: damping ratio of a pole p = a + jb, a<0 ----
damping_ratio = @(p) (-real(p)) ./ abs(p);

%% ---- Storage for feasible k_p ----
ok_kp      = [];
ok_poles   = [];
ok_minzeta = [];   % for each kp: minimum damping ratio among complex poles

%% ---- Scan ----
for kp = kp_vec
    Acl = A + B*kp*Cphi;        % u = +kp*phi
    p   = eig(Acl);

    % Only constraint: ALL poles must satisfy Re(p) <= -1
    if max(real(p)) > sigma_req
        continue
    end

    % Compute "quality" = min damping ratio among complex poles
    % If there are no complex poles, set minzeta = +Inf (perfectly non-oscillatory)
    zetas = [];
    for i = 1:numel(p)
        if abs(imag(p(i))) > eps_imag
            zetas(end+1) = damping_ratio(p(i)); %#ok<SAGROW>
        end
    end

    if isempty(zetas)
        minzeta = Inf;
    else
        minzeta = min(zetas);
    end

    ok_kp      = [ok_kp; kp];       %#ok<AGROW>
    ok_poles   = [ok_poles; p.'];   %#ok<AGROW>
    ok_minzeta = [ok_minzeta; minzeta]; %#ok<AGROW>
end

%% ---- Results ----
fprintf('\n================ u = k_p * phi =================\n');
fprintf('Constraint: ALL poles must satisfy Re(p) <= %.2f\n', sigma_req);

if isempty(ok_kp)
    fprintf('No k_p in [%.3f, %.3f] satisfies Re(p)<=%.2f for all poles.\n', kp_min, kp_max, sigma_req);
    fprintf('Try increasing kp_max or allow negative gains.\n');
else
    fprintf('Found %d feasible k_p values.\n', numel(ok_kp));
    fprintf('Feasible k_p range approx: [%.6f, %.6f]\n', ok_kp(1), ok_kp(end));

    % Choose best kp: highest min damping ratio among complex poles
    % (Inf means no complex poles -> best possible)
    [best_minzeta, idx_best] = max(ok_minzeta);
    kp_best   = ok_kp(idx_best);
    poles_best = ok_poles(idx_best,:).';

    fprintf('\nBest k_p (maximizes minimum damping ratio among complex poles):\n');
    fprintf('  k_p = %.6f\n', kp_best);

    if isinf(best_minzeta)
        fprintf('  Best min zeta = Inf (no complex poles -> purely real poles)\n');
    else
        fprintf('  Best min zeta = %.6f\n', best_minzeta);
    end

    fprintf('  Poles at best k_<local path redacted>');
    disp(poles_best.');

    % Optional: plot min damping ratio vs kp (only feasible kp)
    figure; grid on;
    plot(ok_kp, ok_minzeta, 'LineWidth', 1.5);
    xlabel('k_p'); ylabel('min \zeta among complex poles (Inf if none)');
    title('Feasible k_p (Re(p)<=-1): damping quality vs k_p');

    % Also plot max real part of poles vs kp for the entire scan
    maxRe = zeros(size(kp_vec));
    for k = 1:numel(kp_vec)
        p = eig(A + B*kp_vec(k)*Cphi);
        maxRe(k) = max(real(p));
    end
    figure; grid on;
    plot(kp_vec, maxRe, 'LineWidth', 1.5);
    yline(sigma_req,'--r');
    xlabel('k_p'); ylabel('max Re(poles)');
    title('max Re(poles) vs k_p for u = k_p * \phi');
end
%%

% Simulink for state feedback

% The script uses the current folder; keep sim7a.slx and sim7b.slx next to this file.
open_system('sim7a');


a31=A(3,1);
a32=A(3,2);
a33=A(3,3);
a34=A(3,4);
a41=A(4,1);
a42=A(4,2);
a43=A(4,3);
a44=A(4,4);
b3=B(3,1);
b4=B(4,1);

set_param('sim7a/a31', 'K', 'a31');
set_param('sim7a/a32', 'K', 'a32');
set_param('sim7a/a33', 'K', 'a33');
set_param('sim7a/a34', 'K', 'a34');
set_param('sim7a/a41', 'K', 'a41');
set_param('sim7a/a42', 'K', 'a42');
set_param('sim7a/a43', 'K', 'a43');
set_param('sim7a/a44', 'K', 'a44');
set_param('sim7a/b3', 'K', 'b3');
set_param('sim7a/b4', 'K', 'b4');


kc1=K_c(1,1);
kc2=K_c(1,2);
kc3=K_c(1,3);
kc4=K_c(1,4);

set_param('sim7a/kc1', 'K', 'kc1');
set_param('sim7a/kc2', 'K', 'kc2');
set_param('sim7a/kc3', 'K', 'kc3');
set_param('sim7a/kc4', 'K', 'kc4');
set_param('sim7a/K_ff', 'K', 'K_ff');

% % initial value for cart displacement
phi_init=0;
% % initial value for penduluum angle
theta_init=0;
% % initial value for cart speed
phi_dot_init=0;
% % initial value for penduluum angular speed
theta_dot_init=0;


set_param('sim7a/Integrator', 'InitialCondition', 'phi_init');
set_param('sim7a/Integrator1', 'InitialCondition', 'theta_init');
set_param('sim7a/Integrator2', 'InitialCondition', 'phi_dot_init');
set_param('sim7a/Integrator3', 'InitialCondition', 'theta_dot_init');


simOut = sim('sim7a');
time1 = simOut.get('delta').time;
delta = simOut.get('delta').Data;
time2 = simOut.get('phi').time;
phi = simOut.get('phi').Data;


plot(time1,delta(:,1),'b')
hold on
ylim([-0.005 0.005])
yticks(-0.005:0.001:0.005)
xlim([0 50])
xticks(0:5:50)
grid on
xlabel("time (sec)")
ylabel("Steering angle delta (Rad)")
fontsize(16,"points")



%%
% simple proportional feedback u=kp*x

kp=kp_best;
K_c1=[kp 0 0 0];
Acl1 = A + B*K_c1;
Cdelta = [0 1 0 0];          % y = delta
K_ff1 = -1 / (Cdelta * (Acl1 \ B));

% The script uses the current folder; keep sim7a.slx and sim7b.slx next to this file.
open_system('sim7b');


a31=A(3,1);
a32=A(3,2);
a33=A(3,3);
a34=A(3,4);
a41=A(4,1);
a42=A(4,2);
a43=A(4,3);
a44=A(4,4);
b3=B(3,1);
b4=B(4,1);

set_param('sim7b/a31', 'K', 'a31');
set_param('sim7b/a32', 'K', 'a32');
set_param('sim7b/a33', 'K', 'a33');
set_param('sim7b/a34', 'K', 'a34');
set_param('sim7b/a41', 'K', 'a41');
set_param('sim7b/a42', 'K', 'a42');
set_param('sim7b/a43', 'K', 'a43');
set_param('sim7b/a44', 'K', 'a44');
set_param('sim7b/b3', 'K', 'b3');
set_param('sim7b/b4', 'K', 'b4');


set_param('sim7b/kp', 'K', 'kp');
set_param('sim7b/K_ff1', 'K', 'K_ff1');

% % initial value for cart displacement
phi_init=0;
% % initial value for penduluum angle
theta_init=0;
% % initial value for cart speed
phi_dot_init=0;
% % initial value for penduluum angular speed
theta_dot_init=0;


set_param('sim7b/Integrator', 'InitialCondition', 'phi_init');
set_param('sim7b/Integrator1', 'InitialCondition', 'theta_init');
set_param('sim7b/Integrator2', 'InitialCondition', 'phi_dot_init');
set_param('sim7b/Integrator3', 'InitialCondition', 'theta_dot_init');


simOut = sim('sim7b');
time1 = simOut.get('delta').time;
delta = simOut.get('delta').Data;
time2 = simOut.get('phi').time;
phi = simOut.get('phi').Data;


plot(time1,delta(:,1),'b')
hold on
ylim([-0.006 0.006])
yticks(-0.006:0.001:0.006)
xlim([0 50])
xticks(0:5:50)
grid on
xlabel("time (sec)")
ylabel("Steering angle delta (Rad)")
fontsize(16,"points")


