%% HW8 - Kalman filtering for the penalty kick
% Control of Mechanical Systems - Homework 8
% Reza Babapiri
%
% This script implements a discrete-time Kalman filter for estimating the
% 2D position of a football during a penalty kick using noisy position
% measurements. The ten simulated penalty kicks use the same initial speed
% magnitudes and shooting directions reported in the homework table.

clear; close all; clc;
rng(8);                         % fixed seed for reproducible measurements

%% Problem parameters
L = 11;                         % penalty spot to goal line [m]
W_goal = 7.32;                  % full goal width [m]
half_goal = W_goal/2;           % half goal width [m]
Ts = 0.01;                      % sampling time [s]
threshold = 0.25;               % required maximum error at the goal line [m]

%% Discrete-time constant-velocity model
% State: x = [px; py; vx; vy]
Ad = [1 0 Ts 0;
      0 1 0  Ts;
      0 0 1  0;
      0 0 0  1];

Cd = [1 0 0 0;
      0 1 0 0];

%% Initial estimate and covariance
xhat0 = [0; 0; 29.25; 0];
P0 = diag([0, 0, 8.33, 43.79]);

%% Noise covariances
% Process noise acts on the acceleration components.
Qa = diag([28.3, 28.3]);
Qd = [(Ts^4/4)*Qa, (Ts^3/2)*Qa;
      (Ts^3/2)*Qa, (Ts^2)*Qa];

% Measurement noise covariance for measured px and py.
Rd = diag([0.00694, 0.00694]);

%% Ten penalty kicks used in the report
% Columns: [initial speed magnitude (m/s), shooting direction (deg)]
INIT_VEC = [25.835, -21.330;
            27.467,  18.914;
            30.860,   3.5967;
            31.260, -16.743;
            31.609,  16.017;
            31.664,   3.5368;
            32.298,  -0.69347;
            32.386,  11.880;
            33.908,  15.229;
            34.823, -12.833];

outdir = fullfile(pwd,'HW8_generated_figures');
if ~exist(outdir,'dir')
    mkdir(outdir);
end

Nkick = size(INIT_VEC,1);
summary = zeros(Nkick,5);       % [kick, V0, theta_deg, t_reach, error_at_reach]

for kick = 1:Nkick
    %% True kick parameters
    V0 = INIT_VEC(kick,1);
    theta_deg = INIT_VEC(kick,2);
    theta = deg2rad(theta_deg);

    vx = V0*cos(theta);
    vy = V0*sin(theta);

    t_reach = L/vx;
    t_end = ceil((t_reach + 0.05)/Ts)*Ts;
    t_vec = 0:Ts:t_end;

    %% True trajectory
    x_true = vx*t_vec;
    y_true = vy*t_vec;

    %% Noisy measurements
    sigma_x = sqrt(Rd(1,1));
    sigma_y = sqrt(Rd(2,2));
    x_meas = x_true + sigma_x*randn(size(x_true));
    y_meas = y_true + sigma_y*randn(size(y_true));
    Y = [x_meas; y_meas];

    %% Discrete Kalman filter recursion
    nx = 4;
    Ns = length(t_vec);
    xhat_hist = zeros(nx,Ns);

    xhat = xhat0;
    Pk = P0;
    xhat_hist(:,1) = xhat;

    for k = 1:(Ns-1)
        yk = Y(:,k);

        S = Cd*Pk*Cd' + Rd;
        G = (Ad*Pk*Cd')/S;

        xhat_next = Ad*xhat - G*(Cd*xhat - yk);
        Pk_next = Ad*Pk*Ad' - G*S*G' + Qd;

        xhat = xhat_next;
        Pk = Pk_next;
        xhat_hist(:,k+1) = xhat;
    end

    %% Error computation
    x_est = xhat_hist(1,:);
    y_est = xhat_hist(2,:);
    e_pos = sqrt((x_est - x_true).^2 + (y_est - y_true).^2);
    e_reach = interp1(t_vec,e_pos,t_reach,'linear');

    summary(kick,:) = [kick, V0, theta_deg, t_reach, e_reach];

    %% Plot for the current kick
    fig = figure('Color','w','Name',sprintf('Penalty kick %d',kick));

    subplot(3,1,1);
    plot(t_vec,x_true,'LineWidth',1.5); hold on;
    plot(t_vec,x_meas,'.','MarkerSize',8);
    plot(t_vec,x_est,'LineWidth',1.5);
    xline(t_reach,'--k','t_{reach}','LineWidth',1.5);
    yline(L,':k','x = L','LineWidth',1.2);
    grid on;
    ylabel('x [m]');
    title(sprintf('Penalty kick %d: forward position x',kick));
    legend('true','measured','estimated','Location','best');

    subplot(3,1,2);
    plot(t_vec,y_true,'LineWidth',1.5); hold on;
    plot(t_vec,y_meas,'.','MarkerSize',8);
    plot(t_vec,y_est,'LineWidth',1.5);
    xline(t_reach,'--k','t_{reach}','LineWidth',1.5);
    yline(half_goal,':k','goal limit','LineWidth',1.0);
    yline(-half_goal,':k','LineWidth',1.0);
    grid on;
    ylabel('y [m]');
    title('Lateral position y');
    legend('true','measured','estimated','Location','best');

    subplot(3,1,3);
    plot(t_vec,e_pos,'LineWidth',1.5); hold on;
    xline(t_reach,'--k','t_{reach}','LineWidth',1.5);
    yline(threshold,':r','0.25 m threshold','LineWidth',1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('||e_{pos}|| [m]');
    title(sprintf('Position error norm, e(t_{reach}) = %.4f m',e_reach));

    saveas(fig, fullfile(outdir,sprintf('fig2_%02d_penalty_kick.png',kick)));
end

%% Summary table and success count
success = summary(:,5) < threshold;
T = array2table(summary, ...
    'VariableNames', {'Kick','V0_mps','Theta_deg','ReachTime_s','ErrorAtReach_m'});
T.Success = success;

disp(T);
fprintf('\nNumber of successful estimations: %d out of %d\n',sum(success),Nkick);
fprintf('Criterion: position estimation error at reach time < %.2f m\n',threshold);
