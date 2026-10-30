%% HW4 Part 2 - Adaptive Cruise Control
% Complete MATLAB code matched with the report figures and calculations.
% Reza Babapiri
%
% This script reproduces the computations used for:
% Figure 1.1  - PID tracking error for a parabolic reference
% Figure 2.1  - Simulink block diagram for observer-based control
% Figure 2.2  - Error response for the observer-based design
% Figure 3.1  - Leader vehicle speed profile
% Figure 3.2  - Simulink block diagram for the PID controller
% Figure 3.3  - PID tracking error
% Figure 3.4  - Simulink block diagram for the observer-based controller
% Figure 3.5  - Observer-based tracking error
% Figure 3.6  - PID sensitivity bode diagram
% Figure 3.7  - PID complementary sensitivity bode diagram
% Figure 3.8  - PID load sensitivity bode diagram
% Figure 3.9  - PID control sensitivity bode diagram
% Figure 3.10 - Observer-based sensitivity bode diagram
% Figure 3.11 - Observer-based complementary sensitivity bode diagram
% Figure 3.12 - Observer-based load sensitivity bode diagram
% Figure 3.13 - Observer-based control sensitivity bode diagram

clear; close all; clc;

%% Folder management
thisFile = mfilename('fullpath');
if isempty(thisFile)
    rootDir = pwd;
else
    rootDir = fileparts(thisFile);
end

figDir = fullfile(rootDir,'generated_figures');
if ~exist(figDir,'dir')
    mkdir(figDir);
end

modelDir = fullfile(rootDir,'simulink_models');
if exist(modelDir,'dir')
    addpath(modelDir);
end
addpath(rootDir);

%% Common parameters used in the report
mB = 1500;                      % follower vehicle mass [kg]
d_ref = 5;                      % desired inter-vehicle distance [m]

% PID gains selected from the desired closed-loop polynomial:
% (s+2)(s+3)(s+4) = s^3 + 9s^2 + 26s + 24
Kd = 9*mB;
Kp = 26*mB;
Ki = 24*mB;

s = tf('s');
P = 1/(mB*s^2);                 % plant from force input to position output
Cpid = (Kd*s^2 + Kp*s + Ki)/s;  % ideal PID controller

%% ========================================================================
%% Chapter 1 - PID feedback control
%% Figure 1.1: Tracking error for a parabolic reference
T_pid = feedback(Cpid*P,1);

t1 = (0:0.001:10).';
x0A = 5;
v0A = 0;
aA  = 1;

xA_parabolic = x0A + v0A*t1 + 0.5*aA*t1.^2;
r_parabolic  = xA_parabolic - d_ref;
xB_parabolic = lsim(T_pid,r_parabolic,t1);
e_parabolic  = r_parabolic - xB_parabolic;

figure('Name','Figure 1.1 - Tracking error for a parabolic reference');
plot(t1,e_parabolic,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Error e(t)');
title('Tracking error for a parabolic reference');
saveFigure(figDir,'fig1_1_tracking_error_parabolic');

%% ========================================================================
%% Chapter 2 - Observer-based control
%% Observer and controller gains used in the Simulink model
% Observer poles: (s+10)^3
% Controller poles: (s+2)(s+3)
kC1 = 6*mB;
kC2 = 5*mB;
kO1 = -30;
kO2 = -300;
kO3 = -1000*mB;

% Save the Simulink diagram corresponding to Figure 2.1, if available.
saveSimulinkDiagram('observerbsed',figDir,'fig2_1_observer_based_block_diagram');

% Figure 2.2 can be obtained from the observerbsed.slx scope.  The model is
% included in the package.  The following command runs it with the same gains
% used in the report.
runModelIfAvailable('observerbsed');

%% Pure MATLAB observer-based simulation for the same parabolic manoeuvre
[Aobs,Bobs,Cobs,Dobs,Kobs,Lobs] = observerControllerMatrices(mB);
Cobserver = ss(Aobs,Bobs,Cobs,Dobs);
T_obs = feedback(Cobserver*P,1);
xB_obs_parabolic = lsim(T_obs,r_parabolic,t1);
e_obs_parabolic  = r_parabolic - xB_obs_parabolic;

figure('Name','Figure 2.2 - Observer-based distance error');
plot(t1,e_obs_parabolic,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Error e(t)');
title('Distance error using the observer-based controller');
saveFigure(figDir,'fig2_2_observer_based_error');

%% ========================================================================
%% Chapter 3 - Comparison between PID and observer-based controllers
%% Figure 3.1: Leader vehicle speed curve
[t3,vA,aA,xA] = leaderProfile();
r3 = xA - d_ref;

figure('Name','Figure 3.1 - Leader vehicle speed curve');
plot(t3,vA,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Leader velocity v_A(t) [m/s]');
title('Leader vehicle A speed curve');
saveFigure(figDir,'fig3_1_leader_speed_curve');

%% Figure 3.2: PID Simulink block diagram
saveSimulinkDiagram('PID',figDir,'fig3_2_pid_block_diagram');

%% Figure 3.3: PID tracking error for the leader velocity profile
xB_pid = lsim(T_pid,r3,t3);
e_pid  = r3 - xB_pid;

figure('Name','Figure 3.3 - PID tracking error');
plot(t3,e_pid,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Error e(t)');
title('Tracking error using the PID controller');
saveFigure(figDir,'fig3_3_pid_error');

%% Figure 3.4: Observer-based Simulink block diagram
saveSimulinkDiagram('observerbsed1',figDir,'fig3_4_observer_based_block_diagram');

%% Figure 3.5: Observer-based tracking error for the leader velocity profile
xB_obs = lsim(T_obs,r3,t3);
e_obs  = r3 - xB_obs;

figure('Name','Figure 3.5 - Observer-based tracking error');
plot(t3,e_obs,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Error e(t)');
title('Tracking error using the observer-based controller');
saveFigure(figDir,'fig3_5_observer_based_error');

% The Simulink models used in the original report are also executed here if
% they are available in the simulink_models folder.
runModelIfAvailable('PID');
runModelIfAvailable('observerbsed1');

%% ========================================================================
%% Gang of four for the PID controller
Lpid = minreal(P*Cpid);
S_pid  = minreal(1/(1 + Lpid));      % sensitivity
T_pid2 = minreal(1 - S_pid);         % complementary sensitivity / tracking
PS_pid = minreal(P*S_pid);           % load sensitivity
CS_pid = minreal(Cpid*S_pid);        % control sensitivity

w = logspace(-4,3,5000);
magS = squeeze(abs(freqresp(S_pid,w)));
Ms_pid = max(magS);
SM_pid = 1/Ms_pid;

fprintf('\nPID desig<local path redacted>');
fprintf('Maximum sensitivity Ms = %.4f\n',Ms_pid);
fprintf('Stability margin SM = 1/Ms = %.4f\n',SM_pid);

plotBodeManual(S_pid,w,figDir,'fig3_6_pid_sensitivity_bode', ...
    'Figure 3.6 - Sensitivity bode diagram for PID controller design');
plotBodeManual(T_pid2,w,figDir,'fig3_7_pid_tracking_bode', ...
    'Figure 3.7 - Tracking bode diagram for PID controller design');
plotBodeManual(PS_pid,w,figDir,'fig3_8_pid_load_sensitivity_bode', ...
    'Figure 3.8 - Load sensitivity bode diagram for PID controller design');
plotBodeManual(CS_pid,w,figDir,'fig3_9_pid_control_sensitivity_bode', ...
    'Figure 3.9 - Control sensitivity bode diagram for PID controller design');

%% ========================================================================
%% Gang of four for the output-feedback observer-based controller
Lobs = minreal(P*Cobserver);
S_obs  = minreal(1/(1 + Lobs));      % sensitivity
T_obs2 = minreal(1 - S_obs);         % complementary sensitivity / tracking
PS_obs = minreal(P*S_obs);           % load sensitivity
CS_obs = minreal(Cobserver*S_obs);   % control sensitivity

magS_obs = squeeze(abs(freqresp(S_obs,w)));
Ms_obs = max(magS_obs);
SM_obs = 1/Ms_obs;

fprintf('\nObserver-based desig<local path redacted>');
fprintf('Maximum sensitivity Ms = %.4f\n',Ms_obs);
fprintf('Stability margin SM = 1/Ms = %.4f\n',SM_obs);

plotBodeManual(S_obs,w,figDir,'fig3_10_observer_sensitivity_bode', ...
    'Figure 3.10 - Sensitivity bode diagram for output feedback observer-based design');
plotBodeManual(T_obs2,w,figDir,'fig3_11_observer_tracking_bode', ...
    'Figure 3.11 - Tracking bode diagram for output feedback observer-based design');
plotBodeManual(PS_obs,w,figDir,'fig3_12_observer_load_sensitivity_bode', ...
    'Figure 3.12 - Load sensitivity bode diagram for output feedback observer-based design');
plotBodeManual(CS_obs,w,figDir,'fig3_13_observer_control_sensitivity_bode', ...
    'Figure 3.13 - Control sensitivity bode diagram for output feedback observer-based design');

fprintf('\nDone. All generated figures were saved i<local path redacted>',figDir);

%% ========================================================================
%% Local functions
function [t,vA,aA,xA] = leaderProfile()
    t = (0:0.001:20).';
    aA = zeros(size(t));
    aA(t >= 0  & t < 5)  = 1;
    aA(t >= 10 & t < 15) = -1;

    vA = zeros(size(t));
    for k = 2:length(t)
        dt = t(k)-t(k-1);
        vA(k) = vA(k-1) + aA(k-1)*dt;
        if vA(k) < 0
            vA(k) = 0;
        end
    end

    xA = 5 + cumtrapz(t,vA);
end

function [Acomp,Bcomp,Ccomp,Dcomp,K,L] = observerControllerMatrices(mB)
    % Extended system used for disturbance/acceleration estimation.
    A = [0 1 0;
         0 0 1/mB;
         0 0 0];
    B = [0; 1/mB; 0];
    C = [1 0 0];

    % Controller and observer gains used in the report and Simulink models.
    K = [6*mB 5*mB 1];
    L = [30; 300; 1000*mB];

    % Dynamic compensator from measured error y to control input u.
    Acomp = A - B*K - L*C;
    Bcomp = L;
    Ccomp = -K;
    Dcomp = 0;
end

function plotBodeManual(sys,w,figDir,fileName,plotTitle)
    H = squeeze(freqresp(sys,w));
    magDB = 20*log10(abs(H));
    phaseDeg = unwrap(angle(H))*180/pi;

    figure('Name',plotTitle);
    subplot(2,1,1);
    semilogx(w,magDB,'LineWidth',1.2);
    grid on;
    ylabel('Magnitude (dB)');
    title(plotTitle);

    subplot(2,1,2);
    semilogx(w,phaseDeg,'LineWidth',1.2);
    grid on;
    xlabel('Frequency (rad/s)');
    ylabel('Phase (deg)');

    saveFigure(figDir,fileName);
end

function saveFigure(figDir,fileName)
    pngPath = fullfile(figDir,[fileName,'.png']);
    figPath = fullfile(figDir,[fileName,'.fig']);
    try
        exportgraphics(gcf,pngPath,'Resolution',300);
    catch
        saveas(gcf,pngPath);
    end
    savefig(gcf,figPath);
end

function runModelIfAvailable(modelName)
    if exist([modelName,'.slx'],'file') ~= 2 && exist(modelName,'file') ~= 4
        fprintf('Model %s.slx was not found. Skipping Simulink simulation.\n',modelName);
        return;
    end

    try
        load_system(modelName);
        sim(modelName);
        fprintf('Simulink model %s was executed successfully.\n',modelName);
    catch ME
        warning('Could not run Simulink model %s: %s',modelName,ME.message);
    end
end

function saveSimulinkDiagram(modelName,figDir,fileName)
    if exist([modelName,'.slx'],'file') ~= 2 && exist(modelName,'file') ~= 4
        fprintf('Model %s.slx was not found. Diagram was not saved.\n',modelName);
        return;
    end

    try
        load_system(modelName);
        open_system(modelName);
        pngPath = fullfile(figDir,[fileName,'.png']);
        print(['-s',modelName],'-dpng','-r300',pngPath);
        fprintf('Diagram for %s saved successfully.\n',modelName);
    catch ME
        warning('Could not save the Simulink diagram for %s: %s',modelName,ME.message);
    end
end
