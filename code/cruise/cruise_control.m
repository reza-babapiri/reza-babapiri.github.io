%% CMS HW4 - Vehicle Cruise Control
% MATLAB/Simulink code used to generate the figures of the report.
% The script assumes that the Simulink files simu0.slx, simu1.slx,
% simu2.slx, simu3.slx, simu4.slx and simu5.slx are available in the
% current MATLAB folder or available through m.file.zip.

clc; clear; close all;

%% Parameters
Tm    = 190;          % maximum engine torque [N.m]
wm    = 420;          % engine speed at maximum torque [rad/s]
betta = 0.4;          % engine torque curve parameter

g     = 9.81;         % gravity [m/s^2]
rho   = 1.3;          % air density [kg/m^3]
cd    = 0.32;         % drag coefficient
cr    = 0.01;         % rolling resistance coefficient
Area  = 2.4;          % frontal area [m^2]

alpha = [40 25 16 12 10];    % gear ratios, gears 1 to 5
masses = [1000 2000 3000];   % vehicle masses [kg]

kp = 0.5;
ki = 0.1;
kt = ki/kp;                  % anti-windup tracking gain

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end

outdir = fullfile(scriptDir,'CMS_HW4_figures');
if ~exist(outdir,'dir')
    mkdir(outdir);
end

%% Simulink model folder setup
% The report figures require the original Simulink models simu0.slx to
% simu5.slx. This section automatically looks for them in: 
% 1) the current MATLAB folder,
% 2) the folder where this script is located,
% 3) a subfolder named m.file,
% 4) or inside m.file.zip if that zip file is placed next to this script.
requiredModels = {'simu0','simu1','simu2','simu3','simu4','simu5'};
[modelFolder, missingModels] = locateCruiseModelFolder(requiredModels,scriptDir);

if ~isempty(missingModels)
    error(['The following Simulink model(s) were not found: ', strjoin(missingModels, ', '), newline, ...
           'Put this MATLAB file in the same folder as m.file.zip, or extract m.file.zip and run the script again.']);
end

addpath(modelFolder);
fprintf('Using Simulink models from: %s\n', modelFolder);

% Set this to true only if you want MATLAB to export the Simulink block diagrams.
% Keeping it false avoids print/export warnings and does not affect the simulation plots.
SAVE_SIMULINK_DIAGRAMS = false;

%% Figure 1.2 - Engine torque as a function of vehicle velocity
v = linspace(0,45,1000);

figure('Name','Figure 1.2 - Engine torque curves');
hold on; grid on;
for k = 1:length(alpha)
    omega = alpha(k).*v;
    T = Tm.*(1 - betta.*(omega./wm - 1).^2);
    T(T < 0) = 0;
    plot(v,T,'LineWidth',1.4);
end
xlabel('Vehicle velocity v [m/s]');
ylabel('Engine torque T [N.m]');
title('Engine torque as a function of vehicle velocity for different gears');
legend('Gear 1','Gear 2','Gear 3','Gear 4','Gear 5','Location','best');
saveFigure(gcf,outdir,'fig_1_2_engine_torque.png');

%% Figure 1.3 - Simulink diagram of the nonlinear cruise-control model
if SAVE_SIMULINK_DIAGRAMS
    saveModelDiagram('simu0',outdir,'fig_1_3_simu0_diagram.png');
end

%% Figures 1.4 and 1.5 - Hill disturbance for different gears
vr = 20;
theta_in  = 0;
theta_fin = 4*pi/180;

vel_hill = cell(1,5);
thr_hill = cell(1,5);
time_hill = cell(1,5);
ue_fin_hill = zeros(1,5);

for k = 1:5
    simData = runCruiseModel('simu1',alpha(k),1000,vr,theta_in,theta_fin,kp,ki,[],vr,[]);
    time_hill{k} = simData.time;
    vel_hill{k}  = simData.vout;
    thr_hill{k}  = simData.u;
    ue_fin_hill(k) = simData.ue_fin;
end

figure('Name','Figure 1.4 - Velocity for different gears on a hill');
hold on; grid on;
for k = 1:5
    plot(time_hill{k},vel_hill{k},'LineWidth',1.3);
end
ylim([19 21]); yticks(19:0.5:21);
xlim([0 40]); xticks(0:5:40);
xlabel('Time [s]');
ylabel('Vehicle velocity v [m/s]');
title('Vehicle velocity for different gears after encountering a hill');
legend('Gear 1','Gear 2','Gear 3','Gear 4','Gear 5','Location','best');
saveFigure(gcf,outdir,'fig_1_4_velocity_hill_gears.png');

figure('Name','Figure 1.5 - Throttle for different gears on a hill');
hold on; grid on;
for k = 1:5
    plot(time_hill{k},thr_hill{k},'LineWidth',1.3);
end
ylim([0 1]); yticks(0:0.2:1);
xlim([0 40]); xticks(0:5:40);
xlabel('Time [s]');
ylabel('Throttle position u');
title('Throttle response for different gears after encountering a hill');
legend('Gear 1','Gear 2','Gear 3','Gear 4','Gear 5','Location','best');
saveFigure(gcf,outdir,'fig_1_5_throttle_hill_gears.png');

%% Figure 1.6 - Simulink diagram with anti-windup correction
if SAVE_SIMULINK_DIAGRAMS
    saveModelDiagram('simu2',outdir,'fig_1_6_simu2_antiwindup_diagram.png');
end

%% Figures 1.7, 1.8 and 1.9 - Anti-windup comparison
vr = 20;
theta_in  = 0;
theta_fin = 6*pi/180;
a5 = alpha(5);

withoutAW = runCruiseModel('simu3',a5,1000,vr,theta_in,theta_fin,kp,ki,[],vr,[]);
withAW    = runCruiseModel('simu2',a5,1000,vr,theta_in,theta_fin,kp,ki,kt,vr,[]);

figure('Name','Figure 1.7 - Actuation without anti-windup');
plot(withoutAW.time,withoutAW.u,'LineWidth',1.3); hold on;
plot(withoutAW.time,withoutAW.v,'*'); grid on;
ylim([0 3]); yticks(0:0.2:3);
xlim([0 150]); xticks(0:10:150);
xlabel('Time [s]');
ylabel('Actuation / throttle');
title('Actuation before and after saturation without anti-windup correction');
legend('u: after saturation','v: before saturation','Location','best');
saveFigure(gcf,outdir,'fig_1_7_actuation_without_antiwindup.png');

figure('Name','Figure 1.8 - Actuation with anti-windup');
plot(withAW.time,withAW.u,'LineWidth',1.3); hold on;
plot(withAW.time,withAW.v,'*'); grid on;
ylim([0 2]); yticks(0:0.2:2);
xlim([0 80]); xticks(0:10:80);
xlabel('Time [s]');
ylabel('Actuation / throttle');
title('Actuation before and after saturation with anti-windup correction');
legend('u: after saturation','v: before saturation','Location','best');
saveFigure(gcf,outdir,'fig_1_8_actuation_with_antiwindup.png');

figure('Name','Figure 1.9 - Velocity with and without anti-windup');
plot(withoutAW.time,withoutAW.vout,'LineWidth',1.3); hold on;
plot(withAW.time,withAW.vout,'*'); grid on;
ylim([18 22]); yticks(18:1:22);
xlim([0 150]); xticks(0:10:150);
xlabel('Time [s]');
ylabel('Vehicle velocity v [m/s]');
title('Vehicle velocity with and without anti-windup correction');
legend('Without anti-windup','With anti-windup','Location','best');
saveFigure(gcf,outdir,'fig_1_9_velocity_antiwindup_comparison.png');

%% Figures 1.10, 1.11 and 1.12 - Step reference from 25 m/s to 30 m/s
vr = 25;
theta_in  = 0;
theta_fin = 6*pi/180;

refData = cell(1,3);
for k = 1:3
    refData{k} = runCruiseModel('simu4',a5,masses(k),vr,theta_in,theta_fin,kp,ki,kt,vr,[]);
end

figure('Name','Figure 1.10 - Velocity tracking for different masses');
hold on; grid on;
for k = 1:3
    plot(refData{k}.time,refData{k}.vout,'LineWidth',1.3);
end
ylim([20 35]); yticks(20:5:35);
xlim([0 80]); xticks(0:5:80);
xlabel('Time [s]');
ylabel('Vehicle velocity v [m/s]');
title('Tracking a step reference from 25 m/s to 30 m/s for different masses');
legend('m = 1000 kg','m = 2000 kg','m = 3000 kg','Location','best');
saveFigure(gcf,outdir,'fig_1_10_velocity_reference_masses.png');

figure('Name','Figure 1.11 - Actuation before saturation for different masses');
hold on; grid on;
for k = 1:3
    plot(refData{k}.time,refData{k}.v,'LineWidth',1.3);
end
ylim([0 3]); yticks(0:0.5:3);
xlim([0 80]); xticks(0:5:80);
xlabel('Time [s]');
ylabel('Actuation before saturation');
title('Actuation before saturation during reference tracking');
legend('m = 1000 kg','m = 2000 kg','m = 3000 kg','Location','best');
saveFigure(gcf,outdir,'fig_1_11_actuation_before_sat_masses.png');

figure('Name','Figure 1.12 - Actuation after saturation for different masses');
hold on; grid on;
for k = 1:3
    plot(refData{k}.time,refData{k}.u,'LineWidth',1.3);
end
ylim([0 1.5]); yticks(0:0.5:1.5);
xlim([0 80]); xticks(0:5:80);
xlabel('Time [s]');
ylabel('Actuation after saturation');
title('Throttle after saturation during reference tracking');
legend('m = 1000 kg','m = 2000 kg','m = 3000 kg','Location','best');
saveFigure(gcf,outdir,'fig_1_12_actuation_after_sat_masses.png');

%% Figures 1.13 and 1.14 - Reference tracking with hill disturbance
hillRefData = cell(1,3);
for k = 1:3
    hillRefData{k} = runCruiseModel('simu5',a5,masses(k),vr,theta_in,theta_fin,kp,ki,kt,vr,[]);
end

figure('Name','Figure 1.13 - Velocity tracking with hill disturbance');
hold on; grid on;
for k = 1:3
    plot(hillRefData{k}.time,hillRefData{k}.vout,'LineWidth',1.3);
end
ylim([20 35]); yticks(20:5:35);
xlim([0 200]); xticks(0:10:200);
xlabel('Time [s]');
ylabel('Vehicle velocity v [m/s]');
title('Tracking with simultaneous reference and hill disturbance');
legend('m = 1000 kg','m = 2000 kg','m = 3000 kg','Location','best');
saveFigure(gcf,outdir,'fig_1_13_velocity_reference_hill_masses.png');

figure('Name','Figure 1.14 - Throttle with hill disturbance');
hold on; grid on;
for k = 1:3
    plot(hillRefData{k}.time,hillRefData{k}.u,'LineWidth',1.3);
end
ylim([0 1.5]); yticks(0:0.5:1.5);
xlim([0 200]); xticks(0:10:200);
xlabel('Time [s]');
ylabel('Throttle position u');
title('Throttle after saturation during reference tracking with hill disturbance');
legend('m = 1000 kg','m = 2000 kg','m = 3000 kg','Location','best');
saveFigure(gcf,outdir,'fig_1_14_throttle_reference_hill_masses.png');

%% Figures 1.15 to 1.18 - Gang of four transfer functions
vr = 25;
m  = masses(3);
a  = a5;
theta_in = 0;
ue_init = equilibriumThrottle(a,m,vr,theta_in,Tm,wm,betta,g,cr,rho,cd,Area);

A = (a*ue_init*Tm/m)*(-2*betta*(a*vr/wm - 1)*a/wm) - (rho*cd*Area*vr/m);
B = (a*Tm/m)*(1 - betta*((a*vr/wm - 1)^2));

s = tf('s');
P_lin = B/(s - A);
C_pi  = kp + ki/s;
L = minreal(P_lin*C_pi);

S  = minreal(1/(1 + L));          % sensitivity
PS = minreal(P_lin/(1 + L));      % load sensitivity
CS = minreal(C_pi/(1 + L));       % control sensitivity
T  = minreal(L/(1 + L));          % complementary sensitivity

w = logspace(-4,3,8000);

plotBodeAndSave(S,w,'Figure 1.15 - Sensitivity bode plot',outdir,'fig_1_15_bode_sensitivity.png');
plotBodeAndSave(PS,w,'Figure 1.16 - Load sensitivity bode plot',outdir,'fig_1_16_bode_load_sensitivity.png');
plotBodeAndSave(CS,w,'Figure 1.17 - Control sensitivity bode plot',outdir,'fig_1_17_bode_control_sensitivity.png');
plotBodeAndSave(T,w,'Figure 1.18 - Complementary sensitivity bode plot',outdir,'fig_1_18_bode_complementary_sensitivity.png');

[magS,~,wS] = bode(S,w);
magS = squeeze(magS);
[Ms,idxS] = max(magS);
Sm = 1/Ms;

fprintf('Linearised model: A = %.4f, B = %.4f\n',A,B);
fprintf('Maximum sensitivity Ms = %.4f at w = %.4f rad/s\n',Ms,wS(idxS));
fprintf('Stability margin Sm = 1/Ms = %.4f\n',Sm);
fprintf('All figures were saved in: %s\n',outdir);

%% Local functions
function [modelFolder, missingModels] = locateCruiseModelFolder(modelNames,scriptDir)
    modelFolder = '';
    missingModels = modelNames;

    candidateDirs = {pwd, scriptDir, fullfile(pwd,'m.file'), fullfile(scriptDir,'m.file')};

    % If m.file.zip exists next to the script or in the current folder, unzip it.
    zipCandidates = {fullfile(scriptDir,'m.file.zip'), fullfile(pwd,'m.file.zip')};
    for zz = 1:numel(zipCandidates)
        if exist(zipCandidates{zz},'file') == 2
            try
                unzip(zipCandidates{zz}, fileparts(zipCandidates{zz}));
                candidateDirs{end+1} = fullfile(fileparts(zipCandidates{zz}),'m.file'); %#ok<AGROW>
            catch ME
                warning('Could not unzip %s. Reason: %s', zipCandidates{zz}, ME.message);
            end
        end
    end

    % Search recursively as a backup.
    searchRoots = unique({pwd, scriptDir});
    for rr = 1:numel(searchRoots)
        if exist(searchRoots{rr},'dir') == 7
            hits = dir(fullfile(searchRoots{rr},'**',[modelNames{1} '.slx']));
            for hh = 1:numel(hits)
                candidateDirs{end+1} = hits(hh).folder; %#ok<AGROW>
            end
        end
    end

    candidateDirs = unique(candidateDirs,'stable');

    for cc = 1:numel(candidateDirs)
        folder = candidateDirs{cc};
        if exist(folder,'dir') ~= 7
            continue;
        end

        missingHere = {};
        for ii = 1:numel(modelNames)
            modelName = modelNames{ii};
            hasSlx = exist(fullfile(folder,[modelName '.slx']),'file') == 2;
            hasMdl = exist(fullfile(folder,[modelName '.mdl']),'file') == 2;
            if ~(hasSlx || hasMdl)
                missingHere{end+1} = modelName; %#ok<AGROW>
            end
        end

        if isempty(missingHere)
            modelFolder = folder;
            missingModels = {};
            return;
        end
    end
end

function data = runCruiseModel(modelName,alpha_n,m,vr,theta_in,theta_fin,kp,ki,kt,vinit,ue0)
    Tm = evalin('base','Tm');
    wm = evalin('base','wm');
    betta = evalin('base','betta');
    g = evalin('base','g');
    rho = evalin('base','rho');
    cd = evalin('base','cd');
    cr = evalin('base','cr');
    Area = evalin('base','Area');

    coeff1 = alpha_n*Tm/m;
    coeff2 = alpha_n/wm;
    coeff3 = -g*cr;
    coeff4 = -rho*cd*Area/(2*m);
    coeff5 = -g;

    if isempty(ue0)
        ue_init = equilibriumThrottle(alpha_n,m,vr,theta_in,Tm,wm,betta,g,cr,rho,cd,Area);
    else
        ue_init = ue0;
    end
    ue_fin = equilibriumThrottle(alpha_n,m,vr,theta_fin,Tm,wm,betta,g,cr,rho,cd,Area);

    load_system(modelName);
    setGain(modelName,'coeff1',coeff1);
    setGain(modelName,'coeff2',coeff2);
    setGain(modelName,'coeff3',coeff3);
    setGain(modelName,'coeff4',coeff4);
    setGain(modelName,'coeff5',coeff5);
    setGain(modelName,'kp',kp);
    setGain(modelName,'ki',ki);
    setGain(modelName,'Betta',betta);

    if ~isempty(kt)
        setGain(modelName,'kt',kt);
    end

    setInitialCondition(modelName,'Integrator1',ue_init);
    if ~isempty(vinit)
        setInitialCondition(modelName,'Integrator',vinit);
    end

    simOut = sim(modelName);

    data.time = simOut.get('vout').time;
    data.vout = simOut.get('vout').Data;
    data.u = simOut.get('u').Data;
    data.ue_fin = ue_fin;

    try
        data.v = simOut.get('v').Data;
    catch
        data.v = [];
    end
end

function ue = equilibriumThrottle(alpha_n,m,vr,theta,Tm,wm,betta,g,cr,rho,cd,Area)
    numerator = m*g*cr*sign(vr) + 0.5*rho*cd*Area*(vr^2) + m*g*sin(theta);
    denominator = alpha_n*Tm*(1 - betta*((alpha_n*vr/wm - 1)^2));
    ue = numerator/denominator;
end

function setGain(modelName,blockName,value)
    blockPath = [modelName '/' blockName];
    try
        set_param(blockPath,'K',num2str(value,16));
    catch
        warning('Block %s was not found or could not be updated.',blockPath);
    end
end

function setInitialCondition(modelName,blockName,value)
    blockPath = [modelName '/' blockName];
    try
        set_param(blockPath,'InitialCondition',num2str(value,16));
    catch
        warning('Initial condition for block %s was not updated.',blockPath);
    end
end

function saveModelDiagram(modelName,outdir,fileName)
    try
        load_system(modelName);
        open_system(modelName);
        print(['-s' modelName],'-dpng','-r300',fullfile(outdir,fileName));
    catch ME
        warning('Could not save the Simulink diagram for %s. Reason: %s',modelName,ME.message);
    end
end

function plotBodeAndSave(sys,w,figTitle,outdir,fileName)
    figure('Name',figTitle);
    bode(sys,w); grid on;
    title(figTitle);
    saveFigure(gcf,outdir,fileName);
end

function saveFigure(figHandle,outdir,fileName)
    try
        exportgraphics(figHandle,fullfile(outdir,fileName),'Resolution',300);
    catch
        saveas(figHandle,fullfile(outdir,fileName));
    end
end
