%% HW5 Part 2 - BMW 730 active steering control
% Report-matched MATLAB implementation
% Reza Babapiri
%
% This script reproduces the main numerical computations and plots used in
% the report:
%   1) PI gain design for yaw-rate feedback
%   2) Luenberger observer design
%   3) Wiener filter design
%   4) Observer-based output-feedback control with integral action
%   5) Gang-of-four transfer functions and stability margins
%
% The Simulink models sim6_1 ... sim6_5 are used for the time-domain plots.
% If the models are placed in the same folder as this file, the script adds
% that folder automatically to the MATLAB path.

clear; close all; clc;

%% Auto path and output folder
thisFile = mfilename('fullpath');
if isempty(thisFile)
    rootDir = pwd;
else
    rootDir = fileparts(thisFile);
end
addpath(rootDir);

outDir = fullfile(rootDir,'generated_figures');
if ~exist(outDir,'dir')
    mkdir(outDir);
end

%% Vehicle parameters - BMW 730 bicycle model
par.m  = 1864;       % mass [kg]
par.J  = 3654;       % yaw inertia [kg m^2]
par.Cr = 213800;     % rear cornering stiffness [N/rad]
par.Cf = 101600;     % front cornering stiffness [N/rad]
par.Lr = 1.32;       % rear axle distance [m]
par.Lf = 1.51;       % front axle distance [m]

par.Alpha = par.Cf/par.m + par.Cr/par.m;
par.Betta = -par.Cf*par.Lf + par.Cr*par.Lr;
par.Gemma = (par.Cf*par.Lf^2 + par.Cr*par.Lr^2)/par.J;
par.b1 = par.Cf/par.m;
par.b2 = par.Cf*par.Lf/par.J;

C_yaw = [0 1];
C_lat = [1 0];

%% Chapter 1 - PI gain design
Vgrid = 0.1:0.1:(245/3.6);
K_PI = zeros(numel(Vgrid),3);    % [ki kp V]
closedLoopRoots = zeros(numel(Vgrid),3);

for idx = 1:numel(Vgrid)
    V = Vgrid(idx);
    [A,B] = vehicleMatrices(par,V,1);
    [kp,ki,rootsCL] = designPIYaw(A,B,C_yaw,par,V);
    K_PI(idx,:) = [ki kp V];
    closedLoopRoots(idx,:) = rootsCL(:).';
end

figure('Name','Figure 1.2 - PI gains');
plot(K_PI(:,3),K_PI(:,2),'LineWidth',1.3); hold on;
plot(K_PI(:,3),K_PI(:,1),'LineWidth',1.3);
grid on; xlabel('Velocity [m/s]'); ylabel('Gain value');
title('Figure 1.2 - PI gains versus forward speed');
legend('k_p','k_i','Location','best');
saveFigure(outDir,'fig1_2_PI_gains.png');

% Pole-placement admissible region, used in Fig. 1.1
plotAdmissiblePoleRegion(outDir);

%% Chapter 2 - Luenberger observer design at V = 72 km/h
V0 = 72/3.6;
[A0,B0] = vehicleMatrices(par,V0,1);
K_L = designLuenberger(A0,C_yaw,-10);

% Run Simulink model sim6_1 if available
if modelAvailable('sim6_1')
    setBMWWorkspace(par,V0,1);
    ko1 = K_L(1); ko2 = K_L(2);
    vy_init = 2; vy_hat_init = 0;
    omega_init = 0; omega_hat_init = 0;
    assignin('base','ko1',ko1); assignin('base','ko2',ko2);
    assignin('base','vy_init',vy_init); assignin('base','vy_hat_init',vy_hat_init);
    assignin('base','omega_init',omega_init); assignin('base','omega_hat_init',omega_hat_init);
    setupModelParameters('sim6_1',par,V0,1);
    setIfBlockExists('sim6_1/Integrator1','InitialCondition','vy_init');
    setIfBlockExists('sim6_1/Integrator2','InitialCondition','omega_init');
    setIfBlockExists('sim6_1/Integrator3','InitialCondition','vy_hat_init');
    setIfBlockExists('sim6_1/Integrator4','InitialCondition','omega_hat_init');
    simOut = sim('sim6_1');
    plotSignalPair(simOut,'Vy','Vy_hat','Figure 2.2 - Luenberger lateral-speed estimate',...
        'Time [s]','Lateral speed [m/s]',outDir,'fig2_2_luenberger_lateral.png');
    plotSignalPair(simOut,'omega','omega_hat','Figure 2.3 - Luenberger yaw-rate estimate',...
        'Time [s]','Yaw rate [rad/s]',outDir,'fig2_3_luenberger_yaw.png');
else
    warning('sim6_1.slx was not found. Skipping Luenberger simulation figures.');
end

%% Chapter 2 - Wiener filter design at V = 72 km/h
K_W = designWiener(A0,B0,C_yaw);
if modelAvailable('sim6_2')
    setBMWWorkspace(par,V0,1);
    ko1 = K_W(1); ko2 = K_W(2);
    assignin('base','ko1',ko1); assignin('base','ko2',ko2);
    setupModelParameters('sim6_2',par,V0,1);
    setIfBlockExists('sim6_2/Integrator1','InitialCondition','vy_init');
    setIfBlockExists('sim6_2/Integrator2','InitialCondition','omega_init');
    setIfBlockExists('sim6_2/Integrator3','InitialCondition','vy_hat_init');
    setIfBlockExists('sim6_2/Integrator4','InitialCondition','omega_hat_init');
    simOut = sim('sim6_2');
    plotSignalPair(simOut,'Vy','Vy_hat','Figure 2.5 - Wiener lateral-speed estimate',...
        'Time [s]','Lateral speed [m/s]',outDir,'fig2_5_wiener_lateral.png');
    plotSignalPair(simOut,'omega','omega_hat','Figure 2.6 - Wiener yaw-rate estimate',...
        'Time [s]','Yaw rate [rad/s]',outDir,'fig2_6_wiener_yaw.png');
else
    warning('sim6_2.slx was not found. Skipping Wiener-filter simulation figures.');
end

%% Chapter 3 - Observer-based control with integral action
[kc,KobsExt,Aext,Bext,Cext] = designObserverBasedController(par,V0);

if modelAvailable('sim6_3')
    setBMWWorkspace(par,V0,1);
    assignObserverControllerToBase(kc,KobsExt,par,V0);
    setupModelParameters('sim6_3',par,V0,1);
    simOut = sim('sim6_3');
    plotSignalPair(simOut,'omega','omega_hat','Figure 3.2 - Observer-based yaw-rate response',...
        'Time [s]','Yaw rate [rad/s]',outDir,'fig3_2_observer_based_yaw.png');
else
    warning('sim6_3.slx was not found. Skipping observer-based controller simulation.');
end

%% Chapter 4 - Comparison of PI and observer-based controllers
if modelAvailable('sim6_4') && modelAvailable('sim6_5')
    % Observer-based response
    setBMWWorkspace(par,V0,1);
    assignObserverControllerToBase(kc,KobsExt,par,V0);
    setupModelParameters('sim6_4',par,V0,1);
    simOutObs = sim('sim6_4');
    omegaObs = simOutObs.get('omega').Data;
    timeObs  = simOutObs.get('omega').time;

    % PI response
    [kp0,ki0] = designPIYaw(A0,B0,C_yaw,par,V0);
    assignin('base','k_p',kp0); assignin('base','k_i',ki0);
    setupModelParameters('sim6_5',par,V0,1);
    simOutPI = sim('sim6_5');
    omegaPI = simOutPI.get('omega').Data;
    timePI  = simOutPI.get('omega').time;

    figure('Name','Figure 4.1 - PI vs observer-based response');
    plot(timeObs,omegaObs(:,1),'LineWidth',1.3); hold on;
    plot(timePI,omegaPI(:,1),'LineWidth',1.3);
    grid on; xlabel('Time [s]'); ylabel('Yaw rate [rad/s]');
    title('Figure 4.1 - Response with constant steering-channel disturbance');
    legend('Observer-based control','PI control','Location','best');
    saveFigure(outDir,'fig4_1_PI_vs_observer.png');
else
    warning('sim6_4.slx and/or sim6_5.slx were not found. Skipping comparison simulation.');
end

%% Gang of four transfer functions at V = 72 km/h
s = tf('s');
I2 = eye(2);
P_yaw = minreal(C_yaw*((s*I2 - A0)\B0));
[kp0,ki0] = designPIYaw(A0,B0,C_yaw,par,V0);
C_PI = kp0 + ki0/s;
C_OBS = observerCompensatorTF(Aext,Bext,Cext,kc,KobsExt);

plotGangOfFour(P_yaw,C_PI,outDir,'PI control','PI');
plotGangOfFour(P_yaw,C_OBS,outDir,'observer-based control','OBS');

%% Stability margin as a function of velocity
STABILITY_MARGIN = zeros(numel(Vgrid),3);    % [V, observer, PI]
for idx = 1:numel(Vgrid)
    V = Vgrid(idx);
    [A,B] = vehicleMatrices(par,V,1);
    P = C_yaw*((s*eye(2)-A)\B);
    [kp,ki] = designPIYaw(A,B,C_yaw,par,V);
    Cpi = kp + ki/s;
    [kcV,KoV,AexV,BexV,CexV] = designObserverBasedController(par,V);
    Cobs = observerCompensatorTF(AexV,BexV,CexV,kcV,KoV);
    STABILITY_MARGIN(idx,1) = V;
    STABILITY_MARGIN(idx,2) = stabilityMargin(P*Cobs);
    STABILITY_MARGIN(idx,3) = stabilityMargin(P*Cpi);
end

figure('Name','Figure 4.10 - Stability margins');
plot(STABILITY_MARGIN(:,1),STABILITY_MARGIN(:,3),'LineWidth',1.3); hold on;
plot(STABILITY_MARGIN(:,1),STABILITY_MARGIN(:,2),'LineWidth',1.3);
grid on; xlabel('Velocity [m/s]'); ylabel('Stability margin');
title('Figure 4.10 - Stability margin versus forward speed');
legend('PI controller','Observer-based controller','Location','best');
saveFigure(outDir,'fig4_10_stability_margin.png');

disp('HW5 Part 2 computations completed. Figures are saved in generated_figures/.');

%% Local functions
function [A,B] = vehicleMatrices(par,V,mu)
    if V == 0
        error('Forward speed V must be non-zero.');
    end
    Alpha = mu*par.Alpha;
    Betta = mu*par.Betta;
    Gemma = mu*par.Gemma;
    b1 = mu*par.Cf/par.m;
    b2 = mu*par.Cf*par.Lf/par.J;
    a11 = -Alpha/V;
    a12 = -(V - Betta/(par.m*V));
    a21 = Betta/(par.J*V);
    a22 = -Gemma/V;
    A = [a11 a12; a21 a22];
    B = [b1; b2];
end

function [kp,ki,rootsCL] = designPIYaw(A,B,C,par,V)
    E = eig(A);
    slowPole = real(E(2));
    a11 = A(1,1); a12 = A(1,2); a21 = A(2,1); a22 = A(2,2);
    b1 = B(1); b2 = B(2);
    A1 = -(a11+a22);
    A2 = a11*a22 - a12*a21;
    B1 = b2;
    B2 = a21*b1 - a11*b2;
    bestFound = false;
    for zeta = 0.7:0.1:100
        c1 = -(B1/B2)*slowPole - 1;
        c2 = 2*zeta*(B2/B1) + 2*zeta*slowPole;
        c3 = A2 + (B2/B1)*(-slowPole - A1);
        candidate = roots([c1 c2 c3]);
        for rr = 1:numel(candidate)
            wn = candidate(rr);
            if isreal(wn) && wn >= 0 && -zeta*wn <= slowPole
                bestFound = true;
                break;
            end
        end
        if bestFound
            break;
        end
    end
    if ~bestFound
        warning('No admissible PI solution found at V = %.2f m/s. Falling back to zeta = 0.7.', V);
        zeta = 0.7;
        wn = abs(slowPole);
    end
    ki = (-slowPole/B2)*(wn^2);
    kp = (2*zeta*wn - slowPole - A1)/B1;
    rootsCL = roots([1, A1+B1*kp, A2+B1*ki+B2*kp, B2*ki]);
end

function K = designLuenberger(A,C,observerPole)
    O = [C; C*A];
    I = eye(size(A));
    K = -((A - observerPole*I)^2) / O * [0;1];
end

function K = designWiener(A,B,C)
    q = 1;
    r = 1;
    Q = B*q*B';
    R = r;
    [P,~,~] = care(A',C',Q,R);
    K = -P*C'/R;
end

function [kc,Kobs,Aext,Bext,Cext] = designObserverBasedController(par,V)
    [A,B] = vehicleMatrices(par,V,1);
    C = [0 1];
    E = eig(A);
    R = [B A*B];
    I2 = eye(2);
    kc = -[0 1]/R*((A - real(E(2))*I2)^2);
    Aext = [A, -B; 0 0 0];
    Bext = [B;0];
    Cext = [0 1 0];
    Oext = [Cext; Cext*Aext; Cext*Aext*Aext];
    I3 = eye(3);
    Kobs = -((Aext - 10*real(E(2))*I3)^3) / Oext * [0;0;1];
end

function Cdyn = observerCompensatorTF(Aext,Bext,Cext,kc,Kobs)
    s = tf('s');
    I = eye(size(Aext));
    kC = [kc 1];
    Cdyn = minreal(-kC*((s*I - (Aext + Bext*kC + Kobs*Cext))\Kobs));
end

function Sm = stabilityMargin(L)
    w = logspace(-3,4,3000);
    Ljw = squeeze(freqresp(L,w));
    Sm = min(abs(1 + Ljw));
end

function plotGangOfFour(P,C,outDir,label,prefix)
    S  = minreal(1/(1+P*C));
    T  = minreal(1-S);
    LS = minreal(P*S);
    CS = minreal(C*S);
    plotBode(S,  sprintf('Sensitivity - %s',label),              outDir, sprintf('fig_%s_sensitivity.png',prefix));
    plotBode(T,  sprintf('Tracking - %s',label),                 outDir, sprintf('fig_%s_tracking.png',prefix));
    plotBode(LS, sprintf('Load sensitivity - %s',label),         outDir, sprintf('fig_%s_load_sensitivity.png',prefix));
    plotBode(CS, sprintf('Control sensitivity - %s',label),      outDir, sprintf('fig_%s_control_sensitivity.png',prefix));
end

function plotBode(sysObj,plotTitle,outDir,fileName)
    figure('Name',plotTitle);
    bode(sysObj); grid on;
    title(plotTitle);
    saveFigure(outDir,fileName);
end

function plotAdmissiblePoleRegion(outDir)
    zeta = 0.7;
    x = linspace(-20,0,400);
    y = abs(x).*sqrt(1/zeta^2 - 1);
    figure('Name','Figure 1.1 - Desired pole region');
    plot(x,y,'LineWidth',1.2); hold on; plot(x,-y,'LineWidth',1.2);
    xline(0,'k'); yline(0,'k'); grid on;
    xlabel('Real axis'); ylabel('Imaginary axis');
    title('Figure 1.1 - Locus of admissible complex conjugate poles');
    saveFigure(outDir,'fig1_1_pole_region.png');
end

function saveFigure(outDir,fileName)
    set(gcf,'Color','w');
    try
        exportgraphics(gcf,fullfile(outDir,fileName),'Resolution',200);
    catch
        saveas(gcf,fullfile(outDir,fileName));
    end
end

function tfAvailable = modelAvailable(modelName)
    tfAvailable = exist([modelName '.slx'],'file') == 4 || exist(modelName,'file') == 4;
end

function setBMWWorkspace(par,V,mu)
    [A,B] = vehicleMatrices(par,V,mu);
    assignin('base','a11',A(1,1)); assignin('base','a12',A(1,2));
    assignin('base','a21',A(2,1)); assignin('base','a22',A(2,2));
    assignin('base','b1',B(1)); assignin('base','b2',B(2));
end

function setupModelParameters(modelName,par,V,mu)
    load_system(modelName);
    [A,B] = vehicleMatrices(par,V,mu);
    vars = {'a11',A(1,1); 'a12',A(1,2); 'a21',A(2,1); 'a22',A(2,2); 'b1',B(1); 'b2',B(2)};
    for ii = 1:size(vars,1)
        setIfBlockExists([modelName '/' vars{ii,1}], 'K', vars{ii,1});
        assignin('base',vars{ii,1},vars{ii,2});
    end
end

function setIfBlockExists(blockPath,paramName,paramValue)
    try
        set_param(blockPath,paramName,paramValue);
    catch
        % Some model versions may have slightly different block names.
    end
end

function assignObserverControllerToBase(kc,Kobs,par,V)
    [A,B] = vehicleMatrices(par,V,1);
    assignin('base','kc1',kc(1)); assignin('base','kc2',kc(2));
    assignin('base','ko1',Kobs(1)); assignin('base','ko2',Kobs(2)); assignin('base','ko3',Kobs(3));
    assignin('base','b1',B(1)); assignin('base','b2',B(2));
    assignin('base','b3',B(2)); assignin('base','b4',B(1));
    assignin('base','b5',B(1)); assignin('base','b6',B(2));
    assignin('base','vy_init',0); assignin('base','omega_init',0);
    assignin('base','Vy_tilda_hat_init',0); assignin('base','omega_tilda_hat_init',0);
    assignin('base','delta_star_hat_init',0);
end

function plotSignalPair(simOut,name1,name2,plotTitle,xlab,ylab,outDir,fileName)
    t = simOut.get(name1).time;
    y1 = simOut.get(name1).Data;
    y2 = simOut.get(name2).Data;
    figure('Name',plotTitle);
    plot(t,y1(:,1),'LineWidth',1.2); hold on;
    plot(t,y2(:,1),'*');
    grid on; xlabel(xlab); ylabel(ylab); title(plotTitle);
    legend(name1,name2,'Location','best');
    saveFigure(outDir,fileName);
end
