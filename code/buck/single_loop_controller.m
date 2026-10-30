%%*********************************Reza Babapiri***************************************
clear all; clc; close all;
Ea = 100; % nominal voltage[v]
R = 2.3; % load resistor[ohm]
fs = 50e3; %[Hz]
Vu = 48; % desired output voltage[v]
T = 1/fs;
dutycycle = Vu/Ea;
Tau = dutycycle * T;
Iu = Vu/R;
Vg = 1; % carrier peak value
dI = 0.1 * Iu; % Current ripple
dV = 0.1 * Vu; % voltage ripple

% Filter sizing
L=1e-3;
RL=0.007;
C=100e-6;

% Dynamics
wn = 1/sqrt(L*C);
csi = 1/(2*R)*sqrt(L/C);
wr = wn * sqrt(1 - csi^2);

% Plant
Num = 1;
Den = [L*C, L/R, 1];
Sys = tf(Num, Den);

% Single-loop voltage controller
fc_target = 100; % desired crossover frequency
wc = 2*pi*fc_target;
Taui1 = R*C / 10;
kpi1 = wc * Taui1;

Num = [kpi1 * Taui1, kpi1];
Den = [Taui1, 0];
R1 = tf(Num, Den);

G1 = R1 * Sys;
W1 = feedback(G1, 1);

% Bandwidth Check
[GM, PM, wc_meas] = margin(G1);
f_bw = wc_meas / (2 * pi);
fprintf(">>> Controller Bandwidth: %.2f Hz\n", f_bw);
if f_bw >= 100
    fprintf("✓ Bandwidth meets requirement (≥ 100 Hz)\n");
else
    fprintf("✗ Bandwidth is TOO LOW (< 100 Hz)\n");
end

% Step and Bode plots
figure; bode(W1); grid on; title('Voltage Loop - Closed Loop');
figure; step(W1); title('Voltage Loop Step Response');

% Load Variation Analysis
R_full = 2.3;
T_full = feedback(R1 * tf(R_full, [R_full * C 1]), 1);

R_noload = 1e8;
T_noload = feedback(R1 * tf(R_noload, [R_noload * C 1]), 1);

figure;
step(T_full, T_noload, 0.5);
legend('Full Load (R=2.3Ω)', 'No Load (R=∞)');
title('Step Response Comparison - Full vs No Load');
xlabel('Time (s)');
ylabel('Output Voltage (V)');
grid on;

figure;
bode(T_full, T_noload);
legend('Full Load', 'No Load');
title('Bode Plot Comparison - Full vs No Load');
grid on;
