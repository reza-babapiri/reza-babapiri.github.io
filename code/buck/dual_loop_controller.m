%%*********************************Reza Babapiri***************************************
clear all; clc; close all;

%% === System Setup ===
Ea = 100; % nominal voltage[v]
R = 2.3;  % load resistor[ohm]
fs = 50e3; %[Hz]
Vu = 48;  % desired output voltage[v]
T = 1/fs;
dutycycle = Vu/Ea;
Tau = dutycycle * T;
Iu = Vu/R;
Vg = 1; % carrier peak value
dI = 0.1 * Iu;
dV = 0.1 * Vu;

%% === Filter Sizing ===
L = 1e-3;
RL = 0.007;
C = 100e-6;

%% === Plant Dynamics ===
wn = 1/sqrt(L*C);
csi = 1/(2*R)*sqrt(L/C);
wr = wn * sqrt(1 - csi^2);

Num = 1;
Den = [L*C, L/R, 1];
Sys = tf(Num, Den);

%% === Inner Current Loop ===
Taui2 = L / RL;
Tau2 = 10 / (2 * pi * fs);
k = L / Tau2;
kpi2 = (Vg * k) / (Tau2 * Ea);

Num = [kpi2 * Taui2, kpi2];
Den = [Taui2, 0];
R2 = tf(Num, Den);

G2 = R2 * Ea/Vg * tf(1, [L RL]); 
W2 = feedback(G2, 1); % Use exact current loop

%% === Outer Voltage Loop ===
fci = 1 / (2 * pi * Tau2);
fcv = fci / 20;
Wgc = 2 * pi * fcv;

Taui1 = C * R / 5;  % Less aggressive integral time constant
kpi1 = Wgc * Taui1; % Simpler PI tuning to avoid instability

Num = [kpi1 * Taui1, kpi1];
Den = [Taui1, 0];
R1 = tf(Num, Den);

Sys1 = tf(R, [R*C 1]);
G1 = Sys1 * R1 * W2; % Use W2 instead of W2_approx
W1 = feedback(G1, 1);

%% === Bandwidth Check ===
[GM, PM, wc] = margin(G1);
f_bw = wc / (2 * pi);
fprintf(">>> Controller Bandwidth: %.2f Hz\n", f_bw);
if f_bw >= 100
    fprintf("✓ Bandwidth meets requirement (≥ 100 Hz)\n");
else
    fprintf("✗ Bandwidth is TOO LOW (< 100 Hz)\n");
end

%% === Load Variation Analysis ===
R_full = 2.3;
T_full = feedback(R1 * tf(R_full, [R_full * C 1]) * W2, 1);

R_noload = 1e8;
T_noload = feedback(R1 * tf(R_noload, [R_noload * C 1]) * W2, 1);

%% === Plots ===
figure; step(T_full, T_noload, 0.5);
legend('Full Load (R=2.3Ω)', 'No Load (R=∞)');
title('Step Response Comparison - Full vs No Load');
xlabel('Time (s)'); ylabel('Output Voltage (V)'); grid on;

figure; bode(T_full, T_noload);
legend('Full Load', 'No Load'); title('Bode Plot Comparison - Full vs No Load'); grid on;

figure; bode(W1, W2); legend('Voltage Loop', 'Current Loop'); grid on;
figure; step(W1, W2); legend('Voltage Loop', 'Current Loop'); grid on;
figure; bode(W2); title('Current Loop Response'); grid on;
