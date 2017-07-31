% Get data for drift estimation (keep quad exactly level)
Serial2(20000, 'imustationary.mat');

% Adynamics is the "A" in Ax + Bu
[ ADynamics, B, C, R, Q, P, K, X] = EstimateKalmanFilter(Atrunc', 930);

load imustationary.mat;
[kf_sys, P, ~] = EstimateKalmanFilter(Atrunc', 930);

load imudatanonstationary.mat;
GetKalmanEstimate(kf_sys, P, A', zeros(3,1));

% Find mean drift
[Vmeans, Dmeans] = DEAnalysis.analyzeRotatedDrift('imudata.mat', 952, 220);

% Get live data.
filename = 'imuhorizmove10cm.mat';
Serial2(10000, filename);

% Find adjusted velocity/position
[Vorig, Dorig, Vrot, Drot, Vest, Dest] = DEAnalysis.adjustWithRotatedMeanDrift(filename, 952, 'Drift Adjustment', 240, Vmeans, Dmeans);

%%
clear variables;
% Measurement constants
fs = 930/5; secs = 100;
N = ceil(fs * secs);
baud = 921600;
% Get data
SerialComState(secs, 16, fs, 'COM4', baud, 'imusample.mat', false, false).run();
%%
load imusample.mat;

% LPF filter specs
cutoff = 10; ntaps = 200;

% Rotate acceleration to earth frame
Arot = quatrotate(quatinv(Q'), A');
% Remove low pass content
Arotf = FIRUtils.runLowPassXYZ(Arot, cutoff, fs, ntaps, false);

% Run Kalman Filter and plot
%%% Choice of initial state impacts how many secods of data is required for convergence
%%% -   During operation, the initial state is close to the actual state, so the
%%%     convergence time is not high.
zAcc = Arot(:,3);
%initialState = [0, 0, 0, plot0]';
Zs = [Altitude', zAcc]; [kf_sys, P, X, K] = EstimateWithKalman.AltBiasFromAccBar(Zs, fs, []);
 subplot(4,2,8), plot(FAltitude), title('Chip Altitude');

%%
% Run Kalman Filter and plot
%%% Choice of initial state impacts how many secods of data is required for convergence
%%% -   During operation, the initial state is close to the actual state, so the
%%%     convergence time is not high.

%initialState = [0, 0, 0, 0]';
Zs = [X(:,1), X(:,3)]; [kf_sys, P, X] = EstimateWithKalman.AltBiasFromAccBar(Zs, fs, []);


%%
movMeanWindow = 5; % ceil(N/20)
figure(133), plot(movingmean(X(ceil(N/9):N,1), movMeanWindow)), set(gcf, 'name', 'Position over time', 'numbertitle','off');
sprintf('Variance in position: %f', var(X(ceil(N/9):N,1)))
sprintf('Mean position: %f', mean(X(ceil(N/9):N,1)))


%%
figure(2001)
h = plot([-1, 0, 1], [NaN, 0, NaN]);
xlim([-1,1]);
ylim([7,9]);
h.set('Marker', 'o');
for i = 1:2:size(X(:,1), 1)    
    h.set('YData', [NaN, X(i,1), NaN]);   
    drawnow limitrate;
    pause(.001);
end

%%
syms T;
assume (T, 'positive'), assume(T, 'real');

syms A B F Q C D G R
A = [1, T, 0.5*T^2, 0; 0, 1, T, 0; 0, 0, 1, 0; 0, 0, 0, 1];
P = eye(4);
syms Q_std Q_bias;
assume([Q_std Q_bias], 'real');
assume([Q_std Q_bias], 'positive');
Q = [Q_std, 0, 0, 0; 0, Q_std, 0, 0; 0, 0, Q_std, 0; 0, 0, 0, Q_bias];
C = [1, 0, 0, 0; 0, 0, 1, 1];

syms R_bar R_acc R_bar_acc;
assume([R_bar R_acc], 'real');
assume([R_bar R_acc], 'positive');
assume(R_bar_acc, 'real');

R = [R_bar, R_bar_acc; R_bar_acc, R_acc];

kf_sys = EstimateWithKalman.KFSYS_A_B_F_Q_C_D_G_R(A, 0, 1, Q, C, 0, 1, R);

syms p v a bias state;
assume([p v a bias], 'real');
assume(bias, 'positive');
state = [p v a bias]';

syms z_p z_a;
assume([z_p z_a], 'real');
assume([z_p z_a], 'positive');

[state, P] = KF(kf_sys, [z_p, z_a]', 0, state, P);

state % Print state as symbolic data
P       % Print P as symbolic data

%%
                
