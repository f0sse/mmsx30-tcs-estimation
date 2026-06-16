function [meas, bias] = GyroNoise_modified(truth, Fs)

%% =========================
%% FORMAT FIX
%% =========================
if size(truth,2) ~= 3
    truth = truth';
end

n = size(truth,1);

%% =========================
%% SENSOR PARAMETERS
%% =========================
biasScale = 1;

Cheap = [0.4055   0.3387   0.3830;   % ARW  deg/sqrt(h)
         4.40e-04 3.20e-04 4.05e-04; % RRW  deg/s^2
         0.0072   0.0030   0.0029];  % BI   deg/s

% === KONVERTERA TILL SI ===
deg2rad = pi/180;

ARW = Cheap(1,:) * deg2rad / sqrt(3600);   % rad/sqrt(s)
RRW = Cheap(2,:) * deg2rad;               % rad/s^2
BI  = Cheap(3,:) * deg2rad * biasScale;   % rad/s

dt = 1/Fs;

%% =========================
%% PERSISTENT STATES
%% =========================
persistent bias_rw bias_bi

if isempty(bias_rw)
    bias_rw = zeros(1,3);
    bias_bi = zeros(1,3);
end

%% =========================
%% NOISE MODEL
%% =========================
meas = zeros(n,3);
bias = zeros(n,3);

for k = 1:n

    % === Random Walk (angle random walk → bias drift) ===
    bias_rw = bias_rw + sqrt(dt) * ARW .* randn(1,3);

    % === Bias instability (Gauss-Markov) ===
    tau = 50; % <-- VIKTIG! (tuna 10–200 s)
    bias_bi = bias_bi + dt*(-bias_bi/tau) + sqrt(dt)*BI.*randn(1,3);

    % === Rate noise (white noise på gyro output) ===
    white = RRW .* randn(1,3);

    % === OUTPUT ===
    meas(k,:) = truth(k,:) + bias_rw + bias_bi + white;
    bias(k,:) = bias_rw;

end

end