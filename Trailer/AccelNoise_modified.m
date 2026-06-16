function [meas, bias] = AccelNoise_modified(truth, Fs, ~)

%% =========================
%% FORMAT FIX
%% =========================
% Säkerställ [N x 3]
if size(truth,2) ~= 3
    truth = truth';
end

n = size(truth,1);

%% =========================
%% PARAMETERS
%% =========================
biasScale = 1;

Cheap = ...
    [0.0311    0.0319     0.0409     % ARW  m/s/sqrt(h)
     4.94e-05  2.85e-05   5.10e-05   % RRW
     2.66e-04  2.02e-04   3.61e-04] .* ...
    [1;biasScale;biasScale];

ARW = Cheap(1,:);
RRW = Cheap(2,:);
BI  = Cheap(3,:);

dt = 1/Fs;

%% =========================
%% PERSISTENT STATES (VIKTIGT)
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

    % === Random Walk (bias drift) ===
    bias_rw = bias_rw + sqrt(dt) * ARW .* randn(1,3);

    % === Bias instability (1st order GM) ===
    tau = 1; % correlation time (kan tunas!)
    bias_bi = bias_bi + dt*(-bias_bi/tau) + sqrt(dt)*BI.*randn(1,3);

    % === White noise ===
    white = RRW .* randn(1,3);

    % === Output ===
    meas(k,:) = truth(k,:) + bias_rw + bias_bi + white;
    bias(k,:) = bias_rw;

end

end