function params = prep_params_BicyclMdl(vehicle, pp)
% Parameters for One Track (Bicycle) model – MUSE compatible

if nargin < 2
       pp=struct(); % p.cLat = 1; p.cLon = 1;
end
g = 9.82;
params.g = g;

% --- Use ONLY tractor body ---
body = vehicle.bodies(1);

% --- Mass & inertia ---
params.m  = body.dryMass;          % scalar
params.Jz = body.inertia(3);        % yaw inertia

% --- Geometry (One Track) ---
% axlePositions: [front, rear, ...]
a = abs(body.axlePositions(1));     % front axle distance
b = abs(body.axlePositions(3));     % rear axle distance

params.a = a;
params.b = b;
params.L = a + b;
params.Rw =body.tireRadius(3);    % Tire Radius of the 2 rear tire 
params.CoGPos = body.cogPosition;
params.SenPos=vehicle.transforms.Sensor.Positions(:,1); % sensor position of tractor
params.r= params.CoGPos - params.SenPos; % Translation vector from sensor to CoG (here we have defined our states)

% --- Tire cornering stiffness (per axle, NOT per wheel) ---
% Typical values – adjust later
%params.Cf = 1.2e5;   % front axle
%params.Cr = 1.8e5;   % rear axle

% --- Rolling resistance & drag ---
 % nothing to be add here yet since we assume the simplest case  

% --- Process noise (UKF expects sqrt(Q)) ---
params.q.m     = params.m  * 0.05;
params.q.Jz    = params.Jz * 0.05;
params.q.a     = 0.02;
params.q.b     = 0.02;
params.q.L     = 0.02;
params.q.CoGPos = 0.01/3*ones(3,1);
params.q.Rw = 0;
params.q.SenPos=[0;0;0];
params.q.r=0.01/3*ones(3,1);
%params.q.Cf = params.Cf * 0.1;exit
%params.q.Cr = params.Cr * 0.1;

% Convert to sqrt(Q)
fns = fieldnames(params.q);
for i = 1:length(fns)
    params.q.(fns{i}) = sqrt(params.q.(fns{i}))*0;
end

end
