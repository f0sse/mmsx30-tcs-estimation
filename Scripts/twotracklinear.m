%%%
%%% System Stability Analysis
%%%

clear variables;
close all;
clc;

%%%%%%%%%%%%%%%%%%%
%% Process Model %%
%%%%%%%%%%%%%%%%%%%

syms m_car Jz lf lr track R_w CoM_x Ts
syms delta_FL delta_FR Fx_FL Fx_FR Fx_RL Fx_RR
syms vx vy r Cf_FL Cf_FR Cr_RL Cr_RR

R_FL = [cos(delta_FL), sin(delta_FL); -sin(delta_FL), cos(delta_FL)];
R_FR = [cos(delta_FR), sin(delta_FR); -sin(delta_FR), cos(delta_FR)];

rFL = [lf - CoM_x;  track/2; 0];
rFR = [lf - CoM_x; -track/2; 0];
rRL = [-lr - CoM_x; track/2; 0];
rRR = [-lr - CoM_x; -track/2; 0];

Vc = [vx; vy; 0];
omega = [0; 0; r];

vFL = Vc + cross(omega, rFL);
vFR = Vc + cross(omega, rFR);
vRL = Vc + cross(omega, rRL);
vRR = Vc + cross(omega, rRR);

V_FL_l = R_FL * vFL(1:2);
V_FR_l = R_FR * vFR(1:2);
V_RL_l = vRL(1:2);
V_RR_l = vRR(1:2);

alpha_FL = atan(V_FL_l(2) / V_FL_l(1));
alpha_FR = atan(V_FR_l(2) / V_FR_l(1));
alpha_RL = atan(V_RL_l(2) / V_RL_l(1));
alpha_RR = atan(V_RR_l(2) / V_RR_l(1));

Fy_FL = -Cf_FL * alpha_FL;
Fy_FR = -Cf_FR * alpha_FR;
Fy_RL = -Cr_RL * alpha_RL;
Fy_RR = -Cr_RR * alpha_RR;

F_FL = R_FL.' * [Fx_FL; Fy_FL];
F_FR = R_FR.' * [Fx_FR; Fy_FR];

Fx_sum = F_FL(1) + F_FR(1) + Fx_RL + Fx_RR;
Fy_sum = F_FL(2) + F_FR(2) + Fy_RL + Fy_RR;

dvx = Fx_sum / m_car + vy*r;
dvy = Fy_sum / m_car - vx*r;

Mz = rFL(1)*F_FL(2) + rFR(1)*F_FR(2) + rRL(1)*Fy_RL + rRR(1)*Fy_RR ...
   - rFL(2)*F_FL(1) - rFR(2)*F_FR(1) - rRL(2)*Fx_RL - rRR(2)*Fx_RR;
dr = Mz / Jz;

x  = [vx; vy; r; Cf_FL; Cf_FR; Cr_RL; Cr_RR];
dx = [dvx; dvy; dr; 0; 0; 0; 0];

F = simplify(x + dx*Ts);
J = simplify(jacobian(F, x));

A = eye(size(J)) + J;

clearvars -except A

%%%%%%%%%%%%%%%%%%%%%%%
%% Measurement Model %%
%%%%%%%%%%%%%%%%%%%%%%%

syms m_car Jz lf lr track R_w CoM_x
syms delta_FL delta_FR Fx_FL Fx_FR Fx_RL Fx_RR
syms vx vy r Cf_FL Cf_FR Cr_RL Cr_RR

R_FL = [cos(delta_FL), sin(delta_FL); -sin(delta_FL), cos(delta_FL)];
R_FR = [cos(delta_FR), sin(delta_FR); -sin(delta_FR), cos(delta_FR)];

rFL = [lf - CoM_x;  track/2; 0];
rFR = [lf - CoM_x; -track/2; 0];
rRL = [-lr - CoM_x; track/2; 0];
rRR = [-lr - CoM_x; -track/2; 0];

Vc = [vx; vy; 0];
omega = [0; 0; r];

vFL = Vc + cross(omega, rFL);
vFR = Vc + cross(omega, rFR);
vRL = Vc + cross(omega, rRL);
vRR = Vc + cross(omega, rRR);

V_FL_l = R_FL * vFL(1:2);
V_FR_l = R_FR * vFR(1:2);
V_RL_l = vRL(1:2);
V_RR_l = vRR(1:2);

alpha_FL = atan(V_FL_l(2) / V_FL_l(1));
alpha_FR = atan(V_FR_l(2) / V_FR_l(1));
alpha_RL = atan(V_RL_l(2) / V_RL_l(1));
alpha_RR = atan(V_RR_l(2) / V_RR_l(1));

Fy_FL = -Cf_FL * alpha_FL;
Fy_FR = -Cf_FR * alpha_FR;
Fy_RL = -Cr_RL * alpha_RL;
Fy_RR = -Cr_RR * alpha_RR;

F_FL = R_FL.' * [Fx_FL; Fy_FL];
F_FR = R_FR.' * [Fx_FR; Fy_FR];

Fx_sum = F_FL(1) + F_FR(1) + Fx_RL + Fx_RR;
Fy_sum = F_FL(2) + F_FR(2) + Fy_RL + Fy_RR;

ax = Fx_sum / m_car;
ay = Fy_sum / m_car;

w_FL = V_FL_l(1) / R_w;
w_FR = V_FR_l(1) / R_w;
w_RL = V_RL_l(1) / R_w;
w_RR = V_RR_l(1) / R_w;

x = [vx; vy; r; Cf_FL; Cf_FR; Cr_RL; Cr_RR];
H = simplify([ax; ay; vx; vy; r; w_FL; w_FR; w_RL; w_RR]);
J = simplify(jacobian(H, x));

C = J;

clearvars -except A C

%%%%%%%%%%%%%%
%% Analysis %%
%%%%%%%%%%%%%%

syms m_car Jz lf lr track R_w CoM_x Ts
syms delta_FL delta_FR Fx_FL Fx_FR Fx_RL Fx_RR
syms vx vy r Cf_FL Cf_FR Cr_RL Cr_RR

varsA = [Cf_FL, Cf_FR, CoM_x, Cr_RL, Cr_RR, Fx_FL, Fx_FR, Fx_RL, Fx_RR, Jz, R_w, Ts, delta_FL, delta_FR, lf, lr, m_car, r, track, vx, vy];
varsC = [Cf_FL, Cf_FR, CoM_x, Cr_RL, Cr_RR, Fx_FL, Fx_FR, Fx_RL, Fx_RR, Jz, R_w, delta_FL, delta_FR, lf, lr, m_car, r, track, vx, vy];

matlabFunction(A, "File", "Afun", "Vars", varsA);
matlabFunction(C, "File", "Cfun", "Vars", varsC);

codegen Afun -args num2cell(zeros(1, length(varsA))) -config:mex
codegen Cfun -args num2cell(zeros(1, length(varsC))) -config:mex

valA = {4, 4, 3.6, 4, 4, 100, 100, 100, 100, 14232.357, 0.49, 0.01, 0, 0, 4.52, 0.95, 7000, 0.01, 2.10, 15, 0};
valC = {4, 4, 3.6, 4, 4, 100, 100, 100, 100, 14232.357, 0.49, 0, 0, 4.52, 0.95, 7000, 0.01, 2.10, 15, 0};

Anum = Afun(valA{:});
Cnum = Cfun(valC{:});

fprintf("Steady state condition number = %.4f\n", cond(Anum))

O = [
    Cnum;
    Cnum * Anum;
    Cnum * Anum^2;
    Cnum * Anum^3;
    Cnum * Anum^4;
    Cnum * Anum^5;
    Cnum * Anum^6;
];

fprintf("Rank of observability matrix = %d\n", rank(O))
