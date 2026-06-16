%%%
%%% System Stability Analysis
%%%

clear variables;
close all;
clc;

%%%%%%%%%%%%%%%%%%%
%% Process Model %%
%%%%%%%%%%%%%%%%%%%

syms m g lf lr delta h Trl Trr Rw L Cf Cr Jz Ts

R = [ cos(delta), -sin(delta), 0;
      sin(delta),  cos(delta), 0;
               0,           0, 1;
    ];

vx = sym("vx");
vy = sym("vy");
wz = sym("wz");

V = [ vx; vy;  0 ];
W = [  0;  0; wz ];

Rfax = [ +lf; 0; -h ];
Rrax = [ -lr; 0; -h ];

Vf = R.' * (V + cross(W,Rfax));
Vr = V + cross(W,Rrax);

alpha_f = atan(Vf(2,1)/Vf(1,1));
alpha_r = atan(Vr(2,1)/Vr(1,1));

Fx = (Trl + Trr)/Rw;
ax = Fx/m + vy*wz;

Ffz = m*g*lr/L - m*ax*h/L;
Frz = m*g*lf/L + m*ax*h/L;

Ffy = -Cf*alpha_f*Ffz;
Fry = -Cr*alpha_r*Frz;

dvx = (Fx - Ffy*sin(delta))/m + vy*wz;
dvy = (Ffy*cos(delta) + Fry)/m - vy*wz;
dwz = (lf*Ffy*cos(delta) - lr*Fry)/Jz;

x  = [  vx;  vy;  wz; Cf; Cr ];
dx = [ dvx; dvy; dwz;  0;  0 ];

F = simplify(x + dx*Ts);
J = simplify(jacobian(F, x));

% @(Cf,Cr,Jz,L,Rw,Trl,Trr,Ts,delta,g,h,lf,lr,m,vx,vy,wz)
% A(10,10,12994.917,3.57,0.5,650,650,0.01,0.017,9.82,0.925,1.047,2.523,6800,11,0.4,0.27)
A = eye(size(J)) + J;

clearvars -except A

%%%%%%%%%%%%%%%%%%%%%%%
%% Measurement Model %%
%%%%%%%%%%%%%%%%%%%%%%%

syms m g lf lr delta h Trl Trr Rw L Cf Cr Jz Ts

R = [ cos(delta), -sin(delta), 0;
      sin(delta),  cos(delta), 0;
               0,           0, 1;
    ];

vx = sym("vx");
vy = sym("vy");
wz = sym("wz");

V = [ vx; vy;  0 ];
W = [  0;  0; wz ];

Rfax = [ +lf; 0; -h ];
Rrax = [ -lr; 0; -h ];
Rimu = [ .127; 0; .075 ];

Vf = R.' * (V + cross(W,Rfax));
Vr = V + cross(W,Rrax);

alpha_f = atan(Vf(2,1)/Vf(1,1));
alpha_r = atan(Vr(2,1)/Vr(1,1));

Fx = (Trl + Trr)/Rw;
ax = Fx/m + vy*wz;

Ffz = m*g*lr/L - m*ax*h/L;
Frz = m*g*lf/L + m*ax*h/L;

Ffy = -Cf*alpha_f*Ffz;
Fry = -Cr*alpha_r*Frz;

dvx = (Fx - Ffy*sin(delta))/m;
dvy = (Ffy*cos(delta) + Fry)/m;
dwz = (lf*Ffy*cos(delta) - lr*Fry)/Jz;

x  = [  vx;  vy;  wz; Cf; Cr ];
dx = [ dvx; dvy; dwz;  0;  0 ];

Acog = [ dvx; dvy;   0 ];
dWdt = [   0;   0; dwz ];

Aimu = Acog + cross(W, cross(W,Rimu)) + cross(dWdt,Rimu);
Vimu = V + cross(W,Rimu);

omega_rl = vx / Rw;
omega_rr = vx / Rw;

%H = simplify([ Aimu(1:2); Vimu(1:2); W(3); omega_rl; omega_rr ]);
H = simplify([ Aimu(1:2); 0; 0; W(3); omega_rl; omega_rr ]);
J = simplify(jacobian(H, x));

% @(Cf,Cr,Jz,L,Rw,Trl,Trr,delta,g,h,lf,lr,m,vx,vy,wz)
% C(10,10,12994.917,3.57,0.5,650,650,0.017,9.82,0.925,1.047,2.523,6800,11,0.4,0.27)
C = J;

clearvars -except A C

%%%%%%%%%%%%%%
%% Analysis %%
%%%%%%%%%%%%%%

matlabFunction(A, "File", "Afun");
matlabFunction(C, "File", "Cfun");

codegen Afun -args {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} -config:mex
codegen Cfun -args {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}   -config:mex

Anum = Afun_mex(10,10,12994.917,3.57,0.5,650,650,0.01,0.017,9.82,0.925,1.047,2.523,6800,11,0.4,0.27);
Cnum = Cfun_mex(10,10,12994.917,3.57,0.5,650,650,0.017,9.82,0.925,1.047,2.523,6800,11,0.4,0.27);

fprintf("Steady state condition number = %.4f\n", cond(Anum))

O = [
    Cnum;
    Cnum * Anum;
    Cnum * Anum^2;
    Cnum * Anum^3;
    Cnum * Anum^4;
];

fprintf("Rank of observability matrix = %d\n", rank(O))
