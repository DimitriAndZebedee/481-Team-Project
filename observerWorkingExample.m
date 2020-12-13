clc;
clear;
%% Parameters
mc = 1.5;
mp = 0.5;
g = 9.82;
L = 1;
d1 = 1e-2;
d2 = 1e-2;
%% define matrix
A = [0, 0, 1, 0; 
    0, 0, 0, 1;
    0, (g*mp)/mc, -d1/mc, -d2/(L*mc);
    0, g*(mc + mp)/(L*mc), -d1/(L*mc), -(d2*mc + d2*mp)/(L^2*mc*mp)];

B = [0; 0; 1/mc; 1/(L*mc)];

%% Output
%C = [0;1;0;0]; %controllable not observable
C = [1;0;0;0]; %observable and controllable
D = 0;

x0 = [0;5*pi/180;0;0];

%% Build the system

sys = ss(A,B,C',D);
eig(A);% checks stability, if all values are negative it is stable

%is the system observable and controllable?
Sc = ctrb(sys);
So = obsv(sys);
rank(Sc);
rank(So);

%just having a look at the poles and zeros
%rlocus(sys);

%%  controller
des_poles = [-3; -3; -3; -3]*0.5;
K = acker(A,B,des_poles);

Q = eye(4);
R = 1;
K_lqr = lqr(A,B,Q,R);

%% Discrete time
Ts = 0.1;
sys_d = c2d(sys, Ts);

Ad = sys_d.a;
Bd = sys_d.b;
Cd = sys_d.c;
Dd = sys_d.d;

des_poles_d = [0.3; 0.3; 0.3; 0.3];
K_d = acker(Ad,Bd,des_poles_d);

%% Observer
des_poles_d = [0.3; 0.3; 0.3; 0.3];
Ob = acker(Ad',Cd',des_poles_d);





