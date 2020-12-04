clc;
clear;

%1.State Space Equations
A=[0 1 0 0 ;-491 0 -282.45 0; 0 0 0 1;1488 0 885.65 0];
B=[0;82.33;0;-235.29];
C=[1 0 0 0;0 0 1 0];
D=[0;0];
sys_pend= ss(A,B,C,D);

%2.Transferfunction of open loopsystem
H = tf(sys_pend); % gives the two transfer functions for position and theta 
% w.r.t input force
pos_tf = tf([82.33 0 -6458],[1 0 -394.7 0 -14570]);
theta_tf = tf([-235.3 0 6980],[1 0 -394.7 0 -14570]);
pos_tf =H(1);
theta_tf=H(2);

%3.Canonical forms - controllable, observable, jordan
%3.1 Contrabilitytest
Cx = [B A*B A*A*B A*A*A*B];
rank(Cx);

% 3.2 Observability test
Ox = [C;C*A;C*A*A;C*A*A*A];
rank(Ox);

%3.3 Contrallablecanonicalform
[p] = poly(A); % returns the characteristic polynomial p = [a4 a3 a2 a1 a0]
Ac = [0 1 0 0; 0 0 1 0;0 0 0 1;14569 0 395 0];
Bc = [0;0;0;1];
Cxc = [Bc Ac*Bc Ac*Ac*Bc Ac*Ac*Ac*Bc];
Tc = Cx*(inv(Cxc));
Cc = C*Tc;
Dc = D;

% 3.4 Observable canonical form
Ao = Ac';
Bo = Cc';
Co = Bc';
Do = D;

% 3.5 Jordan canonical form
[M,R] = eig(A);
Aj = jordan(A);
Bj = M\B;
Cj = C*M;
Dj = D;

%4.Open loop systemresponses
figure
impulse(sys_pend);grid
figure
step(sys_pend);grid

% 5.Bode plot and root locus
figure
bode(pos_tf,'r:');
figure
rlocus(pos_tf,'r:');
figure
bode(theta_tf,'r:');
figure
rlocus(theta_tf,'r:');
% signal generators - sine, square
[Usq,Tsq] = gensig('square',2,10,0.01);
[Usn,Tsn] = gensig('sin',2,10,0.01);

%6. PID controllers
%6.1 Position control
Kd = 0.55;Kp =4000; Ki = 10;
contr1=tf([Kd Kp Ki],[1 0]);
loop_contr1 = pos_tf * contr1;
sys_pid_pos= feedback(loop_contr1,1);
% Position control - Step response
figure
step(sys_pid_pos);grid
% Position control - Square wave input
[Y,T]=lsim(sys_pid_pos,Usq,Tsq);
figure
plot(T,Y);grid
legend('Square response for position control')
% Position control - Sine wave input
[Y,T]=lsim(sys_pid_pos,Usn,Tsn);
figure
plot(T,Y);grid
legend('Sine response for position control')

%6.2 Angular control
Kp = -1000;
Kd = -0.27;
Ki = -100;
contr2=tf([Kd Kp Ki],[1 0]);
loop_contr2 = contr2 * theta_tf;
sys_pid_theta= feedback(loop_contr2,1);
% Angular control - Step response
figure
step(sys_pid_theta);grid
% Angular control - Square wave input
[Y,T]=lsim(sys_pid_theta,Usq,Tsq);
figure
plot(T,Y); grid
legend('Square response for angle control')

% Angular control - Sine wave input
[Y,T]=lsim(sys_pid_theta,Usn,Tsn);
figure
plot(T,Y); grid
legend('Sine response for angle control')

%9. PID controllersrobutness
figure
rlocus(sys_pid_pos);
figure
rlocus(sys_pid_theta);

%addstepdisturbancetochecktherobustness
%ys/ds = gs/(1+gs.cs)
loop_distr1 = 1 + loop_contr1;
resp1 = pos_tf / loop_distr1;
figure
step(resp1)
loop_distr2 = 1 + loop_contr2;
resp2 = theta_tf / loop_distr2;
figure
step(resp2)

% 10. State feedback control with transient characterstics
% damping ratio 0.707,Wn = 5.657, time constant = 0.25 sec,
Pole_fb = [-4+4i -4-4i -12 -16];
K = place(A,B,Pole_fb);
Af = A-B*K;
Nbar = K(1); % reference to remove SS error
sys_fdbck_pend= ss(Af,B*Nbar,C,D);
tf_sys_fdbck_pend =tf(sys_fdbck_pend);

% figure
step(sys_fdbck_pend,5);grid
legend('Step response for State feedback control');
%
[Y,T,X]=lsim(sys_fdbck_pend,Usq,Tsq);
figure
plot(T,Y);grid
title('Square response for State feedback control')
legend('Pos','Theta');
[Y,T,X]=lsim(sys_fdbck_pend,Usn,Tsn);
figure
plot(T,Y);grid
title('Sine response for State feedback control')
legend('Pos','Theta');

% 11. Full order observer
% damping ratio 0.707,Wn = 22.63, time constant = 0.083 sec
%(~2.5 times faster)
Pole_f_obs=[-10+10i -10-10i -30 -40];

G = place(A',C',Pole_f_obs)';
%Controller-estimator state equationsandtranferfunction
Axe =A-G*C-B*K;
Bxe =G;
Cxe =-K;
Dxe=[0 0];
sys_obs_pend = ss(Axe,Bxe,Cxe,Dxe);
tf_sys_obs_pend = tf(sys_obs_pend);
figure
step(sys_obs_pend,5);grid

% Closed loop system response with Full order Observer - Controller
Ace = [A-B*K B*K;
zeros(size(A)) (A-G*C)];
Bce = [B*Nbar;
zeros(size(B))];
Cce = [C zeros(size(C))];
Dce = [0;0];
sys_full_obs_cl_pend = ss(Ace,Bce,Cce,Dce);
tf_sys_full_obs_cl_pend = tf(sys_full_obs_cl_pend);
figure
step(sys_full_obs_cl_pend,5);grid
[Y,T,X]=lsim(sys_full_obs_cl_pend,Usn,Tsn);
figure
plot(T,Y);grid
title('Sine response for Closed loop Full order Observer - Controller')
legend('Pos','Theta');

% Reduced order observer
% damping ratio 0.707,Wn = 22.63, time constant = 0.083 sec
%(~2.5 times faster)
Ge = [15 -5; 25 5];% reduced order estimator gain matrix
A11 = [0 0;0 0];
A1e = [1 0;0 1];
Ae1 = [-491 -282.35; 1488 885.65];
Aee = [0 0; 0 0];
B1 =[0;0]; Be =[82.33;-235.29];
K1 =[K(1) K(3)]; Ke =[K(2) K(4)];
Arce = Aee - (Ge*A1e) - (Be*Ke) + (Ge*B1*Ke);
Brce = Ae1 - (Ge*A11) + (Aee*Ge) - (Ge*A1e*Ge) - (Be*K1) + (Ge*B1*K1) -...
(Be*Ke*Ge) + (Ge*B1*Ke*Ge);
Crce = -Ke;
Drce = -K1 - (Ke*Ge);
sys_red_obs_pend = ss(Arce,Brce,Crce,Drce);
tf_sys_red_obs_pend = tf(sys_red_obs_pend);
figure
step(sys_red_obs_pend,5);grid

