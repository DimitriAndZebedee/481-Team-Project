clc;
clear;

%% Properties
%Coefficient values given in Appendix A.1.2.i, default values are chosen

gr = 4;
Jd = 0.0025;
J1 = 0.0271;
k = 8.45;
c1 = 0.004;
c2 = 0.05;

%% Question 1 Statspace representation
%State space represesentation of Industrial Emulator based on Page 65 of
%Lab Manual

%1)State space

A = [ 0 1 0 0;
    (-k*(gr)^(-2))/Jd  -c1/Jd  (k*(gr)^(-1))/Jd  0;
      0 0 0 1;
     (k*(gr)^(-1))/J1  0 -k/J1 -c2/J1];


B = [0; 1/Jd; 0; 0];

%C matrix with C2 set to 1 for system observability
%commented out to for testing
C = [0,0,0,0;
     0,1,0,0; 
     0,0,0,0;
     0,0,0,0];

D = [0;     0;     0;     0];

plant = ss(A, B, C, D);

%Checks to see if the system is observable, if rank 4, it is
observable = rank(obsv(plant));
%Checks to see if the system is controllable, if rank 4, it is
controlable = rank(ctrb(plant));

%% Question 2 Transfer function obtained from state-space representation
[a,b] = ss2tf(A,B,C,D);

%a is given as array since state space is MIMO by default (although only
%zeros for rows 1,3 and 4 --> removing zero rows to form 5X1 matrix
a(1,:)=[];
a(2,:)=[];
a(2,:)=[];

T =tf(a,b);

%% Question 3 Canonical forms
% 3.1 Controllability matrix
ControllabilityMatrix = ctrb(A, B);
charPoly = poly(A);
A_ctrl = [0 1 0 0 0; 0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1;
    -charPoly(1) -charPoly(2) -charPoly(3) -charPoly(4) -charPoly(5)]; 

% 3.2 Observability matrix
Co = [0 0 0 0 1];
Ao = [0 0 0 0 -1;
      1 0 0 0 -3.4450;
      0 1 0 0 -526.0101;
      0 0 1 0 -888.6531;
      0 0 0 1 -6.5*10^(-12)];
ObservabilityMatrix = obsv(Ao, Co);

% 3.3 Jordan Matrix
Aj = jordan(A);
Je = eig(A);
[M,R] = eig(A);
C1 = [0 1 0 0];
Cj = C1*M;
%Cj = Je';

%% Question 4 Reponse of the system
% 4.1 Impulse response
sys = ss(A, B, C, D);
impulseResponse = impulse(sys);
ti = 1:length(impulseResponse);
% Plot Impusle Response
%uncomment next, to get impulse plot
%plot(ti, impulseResponse(:,2));

%4.2 Step reponse
stepResponse = step(sys);
ts = 1:length(stepResponse);
% plot the step response
%uncomment next, to get step plot
%plot(ts, stepResponse);

%4.2 Step reponse for the transfer function
stepResponseT = step(T);
tst = 1:length(stepResponseT);
% plot the step response
%uncomment next, to get step plot
%plot(tst, stepResponseT);

% 5.1 Bode plot
%uncomment next, to get bode plot
%bode(T);

% 5.2 Root Locus
%uncomment next, to get Root Locus plot
%rlocus(T);

%% Question 6 PID Controller
Kp = 0.0109489475340948;
Ki = 0.0306597041287344;
Kd = -0.00168563737676081;

PIDController = pid(Kp, Ki, Kd);
PID_Tr = tf(PIDController);
PID_feedback = feedback(PID_Tr*T, 1);

%% Question 9 Robustness 
%controller design
%choose 4 poles to add since A is 4X4. Negative poles will maintain
%stability

%% Observer
%C = [0;1;0;0]; %controllable not observable
C = [1;0;0;0]; %observable and controllable
D = 0;
x0 = [0;15*pi/180;0;0];
sys = ss(A,B,C',D);
%eig(A);% checks stability, if all values are negative it is stable

%is the system observable and controllable?
Sc = ctrb(sys);
So = obsv(sys);
rank(Sc);
rank(So);

%just having a look at the poles and zeros
%rlocus(sys);

%  controller
des_poles = [-3; -3; -3; -3]*0.5;
K = acker(A,B,des_poles);

% Observer
obs_poles = [-30+10i; -30-10i; -30; -50];
Ob = place(A',C,obs_poles);

%% Unused code
%lqr
%Q = eye(4);
%R = 1;
%K_lqr = lqr(A,B,Q,R);

%Discrete time
%Ts = 0.1;
%sys_d = c2d(sys, Ts);

%Ad = sys_d.a;
%Bd = sys_d.b;
%Cd = sys_d.c;
%Dd = sys_d.d;

%des_poles_d = [0.3; 0.3; 0.3; 0.3];
%K_d = acker(Ad,Bd,des_poles_d);

%des_poles_d = [0.3; 0.3; 0.3; 0.3];
%Ob = acker(Ad',Cd',des_poles_d);