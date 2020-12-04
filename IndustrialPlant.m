%Coefficient values given in Appendix A.1.2.i, default values are chosen

gr = 4;
Jd = 0.0025;
J1 = 0.0271;
k = 8.45;
c1 = 0.004;
c2 = 0.05;

%State space represesentation of Industrial Emulator based on Page 65 of
%Lab Manual

%1)State space

A = [ 0 1 0 0;
    (-k*(gr)^(-2))/Jd  -c1/Jd  (k*(gr)^(-1))/Jd  0;
      0 0 0 1;
     (k*(gr)^(-1))/J1  0 -k/J1 -c2/J1];


B = [0; 1/Jd; 0; 0];

%C matrix with C2 set to 1 for system observability
C = [0,0,0,0;
     0,1,0,0; 
     0,0,0,0;
     0,0,0,0];

D = [0;     0;     0;     0];

%2) Transfer function obtained from state-space representation
[a,b] = ss2tf(A,B,C,D);

%a is given as array since state space is MIMO by default (although only
%zeros for rows 1,3 and 4 --> removing zero rows to form 5X1 matrix
a(1,:)=[];
a(2,:)=[];
a(2,:)=[];

T =tf(a,b);

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
Cj = [Je(1) 0 0 0; 0 Je(2) 0 0; 0 0 Je(3) 0; 0 0 0 Je(4)];

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


% 6. PID Controller
% Hey Dimitri, Just played with the inputs one by one
% Ki didn't really do anything.
% the recomended order is Kp, Kd, Ki

Kp = 0.0109489475340948;
Ki = 0.0306597041287344;
Kd = -0.00168563737676081;

PIDController = pid(Kp, Ki, Kd);
PID_Tr = tf(PIDController);
PID_feedback = feedback(PID_Tr*T, 1);

%9 Robustness 
%controller design
%choose 4 poles to add since A is 4X4. Negative poles will maintain
%stability

p1 = -10 + 10i;
p2 = -10 - 10i;
p3 = -50;
p4 = -10;

K = plain(A,B,[p1 p2 p3 p4]);
sys_closed_loop_with_gain_K = ss(A-B*K,B,C,0);
lsim(sys_closed_loop_with_gain_K,y,t,xo);

