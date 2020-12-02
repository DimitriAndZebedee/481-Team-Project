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
ObservabilityMatrix = obsv(A, C);

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

Kp = 80;
Ki = 0;
Kd = 0.05;

PIDController = pid(Kp, Ki, Kd);
PID_Tr = tf(PIDController);
PID_feedback = feedback(PID_Tr*T, 1);

% 7.1 Step response
ts_PID = 0:0.01:0.5;
stepResponsePID = step(PID_feedback, ts_PID);
% plot the step response
%uncomment next, to get step plot
plot(ts_PID, stepResponsePID);

% 7.2 Square Wave Response
%Generate a square wave
[u_square,t_square] = gensig("sqaure",5,200);
%uncomment to get the square wave response
%lsim(PID_feedback,u_square,t_square);

% 7.3 Sinusoidal Reponse
%generate a sinwave
t_sine = linspace(0, 200, 10000);
u_sine = sin(t_sine*pi*2);
%uncomment to get the sine wave response
%lsim(PID_feedback,u_sine,t_sine);

% 8 Introduce noise
% generate noise
%wgn makes an (m,n,p), an mxn matrix with p power Fix power,
rng('default')
noise_size = 0.05;
noise = noise_size * (-2*rand(1,200,1)+1);
%noisey = noise + PID_feedback;

%just figuring out if the noise is working right
t_noise = 1 : length(noise);
%plot(t_noise, PID_feedback);
%plot(t_noise, noise);



