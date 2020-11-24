%Coefficient values given in Appendix A.1.2.i, default values are chosen

gr = 4;
Jd = 0.0025;
J1 = 0.0271;
k = 8.45;
c1 = 0.004;
c2 = 0.05;

%State space represesentation of Industrial Emulator based on Page 65 of
%Lab Manual


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

D = [0;
     0;
     0;
     0];




%1)





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

% 3.2 Observability matrix
ObservabilityMatrix = obsv(A, C);

% 3.3 Jordan Matrix
JordanMatrix = jordan(A);

% 4.1 Impulse response
sys = ss(A, B, C, D);
impulseResponse = impulse(sys);
ti = 1:length(impulseResponse);

% Plot Impusle Response
%plot(ti, impulseResponse(:,2));

%4.2 Step reponse
stepResponse = step(sys);
ts = 1:length(stepResponse);

% plot the step response
%plot(ts, stepResponse);

% 5 Bode plot
bodePlot = bode(sys);
%bodeMag = bodePlot(:,1);
%bodePhase = bodePlot(:,2);
%bodewout = bodePlot(:,3);
%tbm = 1:length(bodeMag);
%tbp = 1:length(bodePhase);
%tbw = 1:length(bodewout);
%plot(tbm, bodeMag);
%plot(tbp, bodePhase);
%plot(tbw, bodewout);