%State space represesentation of Industrial Emulator based on Page 65 of
%Lab Manual

A1 = [0,1,0,0];
A2 = [(-k*(gr)^(-2))/Jd, -c1/Jd, (k*(gr)^(-1))/Jd, 0];
A3 = [0,0,0,1];
A4 = [(k*(gr)^(-1))/J1, 0, -k/Jl, -c2/J1];
A = [A1;A2;A3;A4]


B = [0;1/Jd;0;0];

%C matrix with C2 set to obtain system observability
C = [0,0,0,0; 0,C2,0,0; 0,0,0,0; 0,0,0,0];

%Coefficient values given in Appendix A.1.2.i, default values are chosen

gr=4;
Jd = 0.0025;
J1= 0.0271;
k = 8.45;
c1= 0.004;
c2= 0.05;
