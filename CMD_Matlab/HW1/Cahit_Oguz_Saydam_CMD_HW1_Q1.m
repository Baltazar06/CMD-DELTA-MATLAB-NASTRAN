close all;
clear all;
clc;

U1=[1 0 0 ]';
U2=[0 1 0 ]';
U3=[0 0 1 ]';

theta=deg2rad(30); 
psi=deg2rad(-45);

%% Question1-PartA
U1_ca=rot3(theta)*rot2(psi)*U1;  %The Vector U1 from c to a elements 
U2_ca=rot3(theta)*rot2(psi)*U2; %The Vector U2 from c to a elements 
U3_ca=rot3(theta)*rot2(psi)*U3; %The Vector U3 from c to a elements 
%% PartB
C_ac=[U1_ca U2_ca U3_ca];%The Matrix C(ac)Result

%% PartC
P=[3 4 6];        % Elements of vector P to a
Ra=C_ac*[2 1 2]'; % Elements of vector r to a
S=Ra*[3 4 6 ];    % Dot products Ra and P
q = cpm(P)*Ra;    % dot product of P to Ra
%%
fprintf("The Vectör U1,U2,U3 from c to a elements \n");
fprintf("%.4f U1_ca   %.4f U2_ca   %.4f U3_ca\n",U1_ca,U2_ca,U3_ca);


fprintf("The Matrix C(ac)Result\n");
display(C_ac);

fprintf("Elements of Vector P to a\n");
display(P);

fprintf("Elements of Vector r to a\n");
display(Ra);

fprintf("Dot and Cross Products\n");
display(q);
display(S);

%% R1=First basic rotation matrix as a matrix
function R1 = rot1(phi);

R1 =[  1         0          0 
       0      cos(phi)   -sin(phi)
       0      sin(phi)   cos(phi) ];
end 
%% R2=Second basic rotation matrix as a matrix
function R2 = rot2(psi);

R2=[  cos(psi)  0     sin(psi)
        0       1       0
     -sin(psi)  0     cos(psi)  ];       
end
%% R3=Third basic rotation matrix as a matrix
function R3 = rot3(theta);

R3=[cos(theta)   -sin(theta)      0
    sin(theta)     cos(theta)     0
      0          0               1];
end 
%% Cross Product Matrix vectör q 
function Q = cpm(q)

Q =[ 0     -q(3)    q(2)
    q(3)     0     -q(1)
   -q(2)   q(1)     0   ];
end 
