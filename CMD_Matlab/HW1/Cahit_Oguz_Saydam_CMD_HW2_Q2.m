close all;
clear all;
clc;

U1=[1 0 0 ]';
U2=[0 1 0 ]';
U3=[0 0 1 ]';

theta=deg2rad(30); 
%% Question1 PartA
U1_ab=U1*cos(theta)+U2*sin(theta);

U2_ab=U2*cos(theta)-U2*sin(theta);

U3_ab=U3;
%% PartB
U1_ab=rot3(theta)*U1; %The Vector U1 from a to b elements 
U2_ab=rot3(theta)*U2; %The Vector U3 from a to b elements 
U3_ab=rot3(theta)*U3; %The Vector U3 from a to b elements 
%% PartC
C_ab=[U1_ab U2_ab U3_ab]; %The Matrix C_ab Result

Ra=[4 5 3 ]'; %R to a Vector Elements

Rb=C_ab*Ra;  %R to b  Vector Elements
%%
fprintf("The Vectör U1,U2,U3 from a to b elements \n");
fprintf("%.4f U1_ab   %.4f U2_ab   %.4f U3_ab  \n", U1_ab, U2_ab, U3_ab);

fprintf("The Matrix C_ab Result\n");
display(C_ab);

fprintf("R to a Vector Elements\n");
display(Ra);


fprintf("R to b  Vector Elements\n");
display(Rb);

%% R1=First basic rotation matrix as a matrix
function R1 = rot1(phi);

R1 =[  1         0          0 
       0      cos(phi)   -sin(phi)
       0      sin(phi)   cos(phi) ];
end 
%% R2=Second basic rotation matrix as
function R2 = rot2(psi);  a matrix

R2=[  cos(psi)  0     sin(psi)
        0       1       0
     -sin(psi)  0     cos(psi)  ];       
end
%% R3=Third basic rotation matrix as a matrix
function R3 = rot3(theta);% R3=Third basic rotation matrix as a matrix

R3=[cos(theta)   -sin(theta)      0
    sin(theta)     cos(theta)     0
      0          0               1];
end 
%% Cross Product Matrix vectör q 
function Q = cpm(q)%Cross Product Matrix vectör q 

Q =[ 0     -q(3)    q(2)
    q(3)     0     -q(1)
   -q(2)   q(1)     0   ];
end 

