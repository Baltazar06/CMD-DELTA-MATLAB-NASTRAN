%% CMD Hw 4 Puma Robot Inverse Kinematics - Fatih Örkmez 21731334
clc, clear;
%% Specify position and transformation matrix by using sinusoidal function

ti = 0;  %[s] initial time
tf = 2; %[s] final time

A = [45 0 30]';   %[cm] initial position
B = [35 30 -15]'; %[cm] final position

ph = 0; % angle phi is zero all the time
th1 = 0; th2 = 90; %theta angle
ps1 = 0; ps2 = 45; %angle psi
t = ti:0.05:tf; %time value

%% definition of position vectors by using sinusoidal function
p1 = (A(1)-B(1))*cos(pi*t/tf)/2 + (A(1)+B(1))/2;
p2 = (B(2)-A(2))*cos((pi*t/tf)+pi)/2 + (B(2)+A(2))/2;
p3 = (A(3)-B(3))*cos(pi*t/tf)/2 + (A(3)+B(3))/2;

p = [p1; p2; p3];

%% definition of the angles phi, theta and psi

theta = (th2-th1)*cos((pi*t/tf)+pi)/2 + (th2+th1)/2;
phi = (ph-ph)*cos((pi*t/tf)+pi)/2 + (ph+ph)/2;
psi = (ps2-ps1)*cos((pi*t/tf)+pi)/2 + (ps2+ps1)/2;

% for t = ti:0.05:tf
%     %definition of position vectors by using sinusoidal function
%     p1 = (A(1)-B(1))*cos(pi*t/tf)/2 + (A(1)+B(1))/2;
%     p2 = (B(2)-A(2))*cos((pi*t/tf)+pi)/2 + (B(2)+A(2))/2;
%     p3= (A(3)-B(3))*cos(pi*t/tf)/2 + (A(3)+B(3))/2;
% 
%     p = [p1 p2 p3]';
% end
% plot(t,p1);
% figure()
% plot(t,p2);
% figure()
% plot(t,p3);
    
%% Inverse kinematics of Puma Robot

%% given parameters
a2 = 30; d2 = 10;  d4 = 20; d6 = 12; %[cm] dimensions of the robot

u1 = [1 0 0]'; u2 = [0 1 0]'; u3 = [0 0 1]'; % unit vectors

sigma1 = 1; 
% sigma3 = -1; 
sigma3 = 1;
sigma5 = 1;

%% transformation matrix along the u3,u2,u1 directions respectively
C = rot3(phi(1)) * rot2(theta(1)) * rot1(psi(1));

%% position of the wrist point of the robot
r = p - d6*C*u3;

r1 = r'*u1;
r2 = r'*u2;
r3 = r'*u3;  
lr = length(r); % length of the wrist position


% 
% 
% 
% s = zeros(1, lr); % initialize s and c arrays
% c = zeros(1, lr);
% for i = 1:lr
%     %% theta1
%     alpha1 = atan2(r2(i),r1(i));
%     psi1 = atan2(d2,sigma1*sqrt(r1(i)^2+r2(i)^2-d2^2));
%     theta1_i = alpha1 - psi1; % use a different variable name for theta1
%     
%     % theta3
%     sigma1=1;
%     s(i) = ((r1(i)^2+r2(i)^2+r3^2)-(d2^2+a2^2+d4^2))/(2*a2*d4);
%     c(i) = sigma1*sqrt(1- s(i)^2);
%     
%     theta3=atan2(s(i),c(i));
%     
%     % theta2
%     D=d4*c(i)*a2-r3*a2-d4*s(i)*d2; % define D
%     s(i)=(r2(i)*a2+d4*s(i)*sigma1*d2-d4*sigma1*c(i)*a2-r3*d4*c(i))/D;
%     c(i)=(r1(i)*a2+d4*c(i)*sigma1*d2+d4*sigma1*s(i)*a2+r3*d4*s(i))/D;
%     
%     theta2=atan2(s(i),c(i));
%     
%     % theta4,5,6  Rotation Matrix C''
%     C11=c(theta4)*c(theta5)*c(theta6)-s(theta4)*s(theta6);
%     C12=c(theta4)*c(theta5)*s(theta6)-s(theta4)*s(theta6);
%     C13=c(theta4)*s(theta5);
%     
%     C21=s(theta4)*c(theta5)*c(theta6)-c(theta4)*s(theta6);
%     C22=-s(theta4)*c(theta5)*s(theta6)+c(theta4)*c(theta6);
%     C23=s(theta4)*s(theta5);
%     
%     C31=-s(theta5)*c(theta6);
%     C32=s(theta5)*s(theta6);
%     C33=c(theta5);
%     C=[C11 C12 C13; C21 C22 C23; C31 C32 C33];
%     
%     % theta5
%     s(theta5)=sigma5*sqrt(1-C(3,3)^2); % use the correct variable name for s
%     c(theta5)=C(3,3);
%     theta5=atan2(s(theta5),c(theta5));
%     
%     % theta4
%     theta4=atan2(sigma5*C(2,3),sigma5*C(1,3));
%     
%     % theta6
%    
% end
% 












% 
% for i = 1:lr
%     %% theta1
%     alpha1 = atan2(r2(i), r1(i));
%     psi1 = atan2(d2, sigma1 * sqrt(r1(i)^2 + r2(i)^2 - d2^2));
%     theta1_i = alpha1 - psi1;
%     
%     % theta3
%     sigma1 = 1;
%     s3 = ((r1(i)^2 + r2(i)^2 + r3(i)^2) - (d2^2 + a2^2 + d4^2)) / (2 * a2 * d4);
%     c3 = sigma3 * sqrt(1 - s3^2);
%     theta3_i = atan2(s3, c3);
% 
% 
%    
% 
%     % theta2
%     
%     D = r2(i) * (a2 + d4 * c3) - r1(i) * d4 * s3 - d2 * (a2 + d4 * c3) * sigma1;
%     s2 = (r3 - d4 * s3 - a2 * sigma1) / D;
%     c2 = (r1(i) * (a2 + d4 * c3) + r2(i) * d4 * c3 + d2 * a2 * sigma1) / D;
%     theta2_i = atan2(s2, c2);
%     
%     % theta4, theta5, theta6
%     theta4=0.7;
%     theta5=0.7;
%     theta6=0.6;
%     
%     R06 = [C11(theta1_i, theta2_i, theta3_i, theta4, theta5, theta6) C12(theta1_i, theta2_i, theta3_i, theta4, theta5, theta6) C13(theta1_i, theta2_i, theta3_i, theta4, theta5, theta6);
%            C21(theta1_i, theta2_i, theta3_i, theta4, theta5, theta6) C22(theta1_i, theta2_i, theta3_i, theta4, theta5, theta6) C23(theta1_i, theta2_i, theta3_i, theta4, theta5, theta6);
%            C31(theta1_i, theta2_i, theta3_i, theta4, theta5, theta6) C32(theta1_i, theta2_i, theta3_i, theta4, theta5, theta6) C33(theta1_i, theta2_i, theta3_i, theta4, theta5, theta6)];
%     
%     theta5_i = atan2(sigma5 * sqrt(1 - R06(3,3)^2), R06(3,3));
%     theta4_i = atan2(sigma5 * R06(2,3), sigma5 * R06(1,3));
%     theta6_i = atan2(sigma5 * R06(3,2), -sigma5 * R06(3,1));
%     
%     % save results
%     theta1(i) = theta1_i;
%     theta2(i) = theta2_i;
%     theta3(i) = theta3_i;
%     theta4(i) = theta4_i;
%     theta5(i) = theta5_i;
%     theta6(i) = theta6_i;
% end





%% finding theta angles
    
    %% theta1
    alpha1 = atan2(r2(i),r1(i));
    psi1 = atan2(d2,sigma1*sqrt(r1(i)^2+r2(i)^2-d2^2));
    theta1(i) = alpha1 - psi1;
    %theta2
    
    %% theta3
    theta1= alpha1 - psi1;
    sigma1=+1
    sin(theta3)=((r1^2+r2^2+r3^2)-(d2^2+a2^2+d4^2))/(2*a2*d4);
    cos(theta3)=sigma3*sqrt(1- sin(theta3)^2);
    theta3=atan2(sin(theta3),cos(theta3));
    %r1^2+r2^2+r3^2=2*a2*d4*sin(theta23)*cos(theta2)==2*a2*d4*sin(theta23)*cos(theta2)
    %r3=-1;
    %% theta2
    R1=d4*sin(theta2)*cos(theta3)+d4*sin(theta3)*cos(theta2)+a2sin(theta2)*cos(theta3);
    R1=sin(theta2)*(d4*cos(theta3))+cos(theta2)*(d4*sin(theta3+a2));
    
    R3=d4*cos(theta2)*cos(theta3)-d4*sin(theta2)*sin(theta3)-a2*sin(theta2);
    R3=sin(theta2)*(-d4*sin(theta3)-a2)+cos(theta2)*(d4+cos(theta3));
    
    D=0; %Determinant 
    %Theta3=-pi/2%singularity
    
    sin(theta2)=(R1*d4*cos(theta3)-r3(a2+d4*sin(theta3)))/D;
    cos(theta2)=(R1*(a2+d4*sin(theta3))+r3*(d4*cos(theta3)))/D;
    
    theta2=atan2(sin(theta2),cos(theta2));
    
    %% theta4,5,6  Rotation Matrix C''
    C=[C11 C12 C13; C21 C22 C23; C31 C32 C33];
    C11=cos(theta4)*cos(theta5)*cos(theta6)-sin(theta4)*sin(theta6);
    C12=cos(theta4)*cos(theta5)*sin(theta6)-sin(theta4)*sin(theta6);
    C13=cos(theta4)*sin(theta5);
    
    C21=sin(theta4)*cos(theta5)*cos(theta6)-cos(theta4)*sin(theta6);
    C22=-sin(theta4)*cos(theta5)*sin(theta6)+cos(theta4)*cos(theta6);
    C23=sin(theta4)*sin(theta5);
    
    C31=-sin(theta5)*cos(theta6);
    C32=sin(theta5)*sin(theta6);
    C33=cos(theta5);
    %% theta5
    cos(theta5)=C33;
    sin(theta5)=sigma5*sqrt(1-C33^2)
    theta5=atan2(sigma5*sqrt(1-C33^2),C33);
    %% theta4
    theta4=atan2(sigma5*C23,sigma5*C13);
    %% theta6
    theta6=atan2(sigma5*C32,-sigma5*C31);













%% Plot the theta angles
figure();
plot(t,theta1*180/pi);

%% Rotation Functions

% Here rotation along u1 direction is defined where a is angle
function Rotation1 = rot1(a)
Rotation1 = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

% Here rotation along u2 direction is defined where a is angle
function Rotation2 = rot2(a)
Rotation2 = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

% Here rotation along u3 direction is defined where a is angle
function Rotation3 = rot3(a)
Rotation3 = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
end


%     % finding thete angles
%     %theta1
%     alpha1 = atan2(r2(i),r1(i));
%     psi1 = atan2(d2,sigma1*sqrt(r1(i)^2+r2(i)^2-d2^2));
%     theta1(i) = alpha1 - psi1;
%     %theta2


% % transformation matrix along the u3,u2,u1 directions respectively
% C = rot3(phi(1)) * rot2(theta(1)) * rot1(psi(1));
% 

% position of the wrist point of the robot
% r = p - d6*C*u3;
% 
% r1 = r'*u1;
% r2 = r'*u2;
% r3 = r'*u3;