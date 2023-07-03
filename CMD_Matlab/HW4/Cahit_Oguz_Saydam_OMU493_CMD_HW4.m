%% CMD HW4 21631211 Cahit Oğuz Saydam
clc;clear;clear all;
%% Sinusoidal Function
ti = 0;  %[s] initial time
tf = 2; %[s] final time
A = [45 0 30]';   %[cm] initial position
B = [35 30 -15]'; %[cm] final position
ph = 0; %phi angle
th1 = 0; th2 = 90*pi/180; %theta angle
ps1 = 0; ps2 = 45*pi/180; %angle psi
t = ti:0.05:tf; %time value
%% Position vectors of sinusoidal function
p1 = (A(1)-B(1))*cos(pi*t/tf)/2 + (A(1)+B(1))/2;
p2 = (B(2)-A(2))*cos((pi*t/tf)+pi)/2 + (B(2)+A(2))/2;
p3 = (A(3)-B(3))*cos(pi*t/tf)/2 + (A(3)+B(3))/2;
p = [p1; p2; p3];
%% Angles phi, theta and psi
theta = (th2-th1)*cos((pi*t/tf)+pi)/2 + (th2+th1)/2;
phi = (ph-ph)*cos((pi*t/tf)+pi)/2 + (ph+ph)/2;
psi = (ps2-ps1)*cos((pi*t/tf)+pi)/2 + (ps2+ps1)/2;

%% Plot position Vectors and Angle
plot(t,p1);
title('P1 Vectors Plot');
figure()
plot(t,p2);
title('P2 Vectors Plot');
figure()
plot(t,p3);
title('P3 Vectors Plot');
figure()
plot(t,theta*180/pi);
title('Theta Plot');
figure()
plot(t,phi*180/pi);
title('Phi Plot');
figure()
plot(t,psi*180/pi);
title('Psi Plot');
%% Given parameters
a2 = 30; d2 = 10;  d4 = 20; d6 = 12;         %[cm] dimensions t
u1 = [1 0 0]'; u2 = [0 1 0]'; u3 = [0 0 1]'; % unit vectors
sigma1 = 1; sigma3 = -1; sigma5 = 1;
  
lr = length(t); 
for i = 1:lr

    C = rot3(phi(i)) * rot2(theta(i)) * rot1(psi(i));
    r = p - d6*C*u3;
    r1 = r'*u1;
    r2 = r'*u2;
    r3 = r'*u3;  
    lr = length(r); 
   
    % theta1
    alpha1 = atan2(r2(i),r1(i));
    sigma1 = 1;
    psi1 = atan2(d2,sigma1*sqrt(r1(i)^2+r2(i)^2-d2^2));
    theta1(i) = alpha1 - psi1;
    % theta2, theta3
    alpha2 = acos((r1(i)^2+r2(i)^2+r3(i)^2-a2^2-d4^2)/(2*a2*d4));
    alpha3 = atan2(sqrt(1-(r3(i)/a2)^2), r3(i)/a2);
    sigma3 = 1;
    theta3(i) = atan2(sigma3*sin(alpha3), cos(alpha3));
    theta2(i) = atan2((r3(i)*sin(theta3(i))+d4*sin(alpha2+theta3(i))),(r1(i)*cos(theta1(i))+r2(i)*sin(theta1(i))-d2));
    %theta4
    c23 = C(2,3); c13 = C(1,3);
    valsin4 = sigma5*c23; valcos4 = sigma5*c13;
    theta4(i) = atan2(valsin4,valcos4);
    %theta5
    c33 = C(3,3);
    valcos5 = c33; valsin5 = sigma5*sqrt(1-valcos5^2);
    theta5(i) = atan2(valsin5,valcos5);
    %theta6
    c32 = C(3,2); c31 = C(3,1);
    valsin6 = sigma5*c32; valcos6 = -sigma5*c31;
    theta6(i) = atan2(valsin6,valcos6);

end
%% All Theta Plot angles
figure();
plot(t(1:41),theta1*180/pi);
title('theta1 Plot');
figure();
plot(t(1:41),theta2*180/pi);
title('theta2 Plot');
figure();
plot(t(1:41),theta3*180/pi);
title('theta3 Plot');
figure();
plot(t(1:41),theta4*180/pi);
title('theta4 Plot');
figure();
plot(t(1:41),theta5*180/pi);
title('theta5 Plot');
figure();
plot(t(1:41),theta6*180/pi);
title('theta6 Plot');
%% Rotation Functions
% Rotation u1 vector direction is angle of a 
function Rotation1 = rot1(a)
Rotation1 = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end
% Rotation u2 vector direction is angle of a
function Rotation2 = rot2(a)
Rotation2 = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end
% Rotation u3 vector direction is angle of a
function Rotation3 = rot3(a)
Rotation3 = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
end
