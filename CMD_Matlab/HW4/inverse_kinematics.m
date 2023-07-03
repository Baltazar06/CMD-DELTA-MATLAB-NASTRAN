function [theta1,theta2,theta3,theta4,theta5,theta6] = inverse_kinematics(p,t)

% Given parameters
a2 = 30; d2 = 10; d4 = 20; d6 = 12; % cm
u1 = [1 0 0]; u2 = [0 1 0]; u3 = [0 0 1]; % unit vectors
sigma1 = 1; sigma3 = -1; sigma5 = 1;

% Length of the wrist position
lr = length(t);

for i = 1:lr

% Transformation matrix
C = rot3(phi(i)) * rot2(theta(i)) * rot1(psi(i));

% Position of the wrist point of the robot
r = p - d6Cu3;

% Theta1
alpha1 = atan2(r(2,i),r(1,i));
sigma1 = 1;
psi1 = atan2(d2,sigma1*sqrt(r(1,i)^2+r(2,i)^2-d2^2));
theta1(i) = alpha1 - psi1;

% Theta2, Theta3
alpha2 = acos((r(1,i)^2+r(2,i)^2+r(3,i)^2-a2^2-d4^2)/(2a2d4));
alpha3 = atan2(sqrt(1-(r(3,i)/a2)^2), r(3,i)/a2);
sigma3 = 1;
theta3(i) = atan2(sigma3*sin(alpha3), cos(alpha3));
theta2(i) = atan2((r(3,i)sin(theta3(i))+d4sin(alpha2+theta3(i))),(r(1,i)*cos(theta1(i))+r(2,i)*sin(theta1(i))-d2));

% Theta4
c23 = C(2,3); c13 = C(1,3);
valsin4 = sigma5c23; valcos4 = sigma5c13;
theta4(i) = atan2(valsin4,valcos4);

% Theta5
c33 = C(3,3);
valcos5 = c33; valsin5 = sigma5*sqrt(1-valcos5^2);
theta5(i) = atan2(valsin5,valcos5);

% Theta6
c32 = C(3,2); c31 = C(3,1);
valsin6 = sigma5c32; valcos6 = -sigma5c31;
theta6(i) = atan2(valsin6,valcos6);

end