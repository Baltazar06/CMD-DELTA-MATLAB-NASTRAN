% MMU 466 Computational Multibody Dynamics HW2-Robot Arm System
% Fatih Örkmez 
clc;
clear;
clear all;

global a1 a2 h Xp Yp thp  

% constant values for the robot arm system
h = 185;
a1 = 150;
a2 = 105;

% initial guess for the joint variables
x0 = [280 40*pi/180 -45*pi/180]';
x0p = x0; % previous solution vector

t = 0:1:10;
n = length(t);

% set joint variables as zero matrix
s = zeros(1,n);
th1 = zeros(1,n);
th2 = zeros(1,n);

% find numerical values
for i = 0:n-1 %time increment from 0 to 10
    Xp = 100*cos((pi*i/10)+pi)+580;
    Yp = 22*cos(pi*i/10)+178;
    thp = (22*cos(pi*i/10)-68)*pi/180; %in radians
    %[y,Fval] = fsolve(@frobotarm,x0p)
    y = fsolve(@frobotarm,x0p) % Numerical solution
    s(i+1) = y(1);
    th1(i+1) = y(2);
    th2(i+1) = y(3);
    x0p = y; % use solution vector for the next initial guess
end

% exact solution part
time = 0:0.01:10;
Yp_new = 22*cos(pi*time/10)+178;
Xp_new = 100*cos((pi*time/10)+pi)+580;
thp_new = (22*cos(pi*time/10)-68)*pi/180; %in radians

sigma = 1;
th2_e = thp_new; %in rad
beta=(Yp_new-h-a2*sin(th2_e))/a1;
th1_e = atan2(beta,sigma*sqrt(1-beta.^2));
s_e = Xp_new-a1*cos(th1_e)-a2*cos(th2_e);

% show the theta1 angle with its exact value
figure(1); 
plot(time,th1_e*180/pi,'k-',t,th1*180/pi,'bo-');
xlabel('time [s]'), ylabel(texlabel('theta1 [deg]'));
legend('exact','numerical','location','best');

% theta2
figure(2); 
plot(time,th2_e*180/pi,'k-',t,th2*180/pi,'bo-');
xlabel('time [s]'), ylabel(texlabel('theta2 [deg]'));
legend('exact','numerical','location','best');

% s 
figure(3); 
plot(time,s_e,'k-',t,s,'bo-');
xlabel('time [s]'), ylabel('s');
legend('exact','numerical','location','best');

% plot Xp, Yp and Thetap values
% figure(4);
% plot(time,Xp_new);
% xlabel('time [s]'), ylabel('Xp(t)')
% figure(5);
% plot(time,Yp_new);
% xlabel('time [s]'), ylabel('Yp(t)');
% figure(6);
% plot(time,thp_new*180/pi);
% xlabel('time [s]'), ylabel(texlabel('theta [deg]'));

% function definition
function F=frobotarm(x)
global a1 a2 h Xp Yp thp  
% s = x(1)
% th1 = x(2)
% th2 = x(3)

 F=[Xp-x(1)-a1*cos(x(2))-a2*cos(x(3));
    Yp-h-a1*sin(x(2))-a2*sin(x(3));
     thp-x(3)];

end