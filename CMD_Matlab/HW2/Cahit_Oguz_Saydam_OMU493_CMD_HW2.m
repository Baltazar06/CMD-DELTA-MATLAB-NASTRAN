clc;
clear;
clear all;
%% Cahit Oğuz Saydam-21631211-OMU493_CMD_HW2
%% Mobile Robot Arm Mechanism Parameters Define
global a1 a2 h Xp Yp thp  
a1=150;
a2=105;
h=185;
x0 = [280 40*pi/180 -45*pi/180]';%Conversion Factor
x0p = x0;
%% Numerical Solution Values
t=(0:1:10);
n=length(t);
th1=zeros(1,n);
th2=zeros(1,n);
s=zeros(1,n);
%%
for i=1:n %time
        Xp=(100*cos(((pi*(i-1))/10)+pi)+580);
        Yp=(22*cos((pi*(i-1))/10)+178);
        thp=(22*cos((pi*(i-1))/10)-68)*pi/180;%Radian Conversion Factor
        y = fsolve(@robotarm_eq,x0p) % Numerical solution
        s(i)=y(1);
        th1(i)=y(2);
        th2(i)=y(3);
        x0p=y;%Use the numerical solution values for the initial guess 
end

%% Exact Solution Values
time = 0:0.01:10;
Yp_new = (22*cos((pi*time)/10)+178);
Xp_new = (100*cos(((pi*time)/10)+pi)+580);
thp_new = (22*cos((pi*time)/10)-68)*pi/180;
sigma=1;

th2_e = thp_new;
beta=(Yp_new-h-a2*sin(th2_e))/a1;
th1_e = atan2(beta,sigma*sqrt(1-beta.^2));
s_e = Xp_new-a1*cos(th1_e)-a2*cos(th2_e);

%% Numerical and Exact Solutions for s, theta1 and theta2 Plots
figure(1);
plot(time,th1_e*180/pi,'k-',t,th1*180/pi,'bo-')
xlabel(texlabel('time [second]')),ylabel('theta1')
legend('exact','numerical','location','best')

figure(2);
plot(time,th2_e*180/pi,'k-',t,th2*180/pi,'bo-')
xlabel(texlabel('theta1 [degrees]')),ylabel('s')
legend('exact','numerical','location','best')

figure(3);
plot(time,s_e,'k-',t,s,'bo-')
xlabel(texlabel('theta1 [degrees]')),ylabel('s')
legend('exact','numerical','location','best')

%% Xp, Yp and Thp Values on the Time domain Plots
figure(4);
plot(time,Xp_new);
xlabel('t[s]'), ylabel('Xp(t)')
figure(5);
plot(time,Yp_new);
xlabel('t[s]'), ylabel('Yp(t)');
figure(6);
plot(time,thp_new*180/pi);
xlabel('t[s]'), ylabel('Thp[deg]');

%% Three Equations of Motion the Kinematics of Robot Arm Mechanism 
function F=robotarm_eq(x)
global a1 a2 h Xp Yp thp  
a1=150;
a2=105;
h=185;
 F=[Xp-x(1)-a1*cos(x(2))-a2*cos(x(3));
    Yp-h-a1*sin(x(2))-a2*sin(x(3));
     thp-x(3)];
end
   
   