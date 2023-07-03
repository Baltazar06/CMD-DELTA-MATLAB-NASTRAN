clc;
clear;
clear all;
%% Cahit Oğuz Saydam-21631211-OMU493_CMD_HW3

tspan = [0 5]; 
IC = [0 0 pi/2 0 pi/4 2*pi]'; % initial conditions define
[t,y] = ode23(@CMBD_mechanism,tspan,IC); % call ode23 solve function

figure (1)
plot(t,y(:,1),'bo','MarkerSize',3);
xlabel('t');
ylabel('Phi');% y(1)=phi
figure (2)
plot(t,y(:,3),'ro','MarkerSize',3);
xlabel('t');
ylabel('Theta');%y(3)=theta
figure (3)
plot(t,y(:,5),'bo','MarkerSize',3);
xlabel('t');
ylabel('Psi');%y(5)=psi

function dydt = CMBD_mechanism(t,y) 
global  J1 J2 k Mw b T Tw h
h=0.25;
b=0.25;
Mw=0.1;
r=0.05;
Tw=0.002;
k=0.2/pi;
T=0.1*sin(pi*t/2);
J1=0.5*Mw*r^2;
J2=(0.25*Mw*r^2)+((1/12)*Mw*(Tw)^2);

%% y(1)=phi, y(2)=phi_dot, y(3)=theta, y(4)=theta_dot, y(5)=psi, y(6)=psi_dot
dydt=zeros(6,1);
dydt(1)=y(2);
dydt(2)= T+(Mw*b^2+J2*cos(y(3)^2)+((2*J2-J1)*y(2)*y(4)*sin(y(3))*cos(y(3)))+(J1*y(6)*y(4)*cos(y(3))));

dydt(3)=y(4);
dydt(4)=((J1*y(2)*y(6)*cos(y(3))/J2)-(J1-J2)*y(2)^2*sin(y(3))*cos(y(3))+k*y(3)); 

dydt(5)=y(6);
dydt(6)=dydt(2)*sin(y(3)) + y(2)*cos(y(1));
end

