% CMD HW3 - Solving multibody system ODE
% Fatih Orkmez 21731334


global J1 J2 k mw b r tw
% define variables
h=0.25; b=0.25; mw=0.1; r=0.05; tw=0.002;
J1 = (mw*r^2)/2;
J2 = (mw*r^2)/4 +  (mw*tw^2)/12;
k = 0.2/pi;

tspan = [0 5]; % time interval
% y(1)=phi, y(2)=phi_dot, y(3)=theta, y(4)=theta_dot, y(5)=psi, y(6)=psi_dot
IC = [0 0 pi/2 0 pi/4 2*pi]'; % initial conditions as defined above
[t,y] = ode23s(@fcn_odesolver, tspan, IC); % call ode23s solver

figure, plot(t,y(:,1),'bo','MarkerSize',3), xlabel('t'), ylabel('phi(t)')
figure, plot(t,y(:,3),'ro','MarkerSize',3), xlabel('t'), ylabel('theta(t)')
figure, plot(t,y(:,5),'ro','MarkerSize',3), xlabel('t'), ylabel('psi(t)')

function dydt = fcn_odesolver(t,y)
global J1 J2 k mw b
% define Torque in terms of time
T = 0.1*sin(pi*t/2);

dydt = zeros(6,1);

dydt(1) = y(2); % derivative of the phi(t)
dydt(2) = (T + J1*y(6)*y(4)*cos(y(3)) + (2*J2-J1)*y(2)*y(4)*sin(y(3))*cos(y(3)))/(mw*b^2+J2*cos(y(3))^2);

dydt(3) = y(4); % derivative of the theta(t)
dydt(4) = ((J1-J2)*y(2)^2*sin(y(3))*cos(y(3)) - k*y(3) - J1*y(2)*y(6)*cos(y(3)))/J2;

dydt(5) = y(6); % derivative of the psi(t)
%dydt(psi_dot)=dydt(phi_dot)*sin(theta)+phi_dot*cos(theta)
dydt(6) = dydt(2)*sin(y(3)) + y(2)*cos(y(3));

end
