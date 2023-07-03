clc, clear, clear all;

% Here we will try to define inverse kinematics of the Delta robot
% We know the p value which is a location verctor for effector
 
%% Define geometric parameters
b0=90; b1=90; b2=110; b7=40; h0=500; h7=0;
beta = [0 2*pi/3 -2*pi/3]'; % angles to define servo positioning

% define unit vectors
u1 = [1 0 0]'; u2 = [0 1 0]'; u3 = [0 0 1]';

%% Controllable part
% Define desired p vector as a effector position
p = [10 10 0]';

% for function solving part
% x0p = [0,0,0]; % initial guess
phi_k = zeros(1,3);
th_k = zeros(1,3);
thstr_k = zeros(1,3);
for i = 1:3 % call for for each leg(k=1,2,3)    
    % define rotation matrix for each leg (B1,B2,B3 respectively)
    Rot3b = rot3(beta(i));
    
    % first we define the rk position vector of leg from point Bk to Ak
    rk = Rot3b*p - u1*(b0-b7) - u3*(h0-h7); % leg position vector
    % rk will change with beta angle
    
    % Then we will be using unit vectors of rk to define unkown angles
    rk1 = rk'*u1;
    rk2 = rk'*u2;
    rk3 = rk'*u3;    
    %     % Define unknown angles with solve function
    %     y = fsolve(@findangle,x0p) 
    %     phi_k(i)   = y(1); 
    %     th_k(i)    = y(2); 
    %     thstr_k(i) = y(3);
    %     x0p = y; % use solution vector for the next initial guess
    
    % Define unknown angles with solve function
    sigma = 1;
    anglesin = rk2/b2; % represents the sin(phi_k)
    anglecos = sigma*sqrt(1-anglesin.^2); % represents cos(phi_k)
    phi_k(i) = atan2(anglesin, anglecos); % to define phi_k angle
    
    
    Fk=(rk1^2)+(rk2^2)+(rk3^2)+(b1^2)-(b2^2)/2*b1;
    thko(i) = atan2(rk3, rk1); % to define thetax angle
    Yk(i)=atan2((rk1^2+rk3^2-Fk^2),Fk);
    th_k=(thko+sigma*Yk);%to define th_k angle
    
    thstr_k=-atan2(sigma*(rk3+b1*sin(th_k)),sigma*(rk1-b1*cos(th_k)));%to define thstr_k angle
end


%% Functions

% Here rotation along u3 direction is defined where a is angle
function Rotation3 = rot3(a)
Rotation3 = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
end

% Function to solve unknown angles
function F = findangle(x)
global b1 b2 rk1 rk2 rk3
% phi_k = x(1)
% th_k = x(2)
% thstr_k = x(3)

 F=[b1*cos(x(2))+b2*cos(x(3))*cos(x(1))-rk1;
    b2*sin(x(1))-rk2;
     b1*sin(x(2))+b2*sin(x(3))*cos(x(1))+rk3];
end