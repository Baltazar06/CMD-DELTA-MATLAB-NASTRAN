clc, clear, clear all;

% Here we will try to define inverse kinematics of the Delta robot
% We know the p value which is a location verctor for effector
 
%% Define geometric parameters
b0=100; b1=110; b2=196; b7=42; h0=500; h7=0;
beta = [0 2*pi/3 -2*pi/3]'; % angles to define servo positioning

% define unit vectors
u1 = [1 0 0]'; u2 = [0 1 0]'; u3 = [0 0 1]';

%% Controllable part
% Define desired p vector as a effector position
p = [0 0 400]';

% for function solving part
phi_k = zeros(1,3);
th_k = zeros(1,3);
thstr_k = zeros(1,3);
Yk = zeros(1,3);
th_k0 = zeros(1,3);
thprime_k = zeros(1,3);

for i = 1:3 % call for for each leg(k=1,2,3)    
    % define rotation matrix for each leg (B1,B2,B3 respectively)
    Rot3b = rot3(-beta(i));
    
    % first we define the rk position vector of leg from point Bk to Ak
    rk = Rot3b*p - u1*(b0-b7) - u3*(h0-h7); % leg position vector
    % rk will change with beta angle
    
    %% Then we will be using unit vectors of rk to define unkown angles
    rk1 = rk'*u1;
    rk2 = rk'*u2; % rk(2)
    rk3 = rk'*u3;    

    % Define unknown angles with solve function
    % all angles are in degree
    % to define phi_k angle
    sigma1 = 1; % sigma_prime
    anglesin = rk2; % represents the sin(phi_k)
    anglecos = sigma1*sqrt(b2^2 -rk2^2); % represents cos(phi_k)
    phi_k(i) = atan2(anglesin, anglecos)*180/pi; % to define phi_k angle
    
    % to define th_k angle
    sigma2 = -1; % sigma k
    Fk = ((rk1^2)+(rk2^2)+(rk3^2)+(b1^2)-(b2^2))/(2*b1);
    
    % version from the book
    th_k0(i) = -atan2(rk3, rk1)*180/pi; % to define theta0 angle in degree
    Yk(i) = atan2(sqrt(rk1^2+rk3^2-Fk^2), Fk)*180/pi;
    th_k(i) = th_k0(i) + sigma2*Yk(i); % th_k angle    
    % to define thstr_k angle
    thstr_k(i) = -atan2(sigma1*(rk3+b1*sin(th_k(i)pi/180)), sigma1(rk1-b1*cos(th_k(i)*pi/180)))*180/pi;
    thprime_k(i) = thstr_k(i) - th_k(i);

    % Singularity Check
    if thprime_k(i) == 0
        fprintf('<< Singularity Inverse Kinematics >>\n In this case forearm and basearm creates singularity!\n');
        
    elseif thstr_k == 180 % pi/2 in rad
                fprintf('<< Singularity of Forward Kinematics >>\n Here is a problem with pose!\n');
    end
end


%% Functions

% Here rotation along u3 direction is defined where a is angle
function Rotation3 = rot3(a)
Rotation3 = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
end
