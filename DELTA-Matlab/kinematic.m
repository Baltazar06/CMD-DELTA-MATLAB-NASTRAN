clc, clear, clear all;


% r1 = [1 0 0]'; r2 = [0 1 0]'; r3 = [0 0 1]';

R1 = [cos(th) -sin(th) 0]'; R2 = [sin(th) cos(th) 0]'; R3 = [0 0 1]';

theta = 30*pi/180; psi = -45*pi/180;
Rot3tob = rot3(theta); 
Rot2toc = rot2(psi);

TFM1 =  Rot3tob*Rot2toc;
U1ca = TFM1*u1; 
U2ca = TFM1*u2; 
U3ca = TFM1*u3; 
Pa = [3 4 6]'; 
Rc = [2 1 2]'; 
Ra = TFM1*Rc;  
s = dot(Pa,Ra); 
q = cpm(Pa)*Ra; 




%% Three Equations of Motion the Kinematics of Robot Arm Mechanism 
function F=kinematic_eq(x)
global 

Rk1=B1*cos(thk)+B2*cos(thk^)*cos(phik);
Rk2=B2*sin(phik);
Rk3=B1*sin(thk)+B2*sin(thk)*cos(phik);
end







Rk1=B1*cos(thk)+B2*cos(thk^)*cos(phik);
Rk2=B2*sin(phik);
Rk3=B1*sin(thk)+B2*sin(thk)*cos(phik);


% time = 0:0.01:10;
% Yp_new = (22*cos((pi*time)/10)+178);
% Xp_new = (100*cos(((pi*time)/10)+pi)+580);
% thp_new = (22*cos((pi*time)/10)-68)*pi/180;
% sigma=1;
% 
% th2_e = thp_new;
% beta=(Yp_new-h-a2*sin(th2_e))/a1;
% th1_e = atan2(beta,sigma*sqrt(1-beta.^2));
% s_e = Xp_new-a1*cos(th1_e)-a2*cos(th2_e);