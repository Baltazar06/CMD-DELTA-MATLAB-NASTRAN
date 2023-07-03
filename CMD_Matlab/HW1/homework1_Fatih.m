clc, clear, clear all;
% in this code we try to solve two problems for frame rotation

% Define unit vectors
u1 = [1 0 0]'; u2 = [0 1 0]'; u3 = [0 0 1]';

%% Problem 1

% Given rotation angles in radian
theta = 30*pi/180; psi = -45*pi/180;

% rotation matrices
Rot3tob = rot3(theta); % rotation from frame a to frame b
Rot2toc = rot2(psi);   % rotation from frame b to frame c

% TransFormation Matrix, C^(a/c)
TFM1 =  Rot3tob*Rot2toc; % unit vectors of frame c with respect to a

% Unit basis vectors
U1ca = TFM1*u1; % unit vector u1 of frame c with respect to a
U2ca = TFM1*u2; % unit vector u2 of frame c with respect to a
U3ca = TFM1*u3; % unit vector u3 of frame c with respect to a

% Dot & Cross product
Pa = [3 4 6]'; % components of vector p in frame a
Rc = [2 1 2]'; % components of vector r in frame c

Ra = TFM1*Rc;  % components of vector r in frame a

s = dot(Pa,Ra); % dot product of vectors
q = cpm(Pa)*Ra; % cross product of vectors
%qalternate = cross(Pa,Ra); % cross product result checked


% Display the results
fprintf("\tResults for Question 1\n");
fprintf("a) unit basis vectors\n");
fprintf("u1(c/a) = (%.4f)u1 + (%.4f)u2 + (%.4f)u3\n",U1ca(1),U1ca(2),U1ca(3));
fprintf("u2(c/a) = (%.4f)u1 + (%.4f)u2 + (%.4f)u3\n",U2ca(1),U2ca(2),U2ca(3));
fprintf("u3(c/a) = (%.4f)u1 + (%.4f)u2 + (%.4f)u3\n",U3ca(1),U3ca(2),U3ca(3));

fprintf("b) transformation matrix\n");
display(TFM1);

fprintf("c) dot and cross products\n");
fprintf(" s = %f\n",s);
display(q);


%% Problem 2

% Given data
Rina = [4 5 3]'; % components of vector r in frame a
theta = -30*pi/180; % rotation angle in radian (clockwise)

% remember rotation matrix along u3 vector
Rot3tob = rot3(theta); % rotation from frame a to frame b

% TransFormation Matrix, C^(a/b)
TFM2 =  Rot3tob; % unit vectors of frame b with respect to a

% Unit basis vectors
U1ba = TFM2*u1 % unit vector u1 of frame b with respect to a
U2ba = TFM2*u2 % unit vector u2 of frame b with respect to a
U3ba = TFM2*u3 % unit vector u3 of frame b with respect to a

Rinb = (TFM2')*Rina; % components of vector r in frame b

% Display the results
fprintf("\tResults for Question 2\n");
fprintf("b) unit basis vectors\n");
fprintf("u1(b/a) = (%.4f)u1 + (%.4f)u2 + (%.4f)u3\n",U1ba(1),U1ba(2),U1ba(3));
fprintf("u2(b/a) = (%.4f)u1 + (%.4f)u2 + (%.4f)u3\n",U2ba(1),U2ba(2),U2ba(3));
fprintf("u3(b/a) = (%.4f)u1 + (%.4f)u2 + (%.4f)u3\n",U3ba(1),U3ba(2),U3ba(3));

display(TFM2);

display(Rinb);
