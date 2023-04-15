% Function to derive the dynamic equations of motion for the double
% pendulum
clear all

name = 'double_pend_springs';
%% Setup
% Constant parameters
syms m1 m2 %% mass
syms I1 I2 %% inertia
syms l1 l2 c1 c2 %%length
syms g %% gravity
%% angles
syms th10 th20  %% initial angle
syms th1 th2 dth1 dth2 ddth1 ddth2 real
%% springs and dampers
syms k1 k2 b1 b2 real 

%% external torques
syms tau1 tau2 real;

%% Symbolic groupings
q  = [th1   th2   ]';
dq = [dth1  dth2  ]';
ddq= [ddth1 ddth2 ]';
u  = [tau1 tau2]';
z = [q ; dq];

% % p   = [l1; l2 ; c1; c2; m1; m2; I1; I2; g; ...
% %             0 ; th10 ; th20 ; k1; k2 ; 0 ; 0 ; 0; b1; b2];  % parameters
p   = [l1; l2 ; c1; c2; m1; m2; I1; I2; g; ...
           th10 ; th20 ; k1; k2 ; b1; b2;];  % parameters

%% Handy anonomous functions
ddt = @(x)( jacobian(x,q)*dq + jacobian(x,dq)*ddq);
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));    % force contributions to generalized forces
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));   % moment contributions to generalized forces

%% Kinemeatics
rO = [0 0 0]';    % position of link 1
ehat1 = [sin(th1) -cos(th1) 0]';           % Define unit vector along Leg 1
ehat2 = [sin(th1+th2) -cos(th1+th2) 0]';  % Define unit vector along Leg 2
ghat  = [0 -1 0]';                       

rc1 = c1*ehat1;          % Position of link 1 CoM
rB  = l1*ehat1;          % Position of base of link 2
rc2 = rB + c2*ehat2;     % Position of CoM of link 2
rC  = rB + l2*ehat2;     % Position of end of link 3

vc1 = ddt(rc1);          % Velocity of link 1 CoM
omega1 = dth1;           % Angular velocity of Link 1
vc2 = ddt(rc2);          % Velocity of link 2 CoM
omega2 = dth1 + dth2;    % Angular velocity of link 2


% % % %% Get acceleration at keypoints 
% % % ddtrO=[0 0 0]';
% % % ddtrB= ddt(ddt(rB));
% % % ddtrC= ddt(ddt(rC));

%% Lagrangian
% Kinetic and Potential Energy of link 1
T1 = simplify( 1/2 * m1 * (vc1.' * vc1) + 1/2 * I1 * omega1^2 );
V1 = m1 * g * dot(rc1, -ghat);

% Kinetic and Potential Energy of link 2
T2 = simplify( 1/2 * m2 * (vc2.' * vc2) + 1/2 * I2 * omega2^2);
V2 = m2 * g * dot(rc2, -ghat);

% Spring potential energies
Ve1 = 1/2 * k1 * (th1 - th10)^2;
Ve2 = 1/2 * k2 * (th2 - th20)^2;

% Energy dissipation through damping
De1 = 1/2 * b1 * (dth1)^2;
De2 = 1/2 * b2 * (dth2)^2;

dD_dqd = jacobian(De1+De2,dq)';

% Compute Lagrangian and total energy
L = simplify(T1+T2 - V1 - V2 - Ve1 - Ve2);
E = simplify(T1+T2 + V1 + V2 + Ve1 + Ve2);

% Total potential energy
V= simplify(V1 + V2 + Ve1 + Ve2);

%% Generalized forces
Q_tau1 = M2Q(tau1,omega1);
Q_tau2 = M2Q(tau2,omega2) + M2Q(-tau2,omega1);
Q = Q_tau1+Q_tau2;

%% Dynamics
dL_dq = jacobian(L,q)';
dL_dqd= jacobian(L,dq)';

%% Equations of motion
eom = ddt(dL_dqd) - dL_dq +dD_dqd- Q;
A = jacobian(eom,ddq);
b = simplify( A*ddq - eom);

%% Helper functions for simulation
keypoints = [rO rB rC];
ddkeypoints = [ddrO ddrB ddrC];

matlabFunction(A,'file',['A_' name],'vars',{z p});
matlabFunction(b,'file',['b_' name],'vars',{z u p});
matlabFunction(keypoints,'file',['keypoints_' name],'vars',{z p});
matlabFunction(E,'file',['E_' name],'vars',{z p});
matlabFunction(V,'file',['V_' name],'vars',{z p});
