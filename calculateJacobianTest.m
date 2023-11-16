
Theta1 = 0;
Theta2 = 0;
Theta3 = 71.23;
Theta4 = -123.53;
Theta5 = 52.3;

l0 = 3.125; % Base height
l1 = 1;     % Link 1 length
l2 = 6.432; % Link 2 length
l3 = 6.432; % Link 3 length
l4 = 1;     % Link 4 length (wrist)
l5 = 3.125; % End effector offset

% forwardKinematics stuff
T06 = forwardKinematics(Theta1, Theta2, Theta3, Theta4, Theta5, l0, l1, l2, l3, l4, l5);

fprintf('End-effector position:\n');
disp(T06(1:3, 4));

fprintf('End-effector orientation:\n');
disp(T06(1:3, 1:3));

% Calculate jacobian 
J = computeJacobian(Theta1, Theta2, Theta3, Theta4, Theta5, l0, l1, l2, l3, l4, l5);

%Jacobian matrix
fprintf('Jacobian matrix:\n');
disp(J);
