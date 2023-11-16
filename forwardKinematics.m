function T06 = forwardKinematics(Theta1, Theta2, Theta3, Theta4, Theta5, l0, l1, l2, l3, l4, l5)
    % Convert degrees to radians for all joint angle

% Link 0 to Link 1
theta1 = deg2rad(Theta1);  
d1 = l0; %inches
a1 = 0;
alpha1 = deg2rad(0);
T01 = tdh(theta1, d1, a1, alpha1);
    
% Link 1 to Link 2
theta2 = deg2rad(Theta2);  
d2 = l1;
a2 = 0;
alpha2 = deg2rad(90);
T12 = tdh(theta2, d2, a2, alpha2);

% Link 2 to Link 3
theta3 = deg2rad(Theta3);  
d3 = 0;
a3 = l2;
alpha3 = deg2rad(0);
T23 = tdh(theta3, d3, a3, alpha3);

% Link 3 to Link 4
theta4 = deg2rad(Theta4);  
d4 = 0;  
a4 = l3;
alpha4 = deg2rad(0);
T34 = tdh(theta4, d4, a4, alpha4);

% Link 4 to Link 5
theta5 = deg2rad(Theta5);  
d5 = 0;
a5 = 0;
alpha5 = deg2rad(90);
T45 = tdh(theta5, d5, a5, alpha5);

theta5_v2 = deg2rad(0);  
d5_v2 = l4;
a5_v2 = 0;
alpha5_v2 = deg2rad(0);
T45_v2 = tdh(theta5_v2, d5_v2, a5_v2, alpha5_v2);

% Link 5 to Link 6 
theta6 = deg2rad(0);
d6 = l5;
a6 = 0;
alpha6 = deg2rad(0);
T56 = tdh(theta6, d6, a6, alpha6);
    
    % Compute the final transformation matrix from base to end effector
    T06 = T01 * T12 * T23 * T34 * T45 * T45_v2 * T56;
end


function T = tdh(theta, d, a, alpha)
%    theta = sym(theta2);
%    alpha = sym(alpha2);


    T = [cos(theta) , (-sin(theta)*cos(alpha)) , (sin(theta)*sin(alpha)) , (a*cos(theta)) ;
        sin(theta), (cos(theta)*cos(alpha) ), (-cos(theta)*sin(alpha) ) , (a*sin(theta)) ;
         0, sin(alpha) , cos(alpha) , d, ;
         0,0,0,1];
end