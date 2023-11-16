clc;
clear;
%syms Theta1 Theta2 Theta3 Theta4 Theta5 Theta6 real

%format longg;

Theta1 = 0;
Theta2 = 63.4349; 
Theta3 = 73.6347;
Theta4 = -114.0662 ;
Theta5 = 40.4316;
Theta6 = 0;

%arm above the block by 1 inches
%Theta1 = 0;
%Theta2 = 0; 
%Theta3 = 71.23;
%Theta4 = -123.53;
%Theta5 = 52.3;
%Theta6 = 0;

%One block in between 
%Theta1 = 0;
%Theta2 = 0; 
%Theta3 = 62.18;
%Theta4 = -124.37;
%Theta5 = 62.18;
%Theta6 = 0;

%two leg next to each other
%Theta1 = 0;
%Theta2 = 0; 
%Theta3 = 76.515;
%Theta4 = - 153.03;
%Theta5 = 76.515;
%Theta6 = 0;

l0 = 3.125;
l1 = 1; 
l2 = 6.432;
l3 = 6.432;
l4 = 1;
l5 = 3.125;

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
theta6 = deg2rad(Theta6);
d6 = l5;
a6 = 0;
alpha6 = deg2rad(0);
T56 = tdh(theta6, d6, a6, alpha6);

disp('T01 = '); disp(T01);
disp('T12 = '); disp(T12);
disp('T23 = '); disp(T23);
disp('T34 = '); disp(T34);
disp('T45 = '); disp(T45);
disp('T56 = '); disp(T56);

T06 = T01*T12*T23*T34*T45*T45_v2*T56;

disp('T06 = '); (disp(T06));

position = T06(1:3, 4);

orientation = T06(1:3, 1:3);

% Display pos and orientation
disp('Position of the end-effector in inches:'); disp(position);
disp('Orientation of the end-effector:'); disp(orientation);
   
% Stick model simulation
figure;
hold on;
grid on;
axis equal;

% Base coordinate
O = [0; 0; 0; 1];

% Link 1 coordinate
O1 = T01 * O;

% Link 2 coordinate
O2 = T01 * T12 * O;

% Link 3 coordinate
O3 = T01 * T12 * T23 * O;

% Link 4 coordinate
O4 = T01 * T12 * T23 * T34 * O;

% Link 5 coordinate
O5 = T01 * T12 * T23 * T34 * T45 * T45_v2 * O;

% Link 6 coordinate (End effector)
O6 = T06 * O;

% Plotting each link
plot3([O(1), O1(1)], [O(2), O1(2)], [O(3), O1(3)], 'b', 'LineWidth', 2); % Link 0-1
plot3([O1(1), O2(1)], [O1(2), O2(2)], [O1(3), O2(3)], 'r', 'LineWidth', 2); % Link 1-2
plot3([O2(1), O3(1)], [O2(2), O3(2)], [O2(3), O3(3)], 'g', 'LineWidth', 2); % Link 2-3
plot3([O3(1), O4(1)], [O3(2), O4(2)], [O3(3), O4(3)], 'y', 'LineWidth', 2); % Link 3-4
plot3([O4(1), O5(1)], [O4(2), O5(2)], [O4(3), O5(3)], 'm', 'LineWidth', 2); % Link 4-5
plot3([O5(1), O6(1)], [O5(2), O6(2)], [O5(3), O6(3)], 'k', 'LineWidth', 2); % Link 5-6

xlabel('X Axis');
ylabel('Y Axis');
zlabel('Z Axis');
title('Stick Model Simulation of Robot');

hold off;

function T = tdh(theta, d, a, alpha)
%    theta = sym(theta2);
%    alpha = sym(alpha2);


    T = [cos(theta) , (-sin(theta)*cos(alpha)) , (sin(theta)*sin(alpha)) , (a*cos(theta)) ;
        sin(theta), (cos(theta)*cos(alpha) ), (-cos(theta)*sin(alpha) ) , (a*sin(theta)) ;
         0, sin(alpha) , cos(alpha) , d, ;
         0,0,0,1];
end



