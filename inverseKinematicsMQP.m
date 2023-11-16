function [theta1, theta2, theta3, theta4, theta5] = inverseKinematicsMQP(Px, Py, Pz)
    L_base = 3.125; % base to joint 1
    L1 = 1;         % Joint 1 to Joint 2
    L2 = 6.432;     % Joint 2 to Joint 3
    L3 = 6.432;     % Joint 3 to Joint 4
    L4 = 1;         % Joint 4 to Joint 5
    L_endEffector = 3.125; % Joint 5 to EE

    % adjust Z axes of the end-effector 
    Pz_adjusted = -Pz;

    % Joint 1 only for rotation 
    theta1 = atan2(Py, Px);

    % Calculate the pose in the XZ-plane
    Wx = sqrt(Px^2 + Py^2);
    Wz = Pz_adjusted ;

    % Joint 2 to the center of the wrist 
    D = sqrt(Wx^2 + Wz^2);

    % Is it possible to have this config 
    if D > (L2 + L3 )
        error('Position is not reachable');
    end

    % Solve joints 2 and 3
    cosTheta2 = (L2^2 + D^2 - L3^2) / (2 * L2 * D);
    theta2 = atan2(Wz, Wx) - acos(cosTheta2);

    cosTheta3 = (L2^2 + L3^2 - D^2) / (2 * L2 * L3);
    theta3 = acos(cosTheta3) - pi;

    % joint 4
    theta4 = -theta2 + theta3;

    % Joint 5 doesn't effect the pose 
    theta5 = 0;

    % radians to degrees 
    theta1 = rad2deg(theta1);
    theta2 = rad2deg(-theta2);
    theta3 = rad2deg(theta3);
    theta4 = rad2deg(-theta4);
    theta5 = rad2deg(theta5);
end
