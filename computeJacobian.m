function J = computeJacobian(Theta1, Theta2, Theta3, Theta4, Theta5, l0, l1, l2, l3, l4, l5)

    delta = 1e-6;
    
   
    T_initial = forwardKinematics(Theta1, Theta2, Theta3, Theta4, Theta5, l0, l1, l2, l3, l4, l5);
    pos_initial = T_initial(1:3, 4);
    
    
    J = zeros(6, 5);
    
    % Calculate Jacobian for each joint 
    for i = 1:5
        
        Theta = [Theta1, Theta2, Theta3, Theta4, Theta5];
        
        
        Theta(i) = Theta(i) + delta;
        
        % new orientation 
        T_new = forwardKinematics(Theta(1), Theta(2), Theta(3), Theta(4), Theta(5), l0, l1, l2, l3, l4, l5);
        pos_new = T_new(1:3, 4);
        
        diff = (pos_new - pos_initial) / delta;
        
        
        J(1:3, i) = diff;
    end
    
end
