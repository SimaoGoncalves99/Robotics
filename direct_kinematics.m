function [x,y,z,alpha,beta,gamma] = direct_kinematics(A1,A2,A3,A4,A5,A6)

    %Definition the rotation matrixes for joints 1 to 6, using the respective 
    %angles that were chosen by the user.

    rot1 = [cos(A1),-sin(A1),0;sin(A1),cos(A1),0;0,0,1]; %Z rotation
    rot2 = [cos(A2),0,sin(A2);0,1,0;-sin(A2),0,cos(A2)]; %Y rotation
    rot3 = [cos(A3),0,sin(A3);0,1,0;-sin(A3),0,cos(A3)]; %Y rotation
    rot4 = [1,0,0;0,cos(A4),-sin(A4);0,sin(A4),cos(A4)]; %X rotation 
    rot5 = [cos(A5),0,sin(A5);0,1,0;-sin(A5),0,cos(A5)]; %Y rotation
    rot6 = [1,0,0;0,cos(A6),-sin(A6);0,sin(A6),cos(A6)]; %X rotation 
    
    %The resting position of the arm is the following:
    %           z
    %           ^
    %            ____________
    %           |
    %           |
    %           |
    %           |
    %           |                 ------->x
    %
    %Definition of the translation matrixes. Here, we use the Nyrio One's dimensions, 
    %even though we DON'T consider the final 5.5mm.
    trans1 = [0;0;103];
    trans2 = [0;0;80];
    trans3 = [0;0;210];
    trans4 = [41.5;0;30];
    trans5 = [180;0;0];
    trans6 = [23.7;0;0];
    
    %Obtaining the transformation matrix and the end effector position 
    matrix1 = [rot1(1,1),rot1(1,2),rot1(1,3),trans1(1);rot1(2,1),rot1(2,2),rot1(2,3),trans1(2);rot1(3,1),rot1(3,2),rot1(3,3),trans1(3);0,0,0,1];
    matrix2 = [rot2(1,1),rot2(1,2),rot2(1,3),trans2(1);rot2(2,1),rot2(2,2),rot2(2,3),trans2(2);rot2(3,1),rot2(3,2),rot2(3,3),trans2(3);0,0,0,1];
    matrix3 = [rot3(1,1),rot3(1,2),rot3(1,3),trans3(1);rot3(2,1),rot3(2,2),rot3(2,3),trans3(2);rot3(3,1),rot3(3,2),rot3(3,3),trans3(3);0,0,0,1];
    matrix4 = [rot4(1,1),rot4(1,2),rot4(1,3),trans4(1);rot4(2,1),rot4(2,2),rot4(2,3),trans4(2);rot4(3,1),rot4(3,2),rot4(3,3),trans4(3);0,0,0,1];
    matrix5 = [rot5(1,1),rot5(1,2),rot5(1,3),trans5(1);rot5(2,1),rot5(2,2),rot5(2,3),trans5(2);rot5(3,1),rot5(3,2),rot5(3,3),trans5(3);0,0,0,1];
    matrix6 = [rot6(1,1),rot6(1,2),rot6(1,3),trans6(1);rot6(2,1),rot6(2,2),rot6(2,3),trans6(2);rot6(3,1),rot6(3,2),rot6(3,3),trans6(3);0,0,0,1];
    
    rot_matrix = rot1*rot2*rot3*rot4*rot5*rot6;
    transf_matrix = matrix1*matrix2*matrix3*matrix4*matrix5*matrix6;

    position = transf_matrix*[0;0;0;1];

    x = position(1);
    y = position(2);
    z = position(3);
    
    %Euler Angles (alpha, beta, gamma) - In order to obtain the orientation of the end effector 
    %relative to the world frame, we need to calculate the euler angles, which are defined by the following
    
    beta = atan2(-sqrt((rot_matrix(1,3)^2)+(rot_matrix(2,3)^2)),rot_matrix(3,3));
    
    %Beta2
    if sin(beta) > 0
        gamma = atan2(rot_matrix(3,2),-rot_matrix(3,1));
        alpha = atan2(rot_matrix(2,3),rot_matrix(1,3));
        
    elseif sin(beta) < 0
        gamma = atan2(-rot_matrix(3,2),rot_matrix(3,1));
        alpha = atan2(-rot_matrix(2,3),-rot_matrix(1,3));
        
    elseif sin(beta) == 0 && cos(beta) == 1 %There is more than one possible solution (Rank deficient)
        
        A = [1,1;1,1;1,1;1,1];
        b = [atan2(rot_matrix(2,1),rot_matrix(1,1));atan2(-rot_matrix(1,2),rot_matrix(2,2));atan2(rot_matrix(2,1),rot_matrix(2,2));atan2(-rot_matrix(1,2),rot_matrix(1,1))];
        Y = A\b;
        alpha= Y(1);
        gamma = Y(2);
        
    elseif sin(beta) == 0 && cos(beta) == -1 %There is more than one possible solution (Rank deficient)
        A = [1,-1;1,-1;1,-1;1,-1];
        b = [atan2(-rot_matrix(2,1),-rot_matrix(1,1));atan2(-rot_matrix(1,2),rot_matrix(2,2));atan2(-rot_matrix(2,1),rot_matrix(2,2));atan2(-rot_matrix(1,2),-rot_matrix(1,1))];
        Y = A\b;
        alpha = Y(1);
        gamma = Y(2);
    
    else
        alpha = "Error";
        beta = "Error";
        gamma = "Error";
    end
    
    end