function [matrix] = inverse_kinematics(x,y,z,alpha,beta,gamma)

    if z < 0 %Check if the end-effector is above the robot base
        
        matrix = zeros(8,6);      
        fprintf('WARNING: No solutions were found for the given input arguments (z<0)\n');
        return;
    end
    
    if sqrt((x^2)+(y^2)+(z^2)) > 640 %Check if the end effector is within the limits of the arm's dimensions, when it is at full extension  
        
        matrix = zeros(8,6);
        fprintf('WARNING: No solutions were found for the given input arguments\n');
        return;
    end

%Computation of MatrixZYZ for the given Euler angles
    RotZYZ = [cos(alpha)*cos(beta)*cos(gamma)-sin(alpha)*sin(gamma), -cos(alpha)*cos(beta)*sin(gamma)-sin(alpha)*cos(gamma), cos(alpha)*sin(beta);
            sin(alpha)*cos(beta)*cos(gamma)+cos(alpha)*sin(gamma), -sin(alpha)*cos(beta)*sin(gamma)+cos(alpha)*cos(gamma), sin(alpha)*sin(beta);
            -sin(beta)*cos(gamma), sin(beta)*sin(gamma), cos(beta)];
          
       
    rot56 = [1,0,0;0,cos(pi/2),-sin(pi/2);0,sin(pi/2),cos(pi/2)]; %Rotation around X with an arbitrary angle
    rot05 = RotZYZ*((rot56)^-1); 
    
    p05 = [x;y;z] - rot05*[23.7;0;0]; %Joint 5 coordinates relative to the world frame
    x5 = p05(1);
    y5 = p05(2);
    z5 = p05(3);
     
    % --------- GEOMETRIC METHOD -----------
    %The two possible solutions for A1 
    A1 = [atan2(y5,x5);atan2(y5,x5) + pi];
    
    %Calculations executed to obtain A2
    Y_trig = sqrt(x5^2 + y5^2);
    Z_trig = z5-183; %Neglecting the distance between the base of the robot and joint 2
    a1 = 210; %Euclidean distance from joint 2 to joint 3
    a2 = sqrt((30^2)+(41.5+180)^2); %Euclidean distance between joints 3 and 5        
    
    tau1 = atan2(Y_trig/sqrt(Y_trig^2 + Z_trig^2),Z_trig/sqrt(Y_trig^2 + Z_trig^2));%Angle between Z0 (Z axis of the world frame) and a line that connects joints 2 and 5
    delta1 = atan2(sqrt(1-(((a2^2) - (a1^2) - (Y_trig^2 + Z_trig^2))/(-2*a1*sqrt(Y_trig^2 + Z_trig^2)))^2),((a2^2) - (a1^2) - (Y_trig^2 + Z_trig^2))/(-2*a1*sqrt(Y_trig^2 + Z_trig^2)));%Angle obtained from the law of cosines
    A2 = [tau1-delta1;tau1+delta1;-(tau1-delta1);-(tau1+delta1)];%The 4 possibilities for the angle of joint 2
    
    %Calculations executed to obtain A2
    difference = -0.1346; %Angle between the X axis of the joint 3 reference frame and a2
    delta2 = atan2(sqrt(1-(((Y_trig^2 + Z_trig^2)-(a1^2)-(a2^2))/(-2*a1*a2))^2),((Y_trig^2 + Z_trig^2)-(a1^2)-(a2^2))/(-2*a1*a2));%Angle obtained from the lasw of cosines
    tau2 = pi-delta2; %Angle between a2 and the Z axis of joint 3 reference frame
    A3 = [-pi/2 + tau2-difference;-pi/2-tau2-difference];%The 2 possibilities for the angle of joint 3
    

    % --------- ALGEBRAIC METHOD -----------
    
    counter = 1;
    i = 1; %Select the first A1 value 
      
    for j = 1:2 %Iterate through the last two values of A2     
      
        if j == 1 
            k = 1; %Pick the first value of A3
        else
            k = 2; %Pick the second value of A3
        end
    
        rot1 = [cos(A1(i)),-sin(A1(i)),0;sin(A1(i)),cos(A1(i)),0;0,0,1]; %Z rotation
        rot2 = [cos(A2(j)),0,sin(A2(j));0,1,0;-sin(A2(j)),0,cos(A2(j))]; %Y rotation
        rot3 = [cos(A3(k)),0,sin(A3(k));0,1,0;-sin(A3(k)),0,cos(A3(k))]; %Y rotation
        trans1 = [0;0;103];
        trans2 = [0;0;80];
        trans3 = [0;0;210];
        
        %Obtaining T01, T12, T23
        matrix1 = [rot1(1,1),rot1(1,2),rot1(1,3),trans1(1);rot1(2,1),rot1(2,2),rot1(2,3),trans1(2);rot1(3,1),rot1(3,2),rot1(3,3),trans1(3);0,0,0,1];
        matrix2 = [rot2(1,1),rot2(1,2),rot2(1,3),trans2(1);rot2(2,1),rot2(2,2),rot2(2,3),trans2(2);rot2(3,1),rot2(3,2),rot2(3,3),trans2(3);0,0,0,1];
        matrix3 = [rot3(1,1),rot3(1,2),rot3(1,3),trans3(1);rot3(2,1),rot3(2,2),rot3(2,3),trans3(2);rot3(3,1),rot3(3,2),rot3(3,3),trans3(3);0,0,0,1];
        %Obtaining T06, T03, T36
        T06 = [RotZYZ(1,1),RotZYZ(1,2),RotZYZ(1,3),x;RotZYZ(2,1),RotZYZ(2,2),RotZYZ(2,3),y;RotZYZ(3,1),RotZYZ(3,2),RotZYZ(3,3),z;0,0,0,1];
        T03 = matrix1*matrix2*matrix3;
        T36 = (T03^(-1))*T06;
    
        %Possible solutions for A4, A5 and A6 from the transformation matrix T36
        A5_1 = acos(T36(1,1)); 
        A5_2 = -acos(T36(1,1));
    
        A4_1 = real(asin(T36(2,1)/sin(A5_1)));
        A4_2 = real(asin(T36(2,1)/sin(A5_2)));
                                            %We use the real() function to prevent bugs related to
                                            %complex numbers like getting
                                            %-1.5703 + 0.0000i instead of -1.5703 due to Matlab approximations
        A6_1 = real(asin(T36(1,2)/sin(A5_1)));
        A6_2 = real(asin(T36(1,2)/sin(A5_2)));
    
        %We needed to group the solutions found for the angles, so we need to filter them through 
        %multiple conditions, in order to be coherent to the transformation matrix that was 
        %previously obtained and that's what we're doing during the following segment of code.
        
        if (sign(T36(3,1)) == sign(-cos(A4_1)*sin(A5_1)) && sign(T36(3,1)) == -cos(A4_2)*sin(A5_2))
                
        A4(counter) = A4_1;
        A5(counter) = A5_1;
        
        if (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_2))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;
        elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = pi-A6_2;
        elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_2)))
            A6(counter) = pi-A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_1))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;   
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = pi-A6_1;   
         elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_1)))
            A6(counter) = pi-A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;  
        end
            
        A4(counter) = A4_2;
        A5(counter) = A5_2;
        counter=counter+1;
        
        elseif (sign(T36(3,1)) == sign(-cos(A4_1)*sin(A5_1)))    
            
            A4(counter) = A4_1;
            A5(counter) = A5_1;
            
            if (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_2))
                A6(counter) = A6_1;
                counter = counter+1;        
                A6(counter) = A6_2;
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)))
                A6(counter) = A6_1;
                counter = counter+1;        
                A6(counter) = pi-A6_2;
            elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_2)))
                A6(counter) = pi-A6_1;
                counter = counter+1;        
                A6(counter) = A6_2;
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_1))
                A6(counter) = A6_2;
                counter = counter+1;        
                A6(counter) = A6_1;   
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)))
                A6(counter) = A6_2;
                counter = counter+1;        
                A6(counter) = pi-A6_1;   
            elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_1)))
                A6(counter) = pi-A6_2;
                counter = counter+1;        
                A6(counter) = A6_1;  
            end
                
            A4(counter) = pi-A4_2;
            A5(counter) = A5_2;
            counter=counter+1;
            
        elseif (sign(T36(3,1)) == sign(-cos(A4_2)*sin(A5_2)))   
            
            A4(counter) = pi-A4_1;
            A5(counter) = A5_1;
            
            if (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_2))
                A6(counter) = A6_1;
                counter = counter+1;        
                A6(counter) = A6_2;
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)))
                A6(counter) = A6_1;
                counter = counter+1;        
                A6(counter) = pi-A6_2;
            elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_2)))
                A6(counter) = pi-A6_1;
                counter = counter+1;        
                A6(counter) = A6_2;
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_1))
                A6(counter) = A6_2;
                counter = counter+1;        
                A6(counter) = A6_1;   
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)))
                A6(counter) = A6_2;
                counter = counter+1;        
                A6(counter) = pi-A6_1;   
            elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_1)))
                A6(counter) = pi-A6_2;
                counter = counter+1;        
                A6(counter) = A6_1;  
            end
                
            A4(counter) = A4_2;
            A5(counter) = A5_2;
            counter=counter+1;
            
        elseif (sign(T36(3,1)) == sign(-cos(A4_1)*sin(A5_2)) && sign(T36(3,1))== sign(-cos(A4_2)*sin(A5_1)))
        
            A4(counter) = A4_1;
            A5(counter) = A5_2;
        
            if (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_2))
                A6(counter) = A6_2;
                counter = counter+1;        
                A6(counter) = A6_1;
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)))
                A6(counter) = pi-A6_2;
                counter = counter+1;        
                A6(counter) = A6_1;
            elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_2)))
                A6(counter) = A6_2;
                counter = counter+1;        
                A6(counter) = pi-A6_1;
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_1))
                A6(counter) = A6_1;
                counter = counter+1;        
                A6(counter) = A6_2;   
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)))
                A6(counter) = pi-A6_1;
                counter = counter+1;        
                A6(counter) = A6_2;   
            elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_1)))
                A6(counter) = A6_1;
                counter = counter+1;        
                A6(counter) = pi-A6_2;  
            end
            
            A4(counter) = A4_2;
            A5(counter) = A5_1;
            counter=counter+1;
            
        elseif (sign(T36(3,1)) == sign(-cos(A4_1)*sin(A5_2))) 
            
            A4(counter) = A4_1;
            A5(counter) = A5_2;
            
            if (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_2))
                A6(counter) = A6_2;
                counter = counter+1;        
                A6(counter) = A6_1;
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)))
                A6(counter) = pi-A6_2;
                counter = counter+1;        
                A6(counter) = A6_1;
            elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_2)))
                A6(counter) = A6_2;
                counter = counter+1;        
                A6(counter) = pi-A6_1;
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_1))
                A6(counter) = A6_1;
                counter = counter+1;        
                A6(counter) = A6_2;   
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)))
                A6(counter) = pi-A6_1;
                counter = counter+1;        
                A6(counter) = A6_2;   
            elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_1)))
                A6(counter) = A6_1;
                counter = counter+1;        
                A6(counter) = pi-A6_2;  
            end
            
            A4(counter) = pi-A4_2;
            A5(counter) = A5_1;
            counter=counter+1;
            
        elseif (sign(T36(3,1)) == sign(-cos(A4_2)*sin(A5_1))) 
            
            A4(counter) = pi-A4_1;
            A5(counter) = A5_2;
            
            if (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_2))
                A6(counter) = A6_2;
                counter = counter+1;        
                A6(counter) = A6_1;
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)))
                A6(counter) = pi-A6_2;
                counter = counter+1;        
                A6(counter) = A6_1;
            elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_2)))
                A6(counter) = A6_2;
                counter = counter+1;        
                A6(counter) = pi-A6_1;
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_1))
                A6(counter) = A6_1;
                counter = counter+1;        
                A6(counter) = A6_2;   
            elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)))
                A6(counter) = pi-A6_1;
                counter = counter+1;        
                A6(counter) = A6_2;   
            elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_1)))
                A6(counter) = A6_1;
                counter = counter+1;        
                A6(counter) = pi-A6_2;  
            end
            
            A4(counter) = A4_2;
            A5(counter) = A5_1;
            counter=counter+1;   

        end

  end
  
  i = 2; %Select the second A1 value 
  for j = 3:4 %Iterate through the last two values of A2
          
    if j == 3
        k = 2; %Pick the second value of A3
    else
        k = 1; %Pick the first value of A3
    end

    rot1 = [cos(A1(i)),-sin(A1(i)),0;sin(A1(i)),cos(A1(i)),0;0,0,1]; %Z rotation
    rot2 = [cos(A2(j)),0,sin(A2(j));0,1,0;-sin(A2(j)),0,cos(A2(j))]; %Y rotation
    rot3 = [cos(A3(k)),0,sin(A3(k));0,1,0;-sin(A3(k)),0,cos(A3(k))]; %Y rotation
    trans1 = [0;0;103];
    trans2 = [0;0;80];
    trans3 = [0;0;210];
    
    %Obtaining T01, T12, T23
    matrix1 = [rot1(1,1),rot1(1,2),rot1(1,3),trans1(1);rot1(2,1),rot1(2,2),rot1(2,3),trans1(2);rot1(3,1),rot1(3,2),rot1(3,3),trans1(3);0,0,0,1];
    matrix2 = [rot2(1,1),rot2(1,2),rot2(1,3),trans2(1);rot2(2,1),rot2(2,2),rot2(2,3),trans2(2);rot2(3,1),rot2(3,2),rot2(3,3),trans2(3);0,0,0,1];
    matrix3 = [rot3(1,1),rot3(1,2),rot3(1,3),trans3(1);rot3(2,1),rot3(2,2),rot3(2,3),trans3(2);rot3(3,1),rot3(3,2),rot3(3,3),trans3(3);0,0,0,1]; 
    %Obtaining T06, T03, T36
    T06 = [RotZYZ(1,1),RotZYZ(1,2),RotZYZ(1,3),x;RotZYZ(2,1),RotZYZ(2,2),RotZYZ(2,3),y;RotZYZ(3,1),RotZYZ(3,2),RotZYZ(3,3),z;0,0,0,1];
    T03 = matrix1*matrix2*matrix3;
    T36 = (T03^(-1))*T06;
    
    A5_1 = acos(T36(1,1));
    A5_2 = -acos(T36(1,1));
    
    A4_1 = real(asin(T36(2,1)/sin(A5_1)));
    A4_2 = real(asin(T36(2,1)/sin(A5_2))); 
                                            %We use the real() function to prevent bugs related to
                                            %complex numbers like getting
                                            %-1.5703 + 0.0000i instead of -1.5703
    A6_1 = real(asin(T36(1,2)/sin(A5_1)));
    A6_2 = real(asin(T36(1,2)/sin(A5_2)));

    %Once again, we needed to group the solutions found for the angles, so we need to filter them through 
    %multiple conditions, in order to be coherent to the transformation matrix that was 
    %previously obtained and that's what we're doing during the following segment of code.
    
    if (sign(T36(3,1)) == sign(-cos(A4_1)*sin(A5_1)) && sign(T36(3,1)) == -cos(A4_2)*sin(A5_2))
                
        A4(counter) = A4_1;
        A5(counter) = A5_1;
       
        if (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_2))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;
        elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = pi-A6_2;
        elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_2)))
            A6(counter) = pi-A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_1))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;   
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = pi-A6_1;   
         elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_1)))
            A6(counter) = pi-A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;  
        end
            
        A4(counter) = A4_2;
        A5(counter) = A5_2;
        counter=counter+1;
        
    elseif (sign(T36(3,1)) == sign(-cos(A4_1)*sin(A5_1)))    
        
        A4(counter) = A4_1;
        A5(counter) = A5_1;

        if (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_2))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;
        elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = pi-A6_2;
        elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_2)))
            A6(counter) = pi-A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_1))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;   
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = pi-A6_1;   
         elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_1)))
            A6(counter) = pi-A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;  
        end
            
        A4(counter) = pi-A4_2;
        A5(counter) = A5_2;
        counter=counter+1;
        
    elseif (sign(T36(3,1)) == sign(-cos(A4_2)*sin(A5_2)))   
        
        A4(counter) = pi-A4_1;
        A5(counter) = A5_1;
        
        if (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_2))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;
        elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = pi-A6_2;
        elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_2)))
            A6(counter) = pi-A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_1))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;   
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = pi-A6_1;   
         elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_1)))
            A6(counter) = pi-A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;  
        end
            
        A4(counter) = A4_2;
        A5(counter) = A5_2;
        counter=counter+1;
        
    elseif (sign(T36(3,1)) == sign(-cos(A4_1)*sin(A5_2)) && sign(T36(3,1))== sign(-cos(A4_2)*sin(A5_1)))
      
        A4(counter) = A4_1;
        A5(counter) = A5_2;

        if (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_2))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;
        elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)))
            A6(counter) = pi-A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;
        elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_2)))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = pi-A6_1;
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_1))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;   
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)))
            A6(counter) = pi-A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;   
         elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_1)))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = pi-A6_2;  
         end
        
        A4(counter) = A4_2;
        A5(counter) = A5_1;
        counter=counter+1;
        
    elseif (sign(T36(3,1)) == sign(-cos(A4_1)*sin(A5_2))) 
        
        A4(counter) = A4_1;
        A5(counter) = A5_2;
        
        if (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_2))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;
        elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)))
            A6(counter) = pi-A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;
        elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_2)))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = pi-A6_1;
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_1))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;   
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)))
            A6(counter) = pi-A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;   
         elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_1)))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = pi-A6_2;  
         end

        A4(counter) = pi-A4_2;
        A5(counter) = A5_1;
        counter=counter+1;
        
     elseif (sign(T36(3,1)) == sign(-cos(A4_2)*sin(A5_1))) 
         
        A4(counter) = pi-A4_1;
        A5(counter) = A5_2;
        
        if (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_2))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;
        elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_1)))
            A6(counter) = pi-A6_2;
            counter = counter+1;        
            A6(counter) = A6_1;
        elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_2)))
            A6(counter) = A6_2;
            counter = counter+1;        
            A6(counter) = pi-A6_1;
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)) && sign(T36(1,3)) == sin(A5_2)*cos(A6_1))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;   
         elseif (sign(T36(1,3)) == sign(sin(A5_1)*cos(A6_2)))
            A6(counter) = pi-A6_1;
            counter = counter+1;        
            A6(counter) = A6_2;   
         elseif (sign(T36(1,3)) == sign(sin(A5_2)*cos(A6_1)))
            A6(counter) = A6_1;
            counter = counter+1;        
            A6(counter) = pi-A6_2;  
         end
     
        A4(counter) = A4_2;
        A5(counter) = A5_1;
        counter=counter+1;   

    end

  end   
  
 
    %SOLUTIONS MATRIX
    
    matrix(1,:) = [A1(1),A2(1),A3(1),A4(1),A5(1),A6(1)];
    matrix(2,:) = [A1(1),A2(1),A3(1),A4(2),A5(2),A6(2)];
    matrix(3,:) = [A1(1),A2(2),A3(2),A4(3),A5(3),A6(3)];
    matrix(4,:) = [A1(1),A2(2),A3(2),A4(4),A5(4),A6(4)];
    matrix(5,:) = [A1(2),A2(3),A3(2),A4(5),A5(5),A6(5)];
    matrix(6,:) = [A1(2),A2(3),A3(2),A4(6),A5(6),A6(6)];
    matrix(7,:) = [A1(2),A2(4),A3(1),A4(7),A5(7),A6(7)];
    matrix(8,:) = [A1(2),A2(4),A3(1),A4(8),A5(8),A6(8)];
    
        
    %UNCOMMENT TO COMPARE THE OBTAINED T06 MATRICES, ONE FOR EACH SOLUTION, WITH THE ORIGINAL T06 MATRIX
    
%     fprintf('(T06) Base tool transformation matrix:\n');
%     T06 %Print the original T06 matrix
%     fprintf('\n');
%     for i = 1:8 %Print the T06 matrix obtained for all the possible solutions
%         fprintf('Solution %d',i);
%         output=T06_checker(matrix(i,1),matrix(i,2),matrix(i,3),matrix(i,4),matrix(i,5),matrix(i,6));
%     end
end