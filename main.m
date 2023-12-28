%% Direct kinematics
% direct_kinematics function format: [x,y,z,alpha,beta,gamma] = direct_kinematics(A1,A2,A3,A4,A5,A6)

[x,y,z,alpha,beta,gamma] = direct_kinematics(0,0,-pi/2,0,0,0);
end_effector = [x;y;z];
euler = [alpha;beta;gamma];

result_direct = [x;y;z;alpha;beta;gamma];

VarNames = {'x', 'y', 'z', 'alpha', 'beta', 'gamma'};

fprintf(1, '  \t%s\t\t%s\t\t%s\t\t%s\t%s\t%s\n', VarNames{:})
fprintf(1, '\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n', result_direct')
fprintf('\n');

%% Inverse kinematics

% Inverse_kinematics function format: [matrix] = inverse_kinematics(x,y,z,alpha,beta,gamma)
% Ignoring the 5.5mm on the end effector

[matrix] = inverse_kinematics(x,y,z,alpha,beta,gamma);

result_inverse = matrix(:,:);
VarNames = {'A1', 'A2', 'A3', 'A4', 'A5', 'A6'};

fprintf(1, '  \t%s\t\t%s\t\t%s\t\t%s\t\t%s\t\t%s\n', VarNames{:})
fprintf(1, '\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n', matrix')
fprintf('\n');


%% Confirming Inverse Kinematics
%direct_kinematics function format: [x,y,z,alpha,beta,gamma] = direct_kinematics(A1,A2,A3,A4,A5,A6)
%For every inverse kinematics solution, we compute the direct kinematics

[x1,y1,z1,alpha1,beta1,gamma1] = direct_kinematics(matrix(1,1),matrix(1,2),matrix(1,3),matrix(1,4),matrix(1,5),matrix(1,6));
end_effector1 = [x1;y1;z1];
euler_1 = [alpha1,beta1,gamma1];

[x2,y2,z2,alpha2,beta2,gamma2] = direct_kinematics(matrix(2,1),matrix(2,2),matrix(2,3),matrix(2,4),matrix(2,5),matrix(2,6));
end_effector2 = [x2;y2;z2];
euler_2 = [alpha2,beta2,gamma2];

[x3,y3,z3,alpha3,beta3,gamma3] = direct_kinematics(matrix(3,1),matrix(3,2),matrix(3,3),matrix(3,4),matrix(3,5),matrix(3,6));
end_effector3 = [x3;y3;z3];
euler_3 = [alpha3,beta3,gamma3];

[x4,y4,z4,alpha4,beta4,gamma4] = direct_kinematics(matrix(4,1),matrix(4,2),matrix(4,3),matrix(4,4),matrix(4,5),matrix(4,6));
end_effector4 = [x4;y4;z4];
euler_4 = [alpha4,beta4,gamma4];

[x5,y5,z5,alpha5,beta5,gamma5] = direct_kinematics(matrix(5,1),matrix(5,2),matrix(5,3),matrix(5,4),matrix(5,5),matrix(5,6));
end_effector5 = [x5;y5;z5];
euler_5 = [alpha5,beta5,gamma5];

[x6,y6,z6,alpha6,beta6,gamma6] = direct_kinematics(matrix(6,1),matrix(6,2),matrix(6,3),matrix(6,4),matrix(6,5),matrix(6,6));
end_effector6 = [x6;y6;z6];
euler_6 = [alpha6,beta6,gamma6];

[x7,y7,z7,alpha7,beta7,gamma7] = direct_kinematics(matrix(7,1),matrix(7,2),matrix(7,3),matrix(7,4),matrix(7,5),matrix(7,6));
end_effector7 = [x7;y7;z7];
euler_7 = [alpha7,beta7,gamma7];

[x8,y8,z8,alpha8,beta8,gamma8] = direct_kinematics(matrix(8,1),matrix(8,2),matrix(8,3),matrix(8,4),matrix(8,5),matrix(8,6));
end_effector8 = [x8;y8;z8];
euler_8 = [alpha8,beta8,gamma8];

end_effector_tot = [end_effector1,end_effector2,end_effector3,end_effector4,end_effector5,end_effector6,end_effector7,end_effector8];
euler_tot = [euler_1,euler_2,euler_3,euler_4,euler_5,euler_6,euler_7,euler_8];
VarNames = {'S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'S7', 'S8'};
VarNames1 = {'Eu1', 'Eu2', 'Eu3', 'Eu4', 'Eu5', 'Eu6', 'Eu7', 'Eu8'};

%Print the T06 matrix

fprintf(1, '  \t%s\t\t%s\t\t%s\t\t%s\t\t%s\t\t%s\t\t%s\t\t%s\n', VarNames{:})
fprintf(1, '\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n', end_effector_tot')
fprintf(1, '  \t%s\t\t%s\t\t%s\t\t%s\t\t%s\t\t%s\t\t%s\t\t%s\n', VarNames1{:})
fprintf(1, '\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n', euler_tot')