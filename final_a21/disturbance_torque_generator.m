function t_d = disturbance_torque_generator(x)
% Joint Velocities
dq_1 = x(1);
dq_2 = x(2);
dq_3 = x(3);
dq_4 = x(4);
%%%%%%%%%%%%%%%%%%%%%%%%%
% Coulomb Friction
f_c1 = x(5);
f_c2 = x(6);
f_c3 = x(7);
f_c4 = x(8);
%%%%%%%%%%%%%%%%%%%%%%%%%
% Static Friction
f_s1 = x(9);
f_s2 = x(10);
f_s3 = x(11);
f_s4 = x(12);
%%%%%%%%%%%%%%%%%%%%%%%%%
% Viscous Friction
f_v1 = x(13);
f_v2 = x(14);
f_v3 = x(15);
f_v4 = x(16);
%%%%%%%%%%%%%%%%%%%%%%%%%
% Stribeck Parameter
v_s1 = x(17);
v_s2 = x(18);
v_s3 = x(19);
v_s4 = x(20);
%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint Positions
q_1 = x(21);
q_2 = x(22);
q_3 = x(23);
q_4 = x(24);
%%%%%%%%%%%%%%%%%%%%%%%%%
% Links Length
a_1 = x(25);
a_2 = x(26);
%%%%%%%%%%%%%%%%%%%%%%%%%
% F-payload
f_payload = x(27);
g = 9.8;

%if(v_s1 ~= 0)
%%%%%%%%%%%%%%%%%%%%%%%%%
% Friction Torque
t_f = zeros(4,1);
t_f(1,1) = f_c1*sign(dq_1)*(1-exp(-(dq_1^2)/(v_s1^2))) + f_s1*sign(dq_1)*exp(-(dq_1^2)/(v_s1^2)) + f_v1*dq_1;
% disp(f_c1)
% disp(dq_1)
% disp(v_s1)
% disp(f_c1*sign(dq_1))
% disp(1-exp(-(dq_1^2)/(v_s1^2)))
% disp("%%%%%%%%%%%%%%%%%%%%%%%%")
t_f(2,1) = f_c2*sign(dq_2)*(1-exp(-(dq_2^2)/(v_s2^2))) + f_s2*sign(dq_2)*exp(-(dq_2^2)/(v_s2^2)) + f_v2*dq_2;
t_f(3,1) = f_c3*sign(dq_3)*(1-exp(-(dq_3^2)/(v_s3^2))) + f_s3*sign(dq_3)*exp(-(dq_3^2)/(v_s3^2)) + f_v3*dq_3;
t_f(4,1) = f_c4*sign(dq_4)*(1-exp(-(dq_4^2)/(v_s4^2))) + f_s4*sign(dq_4)*exp(-(dq_4^2)/(v_s4^2)) + f_v4*dq_4;
%%%%%%%%%%%%%%%%%%%%%%%%%
% Jacobian Matrix
j = zeros(4,4);
j(1,1) = -a_1*sin(q_1)-a_2*sin(q_1+q_2);
j(1,2) = -a_2*sin(q_1+q_2);
j(1,3) = 0;
j(1,4) = 0;
j(2,1) = -a_1*cos(q_1)+a_2*cos(q_1+q_2);
j(2,2) = a_2*cos(q_1+q_2);
j(2,3) = 0;
j(2,4) = 0;
j(3,1) = 0;
j(3,2) = 0;
j(3,3) = -1;
j(3,4) = 0;
j(4,1) = 1;
j(4,2) = 1;
j(4,3) = 0;
j(4,4) = 1;
%end
%%%%%%%%%%%%%%%%%%%%%%%%%
% Torque Of Disturbance
t_d = t_f+j'*[0;0;0;0];

end