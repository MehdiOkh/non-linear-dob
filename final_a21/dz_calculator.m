function [dz] = dz_calculator(x)

% Joint Positions
q_1 = x(1);
q_2 = x(2);
q_3 = x(3);
q_4 = x(4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint Velocities
dq_1 = x(5);
dq_2 = x(6);
dq_3 = x(7);
dq_4 = x(8);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p_1 = x(9);
p_2 = x(10);
p_3 = x(11);
p_4 = x(12);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
z_1 = x(13);
z_2 = x(14);
z_3 = x(15);
z_4 = x(16);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot Parameters
% m_1 =15;
% m_2 = 12;
% m_3 = 3;
% m_4 = 3;
% I_1 = 0.02087*m_1;
% I_2 = 0.08*m_2;
% I_3 = 0.05;
% I_4 = 0.02*m_4;
% a_1 = 0.5;
% a_2 = 0.4;
% x_1 = 0.25;
% x_2 = 0.2;
% x_inv = 95.8;
m_1 = x(17);
m_2 = x(18);
m_3 = x(19);
m_4 = x(20);
I_1 = x(21);
I_2 = x(22);
I_3 = x(23);
I_4 = x(24);
a_1 = x(25);
a_2 = x(26);
x_1 = x(27);
x_2 = x(28);
x_inv = x(29);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u_1 = x(30);
u_2 = x(31);
u_3 = x(32);
u_4 = x(33);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q = [q_1; q_2; q_3; q_4];
dq = [dq_1; dq_2; dq_3; dq_4];
z = [z_1; z_2; z_3; z_4];
u = [u_1; u_2; u_3; u_4];
p = [p_1; p_2; p_3; p_4];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot_parameters = [];
robot_parameters(1) = m_1;
robot_parameters(2) = m_2;
robot_parameters(3) = m_3;
robot_parameters(4) = m_4;
robot_parameters(5) = I_1;
robot_parameters(6) = I_2;
robot_parameters(7) = I_3;
robot_parameters(8) = I_4;
robot_parameters(9) = a_1;
robot_parameters(10) = a_2;
robot_parameters(11) = x_1;
robot_parameters(12) = x_2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[M,C,G] = dynamic_terms(q,dq,robot_parameters);
%M_hat = [m_1,0,0,0;0,m_2,0,0;0,0,m_3,0;0,0,0,m_4];
% try
    M_hat = M;
%     X_inv*inv(M_hat);
% catch
     %M_hat = [m_1,0,0,0;0,m_2,0,0;0,0,m_3,0;0,0,0,m_4];
% end

X_inv = x_inv*eye(4,4);
l = X_inv*inv(M_hat);
N_hat = C*dq+G;
dz = -l*z+l*(N_hat-u-p);


end