function dx = open_loop_dynamics(x)
% Joint Positions
q_1 = x(1);
q_2 = x(2);
q_3 = x(3);
q_4 = x(4);
% Joint Velocities
dq_1 = x(5);
dq_2 = x(6);
dq_3 = x(7);
dq_4 = x(8);
% Joint Torques
u_1 = x(9);
u_2 = x(10);
u_3 = x(11);
u_4 = x(12);
% Robot Parameters
m_1 = x(13);
m_2 = x(14);
m_3 = x(15);
m_4 = x(16);
I_1 = x(17);
I_2 = x(18);
I_3 = x(19);
I_4 = x(20);
a_1 = x(21);
a_2 = x(22);
x_1 = x(23);
x_2 = x(24);
g = 9.8;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint Position, Velocity and Torque Vectors
q = [ q_1; q_2; q_3; q_4];
dq = [dq_1; dq_2; dq_3; dq_4];
u = [ u_1; u_2; u_3; u_4];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Open-Loop Dynamics of the Manipulator
[M,C,G] = dynamic_terms(q,dq,robot_parameters);
f = [dq; -inv(M)*(C*dq+G)];
G = [zeros(4,4); inv(M)];
dx = f+G*u;
end