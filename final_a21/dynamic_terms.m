function [M,C,G]=dynamic_terms(q,dq,robot_parameters)
m_1 = robot_parameters(1);
m_2 = robot_parameters(2);
m_3 = robot_parameters(3);
m_4 = robot_parameters(4);
I_1 = robot_parameters(5);
I_2 = robot_parameters(6);
I_3 = robot_parameters(7);
I_4 = robot_parameters(8);
a_1 = robot_parameters(9);
a_2 = robot_parameters(10);
x_1 = robot_parameters(11);
x_2 = robot_parameters(12);
g = 9.8;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q_1 = q(1);
q_2 = q(2);
dq_1 = dq(1);
dq_2 = dq(2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters of Matrices
p_1 = I_1 + I_2 + I_3 + I_4 + m_1*(x_1^2) + m_2*((x_2^2)+(a_1^2)) + (m_3+m_4)*((a_1^2)+(a_2^2));
p_2 = 2*a_1*(x_2*m_2 + a_2*(m_3+m_4));
p_3 = I_2 + I_3 + I_4 + m_2*(x_2^2) + (a_2^2)*(m_3+m_4);
p_4 = m_3 + m_4;
p_5 = I_4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mass-Inertia Matrix
M = zeros(4,4);

M(1,1) = p_1 + p_2*cos(q_2);
M(1,2) = p_3 + 0.5*p_2*cos(q_2);
M(1,3) = 0;
M(1,4) = -p_5;

M(2,1) = p_3 + 0.5*p_2*cos(q_2);
M(2,2) = p_3;
M(2,3) = 0;
M(2,4) = -p_5;

M(3,1) = 0;
M(3,2) = 0;
M(3,3) = p_4;
M(3,4) = 0;

M(4,1) = -p_5;
M(4,2) = -p_5;
M(4,3) = 0;
M(4,4) = p_5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Corilois and Centrifugal Matrix
C = zeros(4,4);

C(1,1) = -p_2*sin(q_2)*dq_2;
C(1,2) = -0.5*p_2*sin(q_2)*dq_2;
C(1,3) = 0;
C(1,4) = 0;

C(2,1) = 0.5*p_2*sin(q_2)*dq_1;
C(2,2) = 0;
C(2,3) = 0;
C(2,4) = 0;

C(3,1) = 0;
C(3,2) = 0;
C(3,3) = 0;
C(3,4) = 0;

C(4,1) = 0;
C(4,2) = 0;
C(4,3) = 0;
C(4,4) = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gravity Vector
G = zeros(4,1);

G(1,1) = 0;
G(2,1) = 0;
G(3,1) = -p_4*g;
G(4,1) = 0;
end