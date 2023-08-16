% Robot Parameters
m_1 = 15;
m_2 = 12;
m_3 = 3;
m_4 = 3;
I_1 = 0.02087*m_1;
I_2 = 0.08*m_2;
I_3 = 0.05; %%% CHECK THIS OUT %%%
I_4 = 0.02*m_4;
a_1 = 0.5;
a_2 = 0.4;
x_1 = 0.25;
x_2 = 0.2;

% Controller Parameters
k_p = 2;
k_v = 1.5;
f_c1 = 0.49;
f_c2 = 0.31;
f_c3 = 0.1;
f_c4 = 0.1;
f_s1 = 3.5;
f_s2 = 2.8;
f_s3 = 1.65;
f_s4 = 0.7;
f_v1 = 0.15;
f_v2 = 0.12;
f_v3 = 0.08;
f_v4 = 0.03;
v_s1 = 0.19;
v_s2 = 0.15;
v_s3 = 0.12;
v_s4 = 0.03;
x_inv = 95.8;

% F-payload
f_payload = 2;

% Simulation Parameters
T_f = 10; % Simulation Interval
AT = 1e-6; % Absolute Tolerance
RT = 1e-6; % Relative Tolerance
RF = 4; % Refine Factor