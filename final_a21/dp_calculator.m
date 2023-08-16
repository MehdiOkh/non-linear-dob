function [p] = dp_calculator(x)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint Velocities
dq_1 = x(1);
dq_2 = x(2);
dq_3 = x(3);
dq_4 = x(4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot Parameters
x_inv = x(5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dq = [dq_1; dq_2; dq_3; dq_4];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X_inv = x_inv*eye(4,4);
p = X_inv*dq;


end