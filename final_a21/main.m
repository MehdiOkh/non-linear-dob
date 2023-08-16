% Simulation Parameters
T_f = 10; % Simulation Interval
AT = 1e-6; % Absolute Tolerance
RT = 1e-6; % Relative Tolerance
RF = 4; % Refine Factor

sim('controller');
RMS_Error = ["joint1";"joint2";"joint3";"joint4"];
Result = [rms(out.rms1);rms(out.rms2);rms(out.rms3);rms(out.rms4)];

t=table(RMS_Error,Result);
disp(t)


% Plot results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Joint Positions
figure
subplot(221)
plot(out.tout,out.q_1)
hold on
out.qd_1.plot('r--');
ylabel(' q_1 (rad) ')
xlabel(' time (s) ')
legend('q_1','q_d_1')
subplot(222)
plot(out.tout,out.q_2)
hold on
out.qd_2.plot('r--');
ylabel(' q_2 (rad) ')
xlabel(' time (s) ')
legend('q_2','q_d_2')
subplot(223)
plot(out.tout,out.q_3)
ylabel(' q_3 (m) ')
hold on
out.qd_3.plot('r--');
xlabel(' time (s) ')
legend('q_3','q_d_3')
subplot(224)
plot(out.tout,out.q_4)
ylabel(' q_4 (rad) ')
hold on
out.qd_4.plot('r--');
xlabel(' time (s) ')
legend('q_4','q_d_4')

figure
subplot(221)
plot(out.tout,out.u_1)
hold on
out.td_1.plot('m--')
legend('t1','td1')
title('');
ylabel(' t1 (Nm) ')
xlabel(' time (s) ')
subplot(222)
plot(out.tout,out.u_2)
hold on
out.td_2.plot('m--')
legend('t2','td2')
title('');
ylabel(' t2 (Nm) ')
xlabel(' time (s) ')
subplot(223)
plot(out.tout,out.u_3)
hold on
out.td_3.plot('m--')
legend('t3','td3')
title('');
ylabel(' t3 (N) ')
xlabel(' time (s) ')
subplot(224)
plot(out.tout,out.u_4)
hold on
out.td_4.plot('m--')
legend('t4','td4')
title('');
ylabel(' t4 (Nm) ')
xlabel(' time (s) ')


% Plot Joint Positions Tracking Error
figure
subplot(221)
out.q_tilde_1.plot();
title('');
ylabel(' q_1 error (rad) ')
xlabel(' time (s) ')
subplot(222)
out.q_tilde_2.plot();
title('');
ylabel(' q_2 error (rad) ')
xlabel(' time (s) ')
subplot(223)
out.q_tilde_3.plot();
title('');
ylabel(' q_3 error (m) ')
xlabel(' time (s) ')
subplot(224)
out.q_tilde_4.plot();
title('');
ylabel(' q_4 error (rad) ')
xlabel(' time (s) ')


% % Tourque Of Disturbance
% figure
% subplot(221)
% out.td_1.plot();
% title('');
% ylabel(' td1 (rad) ')
% xlabel(' time (s) ')
% subplot(222)
% out.td_2.plot();
% title('');
% ylabel(' td2 (rad) ')
% xlabel(' time (s) ')
% subplot(223)
% out.td_3.plot();
% title('');
% ylabel(' td3 (m) ')
% xlabel(' time (s) ')
% subplot(224)
% out.td_4.plot();
% title('');
% ylabel(' td4 (rad) ')
% xlabel(' time (s) ')
