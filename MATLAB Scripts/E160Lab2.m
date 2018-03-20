%% E160 Lab 2: Odometery Lab

logpath = '../Code/Log/';
addpath(logpath)

% logfilename = 'Bot0_2018_02_16_11_32_02';
% logfilename = 'Bot0_2018_02_16_11_57_51';
% logfilename = 'Bot0_2018_02_16_12_37_01'; % backwards one meter
% logfilename = 'Bot0_2018_02_16_12_36_24'; % forward one meter
logfilename = 'Bot0_2018_03_04_15_05_19';

state = ReadBotState([logpath, logfilename]);

% Plot Desired vs Estimated States

deltaT = 0.1;
time = linspace(0, deltaT*length(state.XDes), length(state.XDes));
%%

figure(3); hold on; grid on; grid minor;
plot(state.XEst, state.YEst, '.', 'MarkerSize', 25);
title('Point Tracking Overhead Paths', 'Interpreter', 'Latex');
xlabel('X Position (m)', 'Interpreter', 'Latex');
ylabel('Y Position (m)', 'Interpreter', 'Latex');
legend('Robot Position', 'Location', 'southeast');
set(gca, 'fontsize', 36); hold off;
axis([-0.3 0.3 -0.3 0.3]); pbaspect([1 1 1]);

%%
figure(1); clf;

subplot(3, 2, 1); hold on; grid on; grid minor;
plot(time, state.XDes, '.', 'MarkerSize', 25);
plot(time, state.XEst, '.', 'MarkerSize', 25);
title('Desired vs Estimated X Position', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Distance (m)', 'Interpreter', 'Latex');
legend('Desired State', 'Estimated State', 'Location', 'southeast');
set(gca, 'fontsize', 36); hold off;

subplot(3, 2, 3); hold on; grid on; grid minor;
plot(time, state.YDes, '.', 'MarkerSize', 25);
plot(time, state.YEst, '.', 'MarkerSize', 25);
title('Desired vs Estimated Y Position', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Distance (m)', 'Interpreter', 'Latex');
legend('Desired State', 'Estimated State', 'Location', 'southeast');
set(gca, 'fontsize', 36); hold off;

subplot(3, 2, 5); hold on; grid on; grid minor;
plot(time, state.ThetaDes, '.', 'MarkerSize', 25);
plot(time, state.ThetaEst, '.', 'MarkerSize', 25);
title('Desired vs Estimated Heading', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Heading (rad)', 'Interpreter', 'Latex');
legend('Desired State', 'Estimated State', 'Location', 'southeast');
set(gca, 'fontsize', 36); hold off;

% plot errors
subplot(3, 2, 2); hold on; grid on; grid minor;
stem(time, state.XError, '.', 'MarkerSize', 25);
title('Desired vs Estimated X Position Error', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Distance Error (m)', 'Interpreter', 'Latex');
legend('State Error', 'Location', 'southeast');
set(gca, 'fontsize', 36); hold off;

subplot(3, 2, 4); hold on; grid on; grid minor;
stem(time, state.YError, '.', 'MarkerSize', 25);
title('Desired vs Estimated Y Position Error', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Distance Error (m)', 'Interpreter', 'Latex');
legend('State Error', 'Location', 'southeast');
set(gca, 'fontsize', 36); hold off;

subplot(3, 2, 6); hold on; grid on; grid minor;
stem(time, state.ThetaError, '.', 'MarkerSize', 25);
title('Desired vs Estimated Heading Error', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Heading Error (rad)', 'Interpreter', 'Latex');
legend('State Error', 'Location', 'southeast');
set(gca, 'fontsize', 36); hold off;

%% Plot only angles

figure(2); clf; 

subplot(2, 1, 1); hold on; grid on; grid minor;
plot(time, state.ThetaDes, '.', 'MarkerSize', 25);
plot(time, state.ThetaEst, '.', 'MarkerSize', 25);
title('Desired vs Estimated Heading', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Heading (rad)', 'Interpreter', 'Latex');
legend('Desired State', 'Estimated State', 'Location', 'southeast');
set(gca, 'fontsize', 36); hold off;

subplot(2, 1, 2); hold on; grid on; grid minor;
stem(time, state.ThetaError, '.', 'MarkerSize', 25);
title('Desired vs Estimated Heading Error', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Heading Error (rad)', 'Interpreter', 'Latex');
legend('State Error', 'Location', 'southeast');
set(gca, 'fontsize', 36); hold off;

%%

rhoDes = sqrt(state.XDes.^2 + state.YDes.^2);
rhoEst = sqrt(state.XEst.^2 + state.YEst.^2);

figure(1); clf; 

subplot(2, 2, 1); hold on; grid on; grid minor;
plot(time, rhoDes, '.', 'MarkerSize', 25);
plot(time, rhoEst, '.', 'MarkerSize', 25);
title('Desired vs Estimated Euclidian Distance from Origin', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Position (m)', 'Interpreter', 'Latex');
legend('Desired State', 'Estimated State', 'Location', 'southeast');
set(gca, 'fontsize', 30); hold off;

subplot(2, 2, 3); hold on; grid on; grid minor;
plot(time, rhoDes-rhoEst, '.', 'MarkerSize', 25);
title('Euclidian Distance Error', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Position (m)', 'Interpreter', 'Latex');
legend('State Error', 'Location', 'southeast');
set(gca, 'fontsize', 30); hold off;

subplot(2, 2, 2); hold on; grid on; grid minor;
plot(time, state.ThetaDes, '.', 'MarkerSize', 25);
plot(time, state.ThetaEst, '.', 'MarkerSize', 25);
title('Desired vs Estimated Heading', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Heading (rad)', 'Interpreter', 'Latex');
legend('Desired State', 'Estimated State', 'Location', 'southeast');
set(gca, 'fontsize', 30); hold off;

subplot(2, 2, 4); hold on; grid on; grid minor;
stem(time, state.ThetaError, '.', 'MarkerSize', 25);
title('Desired vs Estimated Heading Error', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Heading Error (rad)', 'Interpreter', 'Latex');
legend('State Error', 'Location', 'southeast');
set(gca, 'fontsize', 30); hold off;


% %%
% 
% midTime = 25;
% 
% dist = 5;
% 
% distErrorcm = 1.1;
% yMeas = 1.6;
% xMeas = 16.6;
% 
% yerror = distErrorcm - (state.XEst(midTime/deltaT) - dist)*100
% 
% 
% ystarterror = yMeas - state.YEst(end)*100
% xstarterror = xMeas - state.XEst(end)*100