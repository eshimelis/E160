%% E160 Lab 2: Odometery Lab

logpath = '../Code/Log/';
addpath(logpath)

% logfilename = 'Bot0_2018_02_16_11_32_02';
% logfilename = 'Bot0_2018_02_16_11_57_51';
% logfilename = 'Bot0_2018_02_16_12_37_01'; % backwards one meter
% logfilename = 'Bot0_2018_02_16_12_36_24'; % forward one meter
logfilename = '150cmFinal2'

state = ReadBotState([logpath, logfilename]);

% Plot Desired vs Estimated States

deltaT = 0.1;
time = linspace(0, deltaT*length(state.XDes), length(state.XDes));

figure(1); clf;

subplot(3, 2, 1); hold on; grid on; grid minor;
plot(time, state.XDes, 'o');
plot(time, state.XEst, 'o');
title('Desired vs Estimated X Position', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Distance (m)', 'Interpreter', 'Latex');
legend('Desired State', 'Estimated State', 'Location', 'northwest');
set(gca, 'fontsize', 18); hold off;

subplot(3, 2, 3); hold on; grid on; grid minor;
plot(state.YDes, 'o');
plot(state.YEst, 'o');
title('Desired vs Estimated Y Position', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Distance (m)', 'Interpreter', 'Latex');
legend('Desired State', 'Estimated State', 'Location', 'northwest');
set(gca, 'fontsize', 18); hold off;

subplot(3, 2, 5); hold on; grid on; grid minor;
plot(state.ThetaDes, 'o');
plot(state.ThetaEst, 'o');
title('Desired vs Estimated Heading', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Heading (rad)', 'Interpreter', 'Latex');
legend('Desired State', 'Estimated State', 'Location', 'northwest');
set(gca, 'fontsize', 18); hold off;

% plot errors
subplot(3, 2, 2); hold on; grid on; grid minor;
plot(time, state.XError, 'o');
title('Desired vs Estimated X Position Error', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Distance Error (m)', 'Interpreter', 'Latex');
legend('State Error', 'Location', 'northwest');
set(gca, 'fontsize', 18); hold off;

subplot(3, 2, 4); hold on; grid on; grid minor;
plot(state.YError, 'o');
title('Desired vs Estimated Y Position Error', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Distance Error (m)', 'Interpreter', 'Latex');
legend('State Error', 'Location', 'northwest');
set(gca, 'fontsize', 18); hold off;

subplot(3, 2, 6); hold on; grid on; grid minor;
plot(state.ThetaError, 'o');
title('Desired vs Estimated Heading Error', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Heading Error (rad)', 'Interpreter', 'Latex');
legend('State Error', 'Location', 'northwest');
set(gca, 'fontsize', 18); hold off;