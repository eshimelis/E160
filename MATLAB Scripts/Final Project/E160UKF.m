%% E160 Final Project: Unscented Kalman Filter (UKF)

logpath = '../../Code/Log/';
addpath(logpath)

% logfilename = 'Bot0_2018_04_02_14_32_08'; % before fix
logfilename = 'UKFvsPF';

state = ReadBotState([logpath, logfilename]);

% Plot Desired vs Estimated States

deltaT = 0.1;
time = linspace(0, deltaT*length(state.XDes), length(state.XDes));


%%

figure(1); clf; hold on; grid on; grid minor;
plot(state.XOdo, state.YOdo, 'linewidth', 6);
plot(state.XEstUKF, state.YEstUKF, 'linewidth', 4);
plot(state.XEstPF, state.YEstPF, 'linewidth', 4);

plot(state.XOdo(1), state.YOdo(1), 'x', 'markersize', 24, 'color', [0.8500, 0.3250, 0.0980], 'linewidth', 3);
% plot(state.XDes(10), state.YDes(10), 'square', 'markersize', 24, 'color', [0.4660, 0.6740, 0.1880], 'linewidth', 3);
% plot(state.XDe

% draw walls
wall1 = [0, 0, 0, 4];
wall2 = [0, 0, 4, 0];
wall3 = [1.75, 1.8, 1.75, 4];
wall4 = [1.75, 1.8, 1.75+0.87, 1.8];
wall5 = [1.75+0.87, 1.8, 1.75+0.87, 1.8-0.38];
wall6 = [1.75+0.87, 1.8-0.38, 4, 1.8-0.38];
wall7 = [0, 4, 1.75, 4];
wall8 = [4, 0, 4, 1.8-0.38];

map = {wall1, wall2, wall3, wall4, wall5, wall6, wall7, wall8};

for k = 1:length(map)
    wall = map{k};
    plot([wall(1), wall(3)], [wall(2), wall(4)], 'linewidth', 5, 'color', [0, 0, 0])
end

title('Overhead Map UKF Position Estimate', 'Interpreter', 'Latex');
xlabel('X Position (m)', 'Interpreter', 'Latex');
ylabel('Y Position (m)', 'Interpreter', 'Latex');
legend('Odometry Estimate', 'UKF Estimate', 'Particle Filter Estimate', 'Location', 'northeast');
set(gca, 'fontsize', 32); hold off;
axis([-0.25 4.25 -0.25 4.25]); pbaspect([1 1 1]);

hold off;

%% Plot UKF error over time

% odometryPos = sqrt(state.XOdo.^2 + state.YOdo.^2);
% estimatePos = sqrt(state.XEstUKF.^2 + state.YEstUKF.^2);

error = sqrt((state.XOdo-state.XEstUKF).^2 + (state.YOdo - state.YEstUKF).^2)
rmse = sqrt(mean(error.^2))

figure(2); clf; hold on; grid on; grid minor; 
stem(time, error);
title('UKF Estimation Error');
xlabel('Time (s)');
ylabel('Position Error (m)');
set(gca, 'fontsize', 32); hold off;

%% Plot error over time

% odometryPos = sqrt(state.XOdo.^2 + state.YOdo.^2);
% estimatePos = sqrt(state.XEstUKF.^2 + state.YEstUKF.^2);

UKFerror = sqrt((state.XOdo-state.XEstUKF).^2 + (state.YOdo - state.YEstUKF).^2);
PFerror = sqrt((state.XOdo-state.XEstPF).^2 + (state.YOdo - state.YEstPF).^2);

UKFrmse = sqrt(mean(UKFerror.^2))
PFrmse = sqrt(mean(PFerror.^2))

figure(3); clf; hold on; grid on; grid minor; 
stem(time, UKFerror);
stem(time, PFerror);
title('UKF versus PF Estimation Error');
xlabel('Time (s)');
ylabel('Position Error (m)');
set(gca, 'fontsize', 32); hold off;

legend('UKF Error', 'PF Error')

%% Plot overhead map compare filters

% figure(3); clf; hold on; grid on; grid minor;
% plot(state.XOdo, state.YOdo, 'linewidth', 6);
% plot(state.XEstPF, state.YEstPF, 'linewidth', 4);
% plot(state.XEstUKF, state.YEstUKF, 'linewidth', 4);
% 
% plot(state.XOdo(1), state.YOdo(1), 'x', 'markersize', 24, 'color', [0.8500, 0.3250, 0.0980], 'linewidth', 3);
% % plot(state.XDes(10), state.YDes(10), 'square', 'markersize', 24, 'color', [0.4660, 0.6740, 0.1880], 'linewidth', 3);
% % plot(state.XDe
% 
% % draw walls
% wall1 = [0, 0, 0, 4];
% wall2 = [0, 0, 4, 0];
% wall3 = [1.75, 1.8, 1.75, 4];
% wall4 = [1.75, 1.8, 1.75+0.87, 1.8];
% wall5 = [1.75+0.87, 1.8, 1.75+0.87, 1.8-0.38];
% wall6 = [1.75+0.87, 1.8-0.38, 4, 1.8-0.38];
% wall7 = [0, 4, 1.75, 4];
% wall8 = [4, 0, 4, 1.8-0.38];
% 
% map = {wall1, wall2, wall3, wall4, wall5, wall6, wall7, wall8};
% 
% for k = 1:length(map)
%     wall = map{k};
%     plot([wall(1), wall(3)], [wall(2), wall(4)], 'linewidth', 5, 'color', [0, 0, 0])
% end
% 
% title('Overhead Map Filter Comparison', 'Interpreter', 'Latex');
% xlabel('X Position (m)', 'Interpreter', 'Latex');
% ylabel('Y Position (m)', 'Interpreter', 'Latex');
% legend('Odometry Estimate', 'PF Estimate', 'UKF Estimate', 'Location', 'northeast');
% set(gca, 'fontsize', 32); hold off;
% axis([-0.25 4.25 -0.25 4.25]); pbaspect([1 1 1]);
% 
% hold off;

%% Metrics
% odometryPos = sqrt(state.XOdo.^2 + state.YOdo.^2)
% estimatePos = sqrt(state.XEst.^2 + state.YEst.^2)
% 
% error = odometryPos - estimatePos;
% rmse = sqrt(mean(error.^2))
% 
% figure(4); clf; hold on; grid on; grid minor;
% stem(time, error);

% %%
% figure(1); clf;
% 
% subplot(3, 2, 1); hold on; grid on; grid minor;
% plot(time, state.XDes, '.', 'MarkerSize', 25);
% plot(time, state.XEst, '.', 'MarkerSize', 25);
% title('Desired vs Estimated X Position', 'Interpreter', 'Latex');
% xlabel('Time (s)', 'Interpreter', 'Latex');
% ylabel('Distance (m)', 'Interpreter', 'Latex');
% legend('Desired State', 'Estimated State', 'Location', 'southeast');
% set(gca, 'fontsize', 36); hold off;
% 
% subplot(3, 2, 3); hold on; grid on; grid minor;
% plot(time, state.YDes, '.', 'MarkerSize', 25);
% plot(time, state.YEst, '.', 'MarkerSize', 25);
% title('Desired vs Estimated Y Position', 'Interpreter', 'Latex');
% xlabel('Time (s)', 'Interpreter', 'Latex');
% ylabel('Distance (m)', 'Interpreter', 'Latex');
% legend('Desired State', 'Estimated State', 'Location', 'southeast');
% set(gca, 'fontsize', 36); hold off;
% 
% subplot(3, 2, 5); hold on; grid on; grid minor;
% plot(time, state.ThetaDes, '.', 'MarkerSize', 25);
% plot(time, state.ThetaEst, '.', 'MarkerSize', 25);
% title('Desired vs Estimated Heading', 'Interpreter', 'Latex');
% xlabel('Time (s)', 'Interpreter', 'Latex');
% ylabel('Heading (rad)', 'Interpreter', 'Latex');
% legend('Desired State', 'Estimated State', 'Location', 'southeast');
% set(gca, 'fontsize', 36); hold off;
% 
% % plot errors
% subplot(3, 2, 2); hold on; grid on; grid minor;
% stem(time, state.XError, '.', 'MarkerSize', 25);
% title('Desired vs Estimated X Position Error', 'Interpreter', 'Latex');
% xlabel('Time (s)', 'Interpreter', 'Latex');
% ylabel('Distance Error (m)', 'Interpreter', 'Latex');
% legend('State Error', 'Location', 'southeast');
% set(gca, 'fontsize', 36); hold off;
% 
% subplot(3, 2, 4); hold on; grid on; grid minor;
% stem(time, state.YError, '.', 'MarkerSize', 25);
% title('Desired vs Estimated Y Position Error', 'Interpreter', 'Latex');
% xlabel('Time (s)', 'Interpreter', 'Latex');
% ylabel('Distance Error (m)', 'Interpreter', 'Latex');
% legend('State Error', 'Location', 'southeast');
% set(gca, 'fontsize', 36); hold off;
% 
% subplot(3, 2, 6); hold on; grid on; grid minor;
% stem(time, state.ThetaError, '.', 'MarkerSize', 25);
% title('Desired vs Estimated Heading Error', 'Interpreter', 'Latex');
% xlabel('Time (s)', 'Interpreter', 'Latex');
% ylabel('Heading Error (rad)', 'Interpreter', 'Latex');
% legend('State Error', 'Location', 'southeast');
% set(gca, 'fontsize', 36); hold off;
% 
% %% Plot only angles
% 
% figure(2); clf; 
% 
% subplot(2, 1, 1); hold on; grid on; grid minor;
% plot(time, state.ThetaDes, '.', 'MarkerSize', 25);
% plot(time, state.ThetaEst, '.', 'MarkerSize', 25);
% title('Desired vs Estimated Heading', 'Interpreter', 'Latex');
% xlabel('Time (s)', 'Interpreter', 'Latex');
% ylabel('Heading (rad)', 'Interpreter', 'Latex');
% legend('Desired State', 'Estimated State', 'Location', 'southeast');
% set(gca, 'fontsize', 36); hold off;
% 
% subplot(2, 1, 2); hold on; grid on; grid minor;
% stem(time, state.ThetaError, '.', 'MarkerSize', 25);
% title('Desired vs Estimated Heading Error', 'Interpreter', 'Latex');
% xlabel('Time (s)', 'Interpreter', 'Latex');
% ylabel('Heading Error (rad)', 'Interpreter', 'Latex');
% legend('State Error', 'Location', 'southeast');
% set(gca, 'fontsize', 36); hold off;
% 
% %%
% 
% rhoDes = sqrt(state.XDes.^2 + state.YDes.^2);
% rhoEst = sqrt(state.XEst.^2 + state.YEst.^2);
% 
% figure(1); clf; 
% 
% subplot(2, 2, 1); hold on; grid on; grid minor;
% plot(time, rhoDes, '.', 'MarkerSize', 25);
% plot(time, rhoEst, '.', 'MarkerSize', 25);
% title('Desired vs Estimated Euclidian Distance from Origin', 'Interpreter', 'Latex');
% xlabel('Time (s)', 'Interpreter', 'Latex');
% ylabel('Position (m)', 'Interpreter', 'Latex');
% legend('Desired State', 'Estimated State', 'Location', 'southeast');
% set(gca, 'fontsize', 30); hold off;
% 
% subplot(2, 2, 3); hold on; grid on; grid minor;
% stem(time, rhoDes-rhoEst, '.', 'MarkerSize', 25);
% title('Euclidian Distance Error', 'Interpreter', 'Latex');
% xlabel('Time (s)', 'Interpreter', 'Latex');
% ylabel('Position (m)', 'Interpreter', 'Latex');
% legend('State Error', 'Location', 'southeast');
% set(gca, 'fontsize', 30); hold off;
% 
% subplot(2, 2, 2); hold on; grid on; grid minor;
% plot(time, state.ThetaDes, '.', 'MarkerSize', 25);
% plot(time, state.ThetaEst, '.', 'MarkerSize', 25);
% title('Desired vs Estimated Heading', 'Interpreter', 'Latex');
% xlabel('Time (s)', 'Interpreter', 'Latex');
% ylabel('Heading (rad)', 'Interpreter', 'Latex');
% legend('Desired State', 'Estimated State', 'Location', 'southeast');
% set(gca, 'fontsize', 30); hold off;
% 
% subplot(2, 2, 4); hold on; grid on; grid minor;
% stem(time, state.ThetaError, '.', 'MarkerSize', 25);
% title('Desired vs Estimated Heading Error', 'Interpreter', 'Latex');
% xlabel('Time (s)', 'Interpreter', 'Latex');
% ylabel('Heading Error (rad)', 'Interpreter', 'Latex');
% legend('State Error', 'Location', 'southeast');
% set(gca, 'fontsize', 30); hold off;
% 
% 
% % %%
% % 
% % midTime = 25;
% % 
% % dist = 5;
% % 
% % distErrorcm = 1.1;
% % yMeas = 1.6;
% % xMeas = 16.6;
% % 
% % yerror = distErrorcm - (state.XEst(midTime/deltaT) - dist)*100
% % 
% % 
% % ystarterror = yMeas - state.YEst(end)*100
% % xstarterror = xMeas - state.XEst(end)*100