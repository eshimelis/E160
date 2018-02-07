%% P Control Testing
robotStateTowards = readtable('BotStatePControlToWall.txt');
timeTowards = linspace(0, length(robotStateTowards.Var1)*0.1, length(robotStateTowards.Var1));

robotStateAway = readtable('BotStatePControlAwayFromWall.txt');
timeAway = linspace(0, length(robotStateAway.Var1)*0.1, length(robotStateAway.Var1));


figure(2); 
FigH = figure('DefaultAxesPosition', [0.1, 0.1, 0.8, 0.8]);
subplot(2, 2, 1);
% FigH = figure('DefaultAxesPosition', [0.1, 0.1, 0.8, 0.8]);

plot(timeTowards, robotStateTowards.Var1, '.', 'MarkerSize', 15);

title('Proportional Control Wall Distance (Towards Wall, $K_P = 2$)', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Distance from Wall (cm)', 'Interpreter', 'Latex');

legend('Measured distance from wall');
set(gca, 'Fontsize', 28)
grid on; grid minor;


subplot(2, 2, 3);
plot(timeTowards, robotStateTowards.Var7, '.', 'MarkerSize', 15);

title('Proportional Control Error (Towards Wall, $K_P = 2$)', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Error (cm)', 'Interpreter', 'Latex');

legend('Control loop error');
set(gca, 'Fontsize', 28)
grid on; grid minor;


subplot(2, 2, 2);
plot(timeAway, robotStateAway.Var1, '.', 'MarkerSize', 15);

title('Proportional Wall Distance (Away from Wall, $K_P = 2$)', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Distance from Wall (cm)', 'Interpreter', 'Latex');

legend('Measured distance from wall');
set(gca, 'Fontsize', 28)
grid on; grid minor;


subplot(2, 2, 4);
plot(timeAway, robotStateAway.Var7, '.', 'MarkerSize', 15);

title('Proportional Control Error (Away from Wall, $K_P = 2$)', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Error (cm)', 'Interpreter', 'Latex');

legend('Control loop error');
set(gca, 'Fontsize', 28)
grid on; grid minor;



%% PI Control Testing

robotStatePISixth = readtable('BotStatePIControlSixth.txt');
timePISixth = linspace(0, length(robotStatePISixth.Var1)*0.1, length(robotStatePISixth.Var1));

robotStatePITenth = readtable('BotStatePIControlTenth.txt');
timePITenth = linspace(0, length(robotStatePITenth.Var1)*0.1, length(robotStatePITenth.Var1));

% robotStateAway = readtable('BotStatePControlAwayFromWall.txt');
% timeAway = linspace(0, length(robotStateAway.Var1)*0.1, length(robotStateAway.Var1));

figure(3); subplot(2, 2, 1);
plot(timePISixth, robotStatePISixth.Var1, '.', 'MarkerSize', 15);

title('PI Control Wall Distance ($K_P = 2, K_I = 0.6$)', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Distance from Wall (cm)', 'Interpreter', 'Latex');

legend('Measured distance from wall');
set(gca, 'Fontsize', 28)
grid on; grid minor;


subplot(2, 2, 3);
plot(timePISixth, robotStatePISixth.Var7, '.', 'MarkerSize', 15);

title('PI Control Control Error ($K_P = 2, K_I = 0.6$)', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Error (cm)', 'Interpreter', 'Latex');

legend('Control loop error');
set(gca, 'Fontsize', 28)
grid on; grid minor;


subplot(2, 2, 2);
plot(timePITenth, robotStatePITenth.Var1, '.', 'MarkerSize', 15);

title('PI Control Wall Distance ($K_P = 2, K_I = 0.1$)', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Distance from  (cm)', 'Interpreter', 'Latex');

legend('Measured distance from wall');
set(gca, 'Fontsize', 28)
grid on; grid minor;


subplot(2, 2, 4);
plot(timePITenth, robotStatePITenth.Var7, '.', 'MarkerSize', 15);

title('PI Control Error ($K_P = 2, K_I = 0.1$)', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('Error (cm)', 'Interpreter', 'Latex');

legend('Control loop error');
set(gca, 'Fontsize', 28)
grid on; grid minor;