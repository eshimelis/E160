%% E160 Lab 4 - Particle Filter
% Beam-based sensor probability density model

xDens = 1000;
xMax = 10;
x = linspace(0, xMax, xDens);

zMax = 10;  % maximum allowable measurement
z = 3;
zExp = 5;
zSigma = 0.2;

eta = 1/29.87;
lambda = 0.4;

% noisy measurement model
noisyModel = (eta/(2*pi*zSigma))*exp(-(x-zExp).^2 / zSigma);

% unexpected obstacle model
unexpModel = eta*lambda*exp(-lambda*x);
unexpModel(zExp*(xDens/xMax):end) = 0;

% random noise model
randomModel = ones(xDens, 1)' * eta/zMax;

% max range model
maxModel = ones(xDens, 1)'  * eta;
maxModel(1:9.5*(xDens/xMax)) = 0;

% combined model
beamModel = noisyModel+unexpModel+randomModel+maxModel;

% area under curve for normalization (used to normalize pdf)
% cumProb = trapz(beamModel)./xMax

% plot things
figure(1); clf;

subplot(2, 2, 1); 
plot(x, noisyModel, 'Linewidth', 4);
title('Noisy Sensor Measurement Model');
xlabel('Distance Measured (m)');
ylabel('$P(z | z_{exp} = 5m)$', 'Interpreter', 'Latex');
set(gca, 'Fontsize', 29);
ylim([0, 0.04])
grid on; grid minor;

subplot(2, 2, 2);
plot(x, unexpModel, 'Linewidth', 4, 'Color', [0.8500, 0.3250, 0.0980]);
title('Unexpected Obstacle Model');
xlabel('Distance Measured (m)');
ylabel('$P(z | z_{exp} = 5m)$', 'Interpreter', 'Latex');
set(gca, 'Fontsize', 29);
ylim([0, 0.04])
grid on; grid minor;

subplot(2, 2, 3); 
plot(x, randomModel, 'Linewidth', 4, 'Color', [0.4940, 0.1840, 0.5560]);
title('Random Noise Model');
xlabel('Distance Measured (m)');
ylabel('$P(z)$', 'Interpreter', 'Latex');
ylim([0, 0.04])
grid on; grid minor;
set(gca, 'Fontsize', 29);

subplot(2, 2, 4);
plot(x, maxModel, 'Linewidth', 4, 'Color', [0.4660, 0.6740, 0.1880]);
title('Maximum Range Model');
xlabel('Distance Measured (m)');
ylabel('$P(z)$', 'Interpreter', 'Latex');
set(gca, 'Fontsize', 29);
ylim([0, 0.04])
grid on; grid minor;

figure(2); clf; hold on; 
plot(x, noisyModel, 'Linewidth', 4);
plot(x, unexpModel, 'Linewidth', 4, 'Color', [0.8500, 0.3250, 0.0980]);
plot(x, randomModel, 'Linewidth', 4, 'Color', [0.4940, 0.1840, 0.5560]);
plot(x, maxModel, 'Linewidth', 4, 'Color', [0.4660, 0.6740, 0.1880]);
grid on; grid minor;
title('Overlaid Proximity Models');
xlabel('Distance Measured (m)');
ylabel('$P(z|z_{exp} = 5m)$', 'Interpreter', 'Latex');
legend('Noisy Measurement','Unexpected Obstacle', 'Random Noise', ...
'Maximum Measurement', 'Location', 'northwest');
set(gca, 'Fontsize', 29);
ylim([0, 0.04])
hold off;

figure(3); clf;
plot(x, beamModel, 'Linewidth', 4);
grid on; grid minor;
title('Combined Probabilistic Proximity Model');
xlabel('Distance Measured (m)');
ylabel('$P(z|z_{exp} = 5m)$', 'Interpreter', 'Latex');
set(gca, 'Fontsize', 29);
ylim([0, 0.04])
hold off;
