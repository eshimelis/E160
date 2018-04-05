%% E160 Lab 4 - Particle Filter
% Beam-based sensor probability density model

xDens = 1000;
xMax = 3;
x = linspace(0, xMax, xDens);

zMax = 3;  % maximum allowable measurement
% z = 3;
zExp = 1;
zSigma = 0.1;

% eta = 1/29.87;
% eta = 1;
lambda = 0.1;

% noisy measurement model
noisyModel = (1/(2*pi*zSigma))*exp(-(x-zExp).^2 / zSigma);

% unexpected obstacle model
unexpModel = lambda*exp(-lambda*x);
unexpModel(zExp*(xDens/xMax):end) = 0;

% random noise model
randomModel = ones(xDens, 1)'/zMax;

% max range model
maxModel = ones(xDens, 1)';
maxModel(1:2.75*(xDens/xMax)) = 0;

% combined model
beamModel = noisyModel+unexpModel+randomModel+maxModel;

% area under curve for normalization (used to normalize pdf)
eta = 1/trapz(beamModel)

% plot things
figure(1); clf;

subplot(2, 2, 1); 
plot(x, eta*noisyModel, 'Linewidth', 4);
title('Noisy Sensor Measurement Model');
xlabel('Distance Measured (m)');
ylabel('$P(z | z_{exp} = 1m)$', 'Interpreter', 'Latex');
set(gca, 'Fontsize', 29);
ylim([0, 0.003])
grid on; grid minor;

subplot(2, 2, 2);
plot(x, eta*unexpModel, 'Linewidth', 4, 'Color', [0.8500, 0.3250, 0.0980]);
title('Unexpected Obstacle Model');
xlabel('Distance Measured (m)');
ylabel('$P(z | z_{exp} = 1m)$', 'Interpreter', 'Latex');
set(gca, 'Fontsize', 29);
ylim([0, 0.003])
grid on; grid minor;

subplot(2, 2, 3); 
plot(x, eta*randomModel, 'Linewidth', 4, 'Color', [0.4940, 0.1840, 0.5560]);
title('Random Noise Model');
xlabel('Distance Measured (m)');
ylabel('$P(z)$', 'Interpreter', 'Latex');
ylim([0, 0.003])
grid on; grid minor;
set(gca, 'Fontsize', 29);

subplot(2, 2, 4);
plot(x, eta*maxModel, 'Linewidth', 4, 'Color', [0.4660, 0.6740, 0.1880]);
title('Maximum Range Model');
xlabel('Distance Measured (m)');
ylabel('$P(z)$', 'Interpreter', 'Latex');
set(gca, 'Fontsize', 29);
ylim([0, 0.003])
grid on; grid minor;

figure(2); clf; hold on; 
plot(x, eta*noisyModel, 'Linewidth', 4);
plot(x, eta*unexpModel, 'Linewidth', 4, 'Color', [0.8500, 0.3250, 0.0980]);
plot(x, eta*randomModel, 'Linewidth', 4, 'Color', [0.4940, 0.1840, 0.5560]);
plot(x, eta*maxModel, 'Linewidth', 4, 'Color', [0.4660, 0.6740, 0.1880]);
grid on; grid minor;
title('Overlaid Proximity Models');
xlabel('Distance Measured (m)');
ylabel('$P(z|z_{exp} = 1m)$', 'Interpreter', 'Latex');
legend('Noisy Measurement','Unexpected Obstacle', 'Random Noise', ...
'Maximum Measurement', 'Location', 'northwest');
set(gca, 'Fontsize', 29);
ylim([0, 0.003])
hold off;

figure(3); clf;
plot(x, eta*beamModel, 'Linewidth', 4);
grid on; grid minor;
title('Combined Probabilistic Proximity Model');
xlabel('Distance Measured (m)');
ylabel('$P(z|z_{exp} = 1m)$', 'Interpreter', 'Latex');
set(gca, 'Fontsize', 29);
ylim([0, 0.003])
hold off;
