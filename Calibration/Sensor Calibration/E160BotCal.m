% Eyassu Shimelis
% E160 Lab 1: E160 Bot Distance Calibration
% February 3, 2018

load E160FrontCalData

% True data collection points
cmTrue = 15:5:100; cmTrue = cmTrue';

cmMeasured = [mean(cm15), mean(cm20), mean(cm25), mean(cm30), ...
          mean(cm35), mean(cm40), mean(cm45), mean(cm50), ...
          mean(cm55), mean(cm60), mean(cm65), mean(cm70), ...
          mean(cm75), mean(cm80), mean(cm85), mean(cm90), ...
          mean(cm95), mean(cm100)]';

% combine data vectors
calData = [cmMeasured, cmTrue];
         
stdError = [std(cm15);
    std(cm20);
    std(cm25);
    std(cm30);
    std(cm35);
    std(cm40);
    std(cm45);
    std(cm50);
    std(cm55);
    std(cm60);
    std(cm65);
    std(cm70);
    std(cm75);
    std(cm80);
    std(cm85);
    std(cm90);
    std(cm95);
    std(cm100)];
        
% remove 15cm measurement
cmMeasured = cmMeasured(2:end);
cmTrue = cmTrue(2:end); 
stdError = stdError(2:end);

logFit = fittype('a + b*log(x)',...
'dependent',{'y'},'independent',{'x'},...
'coefficients',{'a','b'});

inverseFit = @(c, x) c(1) + c(2)./(x.^c(3));
inverseFitCoeff = nlinfit(cmMeasured, cmTrue, inverseFit, [1, 1, 0.5]);


logFit = fit(cmMeasured, cmTrue, logFit);
[polyFit, polyErr] = polyfit(cmMeasured, cmTrue, 2);

% calculate residuals
polyResid = cmTrue - polyval(polyFit, cmMeasured);
polySSresid = sum(polyResid.^2);
polySStotal = (length(cmTrue) - 1) * var(cmTrue);
polyrsq = 1 - polySSresid/polySStotal;

logResid = cmTrue - logFit(cmMeasured);
logSSresid = sum(logResid.^2);
logSStotal = (length(cmTrue) - 1) * var(cmTrue);
logrsq = 1 - logSSresid/logSStotal;

inverseResid = cmTrue - inverseFit(inverseFitCoeff, cmMeasured);
inverseSSresid = sum(inverseResid.^2);
inverseSStotal = (length(cmTrue) - 1) * var(cmTrue);
inversersq = 1 - inverseSSresid/inverseSStotal;

% plotting
figure(1); clf; hold on;
set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');

% actual results
plot(cmMeasured, cmTrue, 'blx');

% plot fits
plot(cmMeasured, logFit(cmMeasured), 'Color', [0,0.4470,0.7410], 'LineWidth', 2, 'LineStyle', '--');
plot(cmMeasured, polyval(polyFit, cmMeasured), 'Color', [0.8500,0.3250,0.0980], 'LineWidth', 2, 'LineStyle', ':');
plot(cmMeasured, inverseFit(inverseFitCoeff, cmMeasured), 'Color', [0.4940,0.1840,0.5560], 'LineWidth', 2, 'LineStyle', '-');

legend('Measured Digital Voltage', ...
    ['Log Fit, R$^2$ = ', num2str(logrsq)], ...
    ['2nd Order Polynomial Fit, R$^2$ = ', num2str(polyrsq)], ...
    ['Nonlinear Inverse Fit, R$^2$ = ', num2str(inversersq)])

title('Sharp Distance Sensor Calibration Fitting');
ylabel('Actual Distance (cm)');
xlabel('Measured Distance (ADC Output)');
set(gca, 'fontsize', 28);
grid on; grid minor;

% plot error
errorbar(cmMeasured, cmTrue, ones(length(cmTrue), 1).*0.3, 'LineStyle', 'none')
herrorbar(cmMeasured, cmTrue, stdError);
hold off;