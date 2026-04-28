%% Dye kinetic characterization - Plotting
% Process the .mat files generated from process_images_2sat_curves.m
% to plot the desaturation and saturation curves of either color channels.
clear; close all; clc;

%% User parameters
dyeColor = 'yellow';   % 'yellow' or 'cyan'
nTrials  = 3;
saveResults = true;
dt_saturation = 5;     % seconds between saturation images
dt_desaturation = 1;   % seconds between desaturation images

switch lower(dyeColor)
    case 'cyan'
        cLine = [0.20 0.65 0.90];
    case 'magenta'
        cLine = [0.85 0.35 0.70];
    case 'yellow'
        cLine = [0.95 0.80 0.15];
    otherwise
        cLine = [0.3 0.3 0.3];
end

fprintf('Loading %s trials...\n', dyeColor);

%% Load data
satCurves = {}; desCurves = {};
satTimes  = {}; desTimes  = {};

for i = 1:nTrials
    fileName = sprintf('%s_trial_%d_results.mat', dyeColor, i);
    if ~isfile(fileName)
        warning('Missing %s', fileName);
        continue;
    end
    S = load(fileName);
    R = S.results;
    satCurves{end+1} = R.saturation.Normalized_0to1;
    desCurves{end+1} = R.desaturation.Normalized_0to1;
    satTimes{end+1}  = R.saturation.Time_s;
    desTimes{end+1}  = R.desaturation.Time_s;
end

%% Define the common time base
tSatMax = min(cellfun(@max, satTimes));
tDesMax = min(cellfun(@max, desTimes));

tSat = (0:dt_saturation:tSatMax)';
tDes = (0:dt_desaturation:tDesMax)';

%% Interpolate to a common grid
interpSat = nan(numel(tSat), numel(satCurves));
interpDes = nan(numel(tDes), numel(desCurves));

for i = 1:numel(satCurves)
    interpSat(:,i) = interp1(satTimes{i}, satCurves{i}, tSat, 'linear', NaN);
end
for i = 1:numel(desCurves)
    interpDes(:,i) = interp1(desTimes{i}, desCurves{i}, tDes, 'linear', NaN);
end

%% Compute the mean ± STD
meanSat = mean(interpSat, 2, 'omitnan');
stdSat  = std(interpSat, 0, 2, 'omitnan');
meanDes = mean(interpDes, 2, 'omitnan');
stdDes  = std(interpDes, 0, 2, 'omitnan');

%% --- PLOT SEPARATELY ---
figure('Color','w','Name',[dyeColor ' Saturation'],'Position',[200 200 520 380]);
hold on;
for i = 1:numel(satCurves)
    plot(tSat, interpSat(:,i), '-', 'Color', [cLine 0.3], 'LineWidth', 1);
end
plot(tSat, meanSat, '-', 'Color', cLine, 'LineWidth', 3);
xlabel('Time (s)');
ylabel('Normalized Brightness (0–1)');
title(sprintf('%s – Saturation Average', capitalize(dyeColor)));
grid on; box on;
set(gca,'FontSize',12);
legend('Individual Trials','Mean','Location','best');

figure('Color','w','Name',[dyeColor ' Desaturation'],'Position',[750 200 520 380]);
hold on;
for i = 1:numel(desCurves)
    plot(tDes, interpDes(:,i), '-', 'Color', [0 0 0 0.2], 'LineWidth', 1);
end
plot(tDes, meanDes, '-', 'Color', [0 0 0], 'LineWidth', 3);
xlabel('Time (s)');
ylabel('Normalized Brightness (0–1)');
title(sprintf('%s – Desaturation Average', capitalize(dyeColor)));
grid on; box on;
set(gca,'FontSize',12);
legend('Individual Trials','Mean','Location','best');

%% Save results to be processed
if saveResults
    combined.meanSat = meanSat;
    combined.stdSat  = stdSat;
    combined.tSat    = tSat;
    combined.meanDes = meanDes;
    combined.stdDes  = stdDes;
    combined.tDes    = tDes;
    save(sprintf('%s_combined_results.mat', dyeColor), 'combined');
    fprintf('Saved averaged results for %s dye.\n', dyeColor);
end

%% Helper functions
function s = capitalize(str)
    s = lower(str);
    s(1) = upper(s(1));
end
