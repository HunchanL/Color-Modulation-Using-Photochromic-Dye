%% Dye kinetic characterization - Image processing
% Script takes in the images of the photochromic dye solution and get the
% normalized desaturation and saturation curves.
clear; close all; clc;

%% User parameters
dyeColor = 'yellow';                  % 'yellow' or 'cyan'
trialFolder = 'yellow trial 1';       % main folder name
dt_saturation = 5;                    % interval between saturation images (s)
dt_desaturation = 1;                  % interval between desaturation images (s)
saveResults = true;

% Subfolders inside the trial folder
folders = {'saturation', 'desaturation'};
imgExts = {'.jpg', '.jpeg', '.png', '.JPG', '.JPEG', '.PNG'};

fprintf('Processing %s ...\n', trialFolder);
rootPath = fullfile(pwd, trialFolder);
results = struct();

%% Select the ROI
firstImgPath = fullfile(rootPath, folders{1});
imgFiles = dir(firstImgPath);
imgFiles = imgFiles(~[imgFiles.isdir]); % skip directories

% Filter for valid image extensions
validMask = false(size(imgFiles));
for i = 1:numel(imgFiles)
    [~,~,ext] = fileparts(imgFiles(i).name);
    if any(strcmpi(ext, imgExts))
        validMask(i) = true;
    end
end
imgFiles = imgFiles(validMask);

if isempty(imgFiles)
    error('No valid image files found in saturation folder: %s', firstImgPath);
end

% Read first image for ROI selection
img0 = imread(fullfile(firstImgPath, imgFiles(1).name));
figure; imshow(img0);
title(['Select ROI for ', trialFolder]);
rect = round(getrect);
close;

%% Helper function to compute the brightness
getBrightness = @(folder) local_getBrightness(folder, rect, imgExts);

%% Process desaturation and saturationf or one color channel
for k = 1:numel(folders)
    folderPath = fullfile(rootPath, folders{k});
    if ~isfolder(folderPath)
        warning('Folder not found: %s', folderPath);
        continue;
    end

    brightness = getBrightness(folderPath);
    nImgs = numel(brightness);
    if nImgs == 0
        warning('No valid images found in %s', folderPath);
        continue;
    end

    dt = (k == 1) * dt_saturation + (k == 2) * dt_desaturation;
    time = (0:nImgs-1)' * dt;

    % Ensure matching vector sizes
    brightness = brightness(:);
    if numel(time) ~= numel(brightness)
        minLen = min(numel(time), numel(brightness));
        time = time(1:minLen);
        brightness = brightness(1:minLen);
    end

    results.(folders{k}) = table(time, brightness, ...
        'VariableNames', {'Time_s','Brightness_0to255'});
end

%% Normalization
for k = 1:numel(folders)
    f = folders{k};
    if isfield(results, f)
        raw = results.(f).Brightness_0to255;
        Imin = min(raw);
        Imax = max(raw);
        results.meta.(f).Imin = Imin;
        results.meta.(f).Imax = Imax;

        normVal = 1 - (raw - Imin) ./ (Imax - Imin);

        results.(f).Normalized_0to1 = normVal;
    end
end
results.meta.rect = rect;

%% Save
if saveResults
    saveName = sprintf('%s_results.mat', strrep(trialFolder,' ','_'));
    save(saveName, 'results');
    fprintf('Saved %s\n', saveName);
end

%% Quick plot
figure('Color','w'); hold on;
if isfield(results, 'saturation')
    plot(results.saturation.Time_s, results.saturation.Normalized_0to1, ...
        'r.-', 'LineWidth', 1.5, 'DisplayName', 'Saturation');
end
if isfield(results, 'desaturation')
    plot(results.desaturation.Time_s, results.desaturation.Normalized_0to1, ...
        'k.-', 'LineWidth', 1.5, 'DisplayName', 'Desaturation');
end
xlabel('Time (s)'); ylabel('Normalized Brightness (0–1)');
legend('show');
title(strrep(trialFolder,'_',' '));
grid on;

%% Local functions
function brightness = local_getBrightness(folderPath, rect, imgExts)
    files = dir(folderPath);
    files = files(~[files.isdir]);
    brightness = [];
    for i = 1:numel(files)
        [~,~,ext] = fileparts(files(i).name);
        if any(strcmpi(ext, imgExts))
            img = imread(fullfile(folderPath, files(i).name));
            crop = imcrop(img, rect);
            brightness(end+1,1) = mean(rgb2gray(crop), 'all'); %#ok<AGROW>
        end
    end
end