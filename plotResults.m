close all
clc
clear all

% File names and custom time frames for the scenes
sceneFiles = {'dist_scene1.mat', 'dist_scene2.mat', 'dist_scene3.mat', ...
    'dist_scene4.mat', 'dist_scene5.mat', 'dist_scene6.mat', 'dist_scene7.mat', ...
    'dist_scene8.mat', 'dist_scene9.mat'};
timeFrames = [19, 21, 22, 15, 13, 16, 15, 20, 12];  % Custom time frames for each scene
targetVeh = [1, 1, 1, 1, 18, 4, 4, 22, 1];  % Target vehicle ID for each scenario

% Initialize arrays to store data for each scene
d_all = {};  % To store distance arrays for each scene
t_all = {};  % To store time arrays for each scene
magnitude_all = {};  % To store magnitude arrays for each scene
trajError_all = {};  % To store trajectory error for each scene
nop_all = {};  % To store nop for each scene
colors = [1, 160, 233; 0, 81, 149; 46, 177, 53; 0, 119, 61; 255, 178, 1; ...
    234, 113, 37; 220, 41, 30; 154, 155, 157; 0, 0, 0] / 255;  % Different colors for each plot
markers = {'o', '^', 's', 'd', 'o', 'p', '^', 's', 'd'};  % Different markers for each plot
lineStyles = {'-', '-', '-', '-', '-', '-', '-', '-', '-'};

% Structure to store results
dataTable = struct('vel', [], 'velR2', []);

% Loop through each scene and load the distance, relPosition, pred, Inpred, and nop data
for sceneIdx = 1:length(sceneFiles)
    % Load the .mat file
    load(sceneFiles{sceneIdx}, 'distance', 'relPosition', 'pred', 'Inpred', 'nop');  % Load distance, relPosition, pred, Inpred, and nop
    
    % Initialize distance, magnitude, and nop arrays based on the custom time frame
    numFrames = timeFrames(sceneIdx);
    d = NaN(1, numFrames);  % Allocate distance array
    magnitude = NaN(1, numFrames);  % Allocate magnitude array
    nopData = NaN(1, numFrames);  % Allocate nop array
    trajErrorData = [];
    
    % Initialize yDistList for this scene
    distList = [];
    predDTL = [];
distToLane = cell(1, numFrames);  % Preallocate 1xnumFrames cell array

    for i = 1:numFrames
        % Check if both pred and Distance cells are not empty
        if ~isempty(pred{i}) && ~isempty(distance{1,i})
            % Subtract the first value in Distance from the second value in pred
            distToLane{i} = pred{i}(1,2) - distance{1,i}(1);  % Use curly braces {} to assign to a cell
        else
            % If either is empty, leave the corresponding distToLane cell empty
            distToLane{i} = [];
        end
 
    
        %dataTable(sceneIdx).predDist2L=distToLane;
        
        % Convert distToLane to a regular array if desired
        %distToLane = cell2mat(distToLane);  % Convert to a numeric array if you prefer a matrix format

        if ~isempty(distance{1, i})
            idxTargetVeh = find(distance{1, i}(:,2) == targetVeh(sceneIdx));
            d(i) = distance{1, i}(idxTargetVeh, 1) * 100;  % Convert distance to cm
        end

        if ~isempty(relPosition{1, i})  % Ensure relPosition exists
            x = relPosition{1, i}(idxTargetVeh, 1);  % Get x position
            y = relPosition{1, i}(idxTargetVeh, 2);  % Get y position
            magnitude(i) = sqrt(x^2 + y^2);  % Calculate magnitude

            dist = distance{1, i}(1, 1);  % Get y position

            % Add the second value (y) to yDistList
            distList = [distList, dist];  
        end

        if ~isempty(nop{1, i})  % Process the nop variable
            idxTargetVehNop = find(nop{1, i}(:,2) == targetVeh(sceneIdx));
            if ~isempty(idxTargetVehNop)
                nopData(i) = nop{1, i}(idxTargetVehNop, 1);  % Get the number of points for the target vehicle
            end
        end

        % Process trajectory prediction error
        if ~isempty(pred{1, i}) && i < numFrames && ~isempty(Inpred{1, i+1})
            idxTargetVehPred = find(pred{1, i}(:,5) == targetVeh(sceneIdx));
            if ~isempty(idxTargetVehPred)
                xError = abs(pred{1, i}(idxTargetVehPred, 1) - Inpred{1, i+1}(idxTargetVehPred, 1));
                yError = Inpred{1, i+1}(idxTargetVehPred, 2) - pred{1, i}(idxTargetVehPred, 2);
                thetaError = abs(pred{1, i}(idxTargetVehPred, 3) - Inpred{1, i+1}(idxTargetVehPred, 3));
                zError = abs(pred{1, i}(idxTargetVehPred, 4) - Inpred{1, i+1}(idxTargetVehPred, 4));

                trajErrorData(i,:) = [xError * 100, yError * 100, sqrt(xError^2 + yError^2) * 100, thetaError, zError * 100];
            else
                trajErrorData(i,:) = [NaN, NaN, NaN, NaN, NaN];
            end
        else
            trajErrorData(i,:) = [NaN, NaN, NaN, NaN, NaN];
        end
    end

    % Create time array such that t = 0 ms is the last point
    t = -((numFrames - 1) * 100):100:0;

    % Store the calculated min, max, average, and std of distList
    dataTable(sceneIdx).distMin = min(distList);
    dataTable(sceneIdx).distMax = max(distList);
    dataTable(sceneIdx).distAvg = mean(distList);
    dataTable(sceneIdx).distSTD = std(distList);
    dataTable(sceneIdx).detectDist = max(magnitude);

    % Store for plotting
    d_all{sceneIdx} = d;
    t_all{sceneIdx} = t;
    magnitude_all{sceneIdx} = magnitude;
    trajError_all{sceneIdx} = trajErrorData;
    nop_all{sceneIdx} = nopData;

    % Filter out rows with NaN values from trajError_all for the current scene
    trajError_filtered = trajError_all{sceneIdx}(~all(isnan(trajError_all{sceneIdx}), 2), :);

    % Calculate trajAvgError using the filtered data
    if ~isempty(trajError_filtered)
        dataTable(sceneIdx).trajAvgError = mean(abs(trajError_filtered(:,2)))/100;  % Calculate the average error in the y component (2nd column)
    else
        dataTable(sceneIdx).trajAvgError = NaN;  % If no valid data, set to NaN
    end

    if ~isempty(trajError_filtered)
        dataTable(sceneIdx).trajSTD = std(trajError_filtered(:,2))/100;  % Calculate the average error in the y component (2nd column)
    else
        dataTable(sceneIdx).trajSTD = NaN;  % If no valid data, set to NaN
    end

    if ~isempty(trajError_filtered)
        dataTable(sceneIdx).trajMinError = min(abs(trajError_filtered(:,2)))/100;  % Calculate the average error in the y component (2nd column)
    else
        dataTable(sceneIdx).trajMinError = NaN;  % If no valid data, set to NaN
    end
    if ~isempty(trajError_filtered)
        dataTable(sceneIdx).trajMaxError = max(abs(trajError_filtered(:,2)))/100;  % Calculate the average error in the y component (2nd column)
    else
        dataTable(sceneIdx).trajMaxError = NaN;  % If no valid data, set to NaN
    end

        % Assuming 'nop' is your 1xm cell array
    nop_filtered = {};  % Initialize an empty cell array to store filtered values
    
    for i = 1:length(nop)
        if ~isempty(nop{i})  % Check if the cell is not empty
            nop_filtered{end+1} = nop{i}(1);  % Add the first number of the non-empty cell to nop_filtered
        end
    end
    
    % Convert nop_filtered to a regular array if desired
    nop_filtered = cell2mat(nop_filtered);  % Convert to a numeric array if you prefer a matrix format


    if ~isempty(nop_filtered)
        dataTable(sceneIdx).maxNop = max(nop_filtered);  % Calculate the average error in the y component (2nd column)
    else
        dataTable(sceneIdx).maxNop = NaN;  % If no valid data, set to NaN
    end



    % Linear fit and R^2 calculation
    validIndices = ~isnan(t) & ~isnan(magnitude);
    t_valid = t(validIndices);
    magnitude_valid = magnitude(validIndices);

    if length(t_valid) > 1
        % Perform linear fit (1st degree polynomial)
        p = polyfit(t_valid, magnitude_valid, 1);
        fitted_values = polyval(p, t_valid);

        % Store the slope (p(1) is the slope, p(2) is the intercept)
        vel = p(1);
        dataTable(sceneIdx).vel = vel * (-1000);

        % Compute R^2 value
        SS_res = sum((magnitude_valid - fitted_values).^2);  % Residual sum of squares
        SS_tot = sum((magnitude_valid - mean(magnitude_valid)).^2);  % Total sum of squares
        R2 = 1 - (SS_res / SS_tot);  % Coefficient of determination
        dataTable(sceneIdx).velR2 = R2;
    else
        % If there's not enough data, store NaN
        dataTable(sceneIdx).vel = NaN;
        dataTable(sceneIdx).velR2 = NaN;
    end
% Find indices of non-empty cells in distToLane
nonEmptyIndices = find(~cellfun('isempty', distToLane));

% Convert distToLane to a numeric array, ignoring empty cells
nonEmptyValues{sceneIdx} = cell2mat(distToLane(nonEmptyIndices));

% Count how many of the values are less than 0
countLessThanZero = sum(nonEmptyValues{sceneIdx} < 0);

% Store the count in the dataTable
dataTable(sceneIdx).predCrossings = countLessThanZero;

% Display or store the indices of non-empty cells if needed
% disp('Indices of non-empty cells:');
% disp(nonEmptyIndices);

timeValuesWithCorrespondingData{sceneIdx} = t_all{sceneIdx}(nonEmptyIndices);

end

%plots Predicted Vehicle Distance to Lane Marking
figure("Color", [1 1 1])

for sceneIdx = 1:length(sceneFiles)
    plot(timeValuesWithCorrespondingData{sceneIdx}, nonEmptyValues{sceneIdx}*100, 'LineWidth', 2, 'Color', colors(sceneIdx, :), 'Marker', markers{sceneIdx}, 'LineStyle', lineStyles{sceneIdx}, 'MarkerSize', 5)  % Line plot with markers
    hold on
end
hold off
%ylim([-1500 0])
grid minor
title('Predicted Vehicle Distance to Lane Marking', 'Interpreter', 'latex', 'FontWeight', 'bold')
xlabel('$t(ms)$', 'Interpreter', 'latex', 'FontWeight', 'bold')
ylabel('$cm$', 'Interpreter', 'latex', 'FontWeight', 'bold')
legend('Scen. 1', 'Scen. 2', 'Scen. 3', 'Scen. 4', 'Scen. 5', 'Scen. 6', 'Scen. 7', 'Scen. 8', 'Scen. 9', 'Location', 'best')
xlim([-1500 0])


% Line plot with markers for distance
figure("Color", [1 1 1])
hold on
for sceneIdx = 1:length(sceneFiles)
    plot(t_all{sceneIdx}, d_all{sceneIdx}, 'LineWidth', 2, 'Color', colors(sceneIdx, :), 'Marker', markers{sceneIdx}, 'LineStyle', lineStyles{sceneIdx}, 'MarkerSize', 5)  % Line plot with markers
end
hold off
ylim([0 500])
grid minor
title('Vehicle Distance to Lane Marking', 'Interpreter', 'latex', 'FontWeight', 'bold')
xlabel('$t(ms)$', 'Interpreter', 'latex', 'FontWeight', 'bold')
ylabel('$cm$', 'Interpreter', 'latex', 'FontWeight', 'bold')
legend('Scen. 1', 'Scen. 2', 'Scen. 3', 'Scen. 4', 'Scen. 5', 'Scen. 6', 'Scen. 7', 'Scen. 8', 'Scen. 9', 'Location', 'best')
xlim([-1500 0])

% Line plot with markers for magnitude
figure("Color", [1 1 1])
hold on
for sceneIdx = 1:length(sceneFiles)
    plot(t_all{sceneIdx}, magnitude_all{sceneIdx}, 'LineWidth', 2, 'Color', colors(sceneIdx, :), 'Marker', markers{sceneIdx}, 'LineStyle', lineStyles{sceneIdx}, 'MarkerSize', 5)  % Line plot with markers
end
hold off
ylim([0 max(cellfun(@max, magnitude_all)) * 1.1])  % Adjust y-limit based on magnitude
grid minor
title('BB Center Relative Distance', 'Interpreter', 'latex', 'FontWeight', 'bold')
xlabel('$t(ms)$', 'Interpreter', 'latex', 'FontWeight', 'bold')
ylabel('$m$', 'Interpreter', 'latex', 'FontWeight', 'bold')
legend('Scen. 1', 'Scen. 2', 'Scen. 3', 'Scen. 4', 'Scen. 5', 'Scen. 6', 'Scen. 7', 'Scen. 8', 'Scen. 9', 'Location', 'best')
xlim([-1500 0])

% Line plot with markers for trajectory prediction error (y component)
figure("Color", [1 1 1])
hold on
for sceneIdx = 1:length(sceneFiles)
    plot(t_all{sceneIdx}, trajError_all{sceneIdx}(:,2), 'LineWidth', 2, 'Color', colors(sceneIdx, :), 'Marker', markers{sceneIdx}, 'LineStyle', lineStyles{sceneIdx}, 'MarkerSize', 5)  % Line plot with markers
end
hold off
grid minor
title('Trajectory Prediction Error', 'Interpreter', 'latex', 'FontWeight', 'bold')
xlabel('$t(ms)$', 'Interpreter', 'latex', 'FontWeight', 'bold')
ylabel('cm', 'Interpreter', 'latex', 'FontWeight', 'bold')
legend('Scen. 1', 'Scen. 2', 'Scen. 3', 'Scen. 4', 'Scen. 5', 'Scen. 6', 'Scen. 7', 'Scen. 8', 'Scen. 9', 'Location', 'best')
xlim([-1500 0])

% Line plot for NOP with markers
figure("Color", [1 1 1])
hold on
for sceneIdx = 1:length(sceneFiles)
    plot(t_all{sceneIdx}, nop_all{sceneIdx}, 'LineWidth', 2, 'Color', colors(sceneIdx, :), 'Marker', markers{sceneIdx}, 'LineStyle', lineStyles{sceneIdx}, 'MarkerSize', 5);  % Plot with line and markers
end
hold off
grid minor
title('Number of Points (NOP) Over Time', 'Interpreter', 'latex', 'FontWeight', 'bold')
xlabel('$t(ms)$', 'Interpreter', 'latex', 'FontWeight', 'bold')
ylabel('Number of Points (NOP)', 'Interpreter', 'latex', 'FontWeight', 'bold')
legend('Scen. 1', 'Scen. 2', 'Scen. 3', 'Scen. 4', 'Scen. 5', 'Scen. 6', 'Scen. 7', 'Scen. 8', 'Scen. 9', 'Location', 'best')
xlim([-1500 0])

