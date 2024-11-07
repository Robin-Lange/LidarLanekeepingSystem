


%% to run this code, youn need to follow these steps:
% storage and nearIr by running dataFormatting
% load storage and nearIr into workspace
% I think thats all?

%to run the tracking code, you need to run the first part of
%vehicleTracking to get all variables loaded
%then run laneExample.mlx
%then run the whole vehicleTracking code
%Please not that vehicleTracking is not set up to use this lane detection
%yet. it will only use the abd one



clearvars -except storage nearIr
clc

% Create an empty cell array to store the peak points for each point cloud
peakPointsCellArray = {};
lastValidPolynomialLeft = [];  % Variable to store the last valid polynomial for the left side
lastValidPolynomialRight = []; % Variable to store the last valid polynomial for the right side

pcidx = 2000;
scatterHandleLeft = [];  % Handle for left lane scatter plot
scatterHandleRight = []; % Handle for right lane scatter plot

while pcidx <= length(storage)
    ptCloud0 = storage{pcidx}(1);
    roi = [-60 0 -4 12 -3 -1];
    
    YLaneSearch = [-3 3];
    
    indices = findPointsInROI(ptCloud0, roi);
    ptCloud1 = select(ptCloud0, indices);
    
    projectedPtCloud = ptCloud1.Location(:, 1:2);
    intensityPtCloud = ptCloud1.Intensity; 

    % Plot the point cloud
    figure(1)
    pcshow(ptCloud1.Location, ptCloud1.Intensity);
    view(2);
    colormap('jet');
    colorbar;

    figure(2)
    imshow(nearIr{1, pcidx}*6.5)

    bins = 20;  % must be a multiple of 4
    
    % ROI settings
    lengthOffSet = -4;
    lengths(1:14) = -0.5;
    lengths(15) = -1;
    lengths(16:20) = -2;

    lengths(1) = lengths(1) + 0.2;  % Custom modifications
    lengths(13) = lengths(13) - 0.25;
    lengths(14) = lengths(14) - 0.5;
    for i = 15:20
        lengths(i) = lengths(i) - (i - 15) * 0.2; % Larger increment for farther bins
    end

    i = 1;
    while i <= bins
        if i == 1
            birdRoiRange{i} = [lengths(1) + lengthOffSet, lengthOffSet, YLaneSearch(1), YLaneSearch(2)];
        else
            birdRoiRange{i} = [lengths(i) + sum(lengths(1:i-1)) + lengthOffSet, sum(lengths(1:i-1)) + lengthOffSet, YLaneSearch(1), YLaneSearch(2)];
        end
        i = i + 1;
    end

    i = 1;
    % Initialize arrays to hold the peaks for the entire point cloud
    allPeaks_x = [];
    allPeaks_y = [];

    while i <= bins
        birdRoiInd{i} = projectedPtCloud(:, 1) >= birdRoiRange{i}(1) ...
            & projectedPtCloud(:, 1) <= birdRoiRange{i}(2) & projectedPtCloud(:, 2) ...
            >= birdRoiRange{i}(3) & projectedPtCloud(:, 2) <= birdRoiRange{i}(4);
        birdRoi{i} = projectedPtCloud(birdRoiInd{i}, :);
        birdRoiInt{i} = intensityPtCloud(birdRoiInd{i});
        
        % Get the peaks for this bin
        x = birdRoi{i}(:, 2);
        yval = double(birdRoiInt{i}(:, 1));
        peaks_x = func(x, yval);
        
        % Calculate the center of the bin for y-values
        bin_center = mean(birdRoi{i}(:, 1));
        
        % Append the peaks from this bin to the overall list for the point cloud
        allPeaks_x = [allPeaks_x; peaks_x];
        allPeaks_y = [allPeaks_y; repmat(bin_center, length(peaks_x), 1)];
        
        i = i + 1;
    end

    % Store the compiled list of x and y peak points for the entire point cloud
    peakPointsCellArray{pcidx} = [allPeaks_x, allPeaks_y];

    % Ensure that peakPointsCellArray is not empty before accessing
    if ~isempty(peakPointsCellArray{pcidx}) && size(peakPointsCellArray{pcidx}, 2) >= 2
        j = 1;
        j1 = 1;
        j2 = 1;
        leftPeaks = [];
        rightPeaks = [];

        % Process the peak points, split into left and right lanes
        while j <= size(peakPointsCellArray{1, pcidx},1)
            if peakPointsCellArray{1, pcidx}(j, 1) < -1
                leftPeaks(j1, 1) = peakPointsCellArray{1, pcidx}(j, 1);
                leftPeaks(j1, 2) = peakPointsCellArray{1, pcidx}(j, 2);
                j1 = j1 + 1;
            elseif (peakPointsCellArray{1, pcidx}(j, 1) > 1)
                rightPeaks(j2, 1) = peakPointsCellArray{1, pcidx}(j, 1);
                rightPeaks(j2, 2) = peakPointsCellArray{1, pcidx}(j, 2);
                j2 = j2 + 1;
            end
            j = j + 1;
        end
    else
        leftPeaks = [];
        rightPeaks = [];
    end
    
    % Clear only the previous scatter plot for both lanes
    if ~isempty(scatterHandleLeft) && isvalid(scatterHandleLeft)
        delete(scatterHandleLeft); % Delete the previous scatter plot handle for left lane
    end
    if ~isempty(scatterHandleRight) && isvalid(scatterHandleRight)
        delete(scatterHandleRight); % Delete the previous scatter plot handle for right lane
    end
    
    % Plot left peaks in magenta and right peaks in red, but only if they exist
    figure(1)
    hold on
    if ~isempty(leftPeaks)
        scatterHandleLeft = scatter(leftPeaks(:, 2), leftPeaks(:, 1), 'magenta'); % Plot left lane points in magenta
    end
    if ~isempty(rightPeaks)
        scatterHandleRight = scatter(rightPeaks(:, 2), rightPeaks(:, 1), 'red');   % Plot right lane points in red
    end

    % Fit 3rd-degree polynomial for left side if there are more than 4 points
    if size(leftPeaks, 1) >= 3
        pLeft = polyfit(leftPeaks(:, 2), leftPeaks(:, 1), 2);  % Fit end-degree polynomial
        lastValidPolynomialLeft = pLeft;  % Store the last valid polynomial for future use
        % Generate y-values for plotting the polynomial, including extrapolation to y = 0
        y_fit_left = linspace(min(leftPeaks(:, 2)), -4, 200);  % Extend to y = 0
        x_fit_left = polyval(pLeft, y_fit_left);  % Compute the fitted x-values
        plot(y_fit_left, x_fit_left, 'magenta', 'LineWidth', 2);  % Plot the fitted polynomial
    end

    % Fit 3rd-degree polynomial for right side if there are more than 4 points
    if size(rightPeaks, 1) >= 3
        pRight = polyfit(rightPeaks(:, 2), rightPeaks(:, 1), 2);  % Fit 2nd-degree polynomial
        lastValidPolynomialRight = pRight;  % Store the last valid polynomial for future use
        % Generate y-values for plotting the polynomial, including extrapolation to y = 0
        y_fit_right = linspace(min(rightPeaks(:, 2)), -5, 100);  % Extend to y = 0
        x_fit_right = polyval(pRight, y_fit_right);  % Compute the fitted x-values
        plot(y_fit_right, x_fit_right, 'red', 'LineWidth', 2);  % Plot the fitted polynomial
    end
    hold off

    pcidx = pcidx + 1;
end

function [top_peaks_x] = func(x, y)
    % Sort x and y based on the increasing order of x
    [x_sorted, sort_idx] = sort(x);
    y_sorted = y(sort_idx);

    % Define MinPeakDistance
    min_peak_distance = 1.5;

    % Check if there are at least 3 data points and if the x range is larger than MinPeakDistance
    if length(x_sorted) >= 5 && (max(x_sorted) - min(x_sorted)) > min_peak_distance
        % Find peaks on the sorted data, with MinPeakDistance, MinPeakProminence, and MinPeakHeight
        [pks, locs] = findpeaks(y_sorted, x_sorted, 'MinPeakDistance', min_peak_distance, 'MinPeakProminence', 9, 'MinPeakHeight', 15);
        
        % Sort the peaks in descending order and get the top 3 peak locations
        [~, peak_sort_idx] = sort(pks, 'descend');
        top_peaks_x = locs(peak_sort_idx(1:min(3, length(peak_sort_idx))));
    else
        % Return an empty array if there are not enough points or the range is too small
        top_peaks_x = [];
    end
end

