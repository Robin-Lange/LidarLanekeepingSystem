function [lanePolynomials,leftPeaks,rightPeaks,peakPointsCellArray]=Attempt2LaneFindingFunction(inptStorage,confirmedTracks)
%function [lanePolynomials,leftPeaks,rightPeaks,peakPointsCellArray]=Attempt2LaneFindingFunction(inptStorage,confirmedTracks)
% inptStorage=lidarDataOrganized{520};
storage{1}=inptStorage;

% Create an empty cell array to store the peak points for each point cloud
peakPointsCellArray = {};
pcidx = 1;

rotationAngles = [0 0 180]; % Rotates it to work on the -x axis
translation = [0 0 0];
tform = rigidtform3d(rotationAngles,translation);

i=1;
while i<=length(storage)
    storage{1,i} = pctransform(storage{1,i},tform);
    i=i+1;
end

while pcidx <= length(storage)
    ptCloud0 = storage{pcidx}(1);
    roi = [-60 0 -4 12 -3 -1];
    
    YLaneSearch = [-3 3];
    
    indices = findPointsInROI(ptCloud0, roi);
    ptCloud1 = select(ptCloud0, indices);
    
    projectedPtCloud = ptCloud1.Location(:, 1:2);
    intensityPtCloud = ptCloud1.Intensity; 

    bins = 32;  % must be a multiple of 4

    lengths(1:7) = -0.5;
    lengths(8:14) = -0.75;
    lengths(15:22) = -1;
    lengths(23:32) = -2;
    lengthOffSet = -3.5;

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

    % % Remove peaks within vehicle bounding boxes
    % for idx = length(allPeaks_x):-1:1 % Loop backward to safely remove elements
    %     peak_x = allPeaks_x(idx);
    %     peak_y = allPeaks_y(idx);
    %     inVehicleBox = false;
    % 
    %     % Check each vehicle bounding box
    %     for t = 1:length(confirmedTracks)
    %         state = confirmedTracks(t,1).State;
    % 
    %         % Mirror vehicle state coordinates for comparison
    %         vehicle_x = state(3);  % Mirror to -x
    %         vehicle_y = state(1);  % Mirror to -y
    % 
    %         % Apply 10% enlargement to the bounding box dimensions
    %         vehicle_length = state(9) * 10;  % Increase length by 10%
    %         vehicle_width = state(10) * 10;  % Increase width by 10%
    % 
    % 
    %         % Check if the peak is within the mirrored and enlarged bounding box
    %         peak_x=peak_x*-1;
    %         %peak_y=peak_y*-1;
    %         % peak_y
    %         % figure(126789)
    %         % view(2)
    %         %     pcshow(ptCloud1.Location, ptCloud1.Intensity, 'MarkerSize', 50);
    %         %     view(2)
    %         %     hold on
    %         %     scatter(peak_y,peak_x);
    %         %     hold off
    %         if peak_x >= vehicle_x - vehicle_length/2 && peak_y <= vehicle_x + vehicle_length/2 && ... %abs
    %            peak_y >= vehicle_y - vehicle_width/2 && peak_x <= vehicle_y + vehicle_width/2
    %             peak_y
    %             peak_x
    %              figure(126789)
    %             pcshow(ptCloud1.Location, ptCloud1.Intensity, 'MarkerSize', 50);
    %             view(2)
    %              hold on
    %             scatter(peak_y,peak_x);
    %             hold off
    %             % figure(126789)
    %             % hold on
    %             % scatter(peak_y,peak_x);
    %             % hold off
    %             % figure(2555555)
    %             % hold on
    %             % scatter(peak_y,peak_x);
    %             % hold off
    %             %peak_x=peak_x*-1;
    %             %peak_y=peak_y*-1;
    %            % peak_x
    %             inVehicleBox = true;
    %             break; % Exit the loop if within a bounding box
    %         end
    %         %peak_x=peak_x*-1;
    %         %peak_y=peak_y*-1;
    %     end
    % 
    %     % Remove peak if within any vehicle bounding box
    %     if inVehicleBox
    %         allPeaks_x(idx) = [];
    %         allPeaks_y(idx) = [];
    %     end
    % end


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
            if peakPointsCellArray{1, pcidx}(j, 1) < -.5
                leftPeaks(j1, 1) = -peakPointsCellArray{1, pcidx}(j, 1);
                leftPeaks(j1, 2) = -peakPointsCellArray{1, pcidx}(j, 2);
                j1 = j1 + 1;
            elseif (peakPointsCellArray{1, pcidx}(j, 1) > .5)
                rightPeaks(j2, 1) = -peakPointsCellArray{1, pcidx}(j, 1);
                rightPeaks(j2, 2) = -peakPointsCellArray{1, pcidx}(j, 2);
                j2 = j2 + 1;
            end
            j = j + 1;
        end
    else
        leftPeaks = [];
        rightPeaks = [];
    end
    
    degree=2;
    if size(leftPeaks, 1) >= 3
        pLeft = polyfit(leftPeaks(:, 2), leftPeaks(:, 1), degree);
        lanePolynomials(2,1:length(pLeft))=pLeft;
    else
        lanePolynomials(2,1:degree+1)=0;
    end

    if size(rightPeaks, 1) >= 3
        pRight = polyfit(rightPeaks(:, 2), rightPeaks(:, 1), degree);
        lanePolynomials(1,1:length(pRight))=pRight;
    else
        lanePolynomials(1,1:degree+1)=0;
    end
    
    pcidx = pcidx + 1;
end
end


function [top_peaks_x] = func(x, y)
    % Sort x and y based on the increasing order of x
    [x_sorted, sort_idx] = sort(x);
    y_sorted = y(sort_idx);

    % Define MinPeakDistance
    min_peak_distance = 2.5;
    % ma=max(y_sorted)
    % %display(ma)
    % mi=min(y_sorted)
    if (max(y_sorted)>25) %the first method works much better for high intensities
        % Check if there are at least 3 data points and if the x range is larger than MinPeakDistance
        if length(x_sorted) >= 5 && (max(x_sorted) - min(x_sorted)) > min_peak_distance
            % Find peaks on the sorted data, with MinPeakDistance, MinPeakProminence, and MinPeakHeight
            [pks, locs] = findpeaks(y_sorted, x_sorted, 'MinPeakDistance', min_peak_distance, 'MinPeakProminence', 9, 'MinPeakHeight', 13); %2.5 9 13
            
            % Sort the peaks in descending order and get the top 3 peak locations
            [~, peak_sort_idx] = sort(pks, 'descend');
            top_peaks_x = locs(peak_sort_idx(1:min(3, length(peak_sort_idx))));
        else
            % Return an empty array if there are not enough points or the range is too small
            top_peaks_x = [];
        end
    else %this method works better for low intensities
            % Check if there are at least 3 data points and if the x range is larger than MinPeakDistance
        if length(x_sorted) >= 5 && (max(x_sorted) - min(x_sorted)) > min_peak_distance
            % Find peaks on the sorted data, with MinPeakDistance, MinPeakProminence, and MinPeakHeight
            [pks, locs] = findpeaks(y_sorted, x_sorted, 'MinPeakDistance', min_peak_distance, 'MinPeakProminence', (max(y_sorted)-min(y_sorted))*.45, 'MinPeakHeight', ((max(y_sorted)-min(y_sorted))*0.65)+min(y_sorted)); %2.5 9 13
            
            % Sort the peaks in descending order and get the top 3 peak locations
            [~, peak_sort_idx] = sort(pks, 'descend');
            top_peaks_x = locs(peak_sort_idx(1:min(3, length(peak_sort_idx))));
        else
            % Return an empty array if there are not enough points or the range is too small
            top_peaks_x = [];
        end
    end
end