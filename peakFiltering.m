function [lanePolynomials,peaks]=peakFiltering(allPeaks,confirmedTracks)
     % Remove peaks within vehicle bounding boxes
   allPeaks_x=allPeaks(:,2);
   allPeaks_y=allPeaks(:,1);
    for idx = length(allPeaks_x):-1:1 % Loop backward to safely remove elements
        peak_x = allPeaks_x(idx);
        peak_y = allPeaks_y(idx);
        inVehicleBox = false;

        % Check each vehicle bounding box
        for t = 1:length(confirmedTracks)
            state = confirmedTracks(t,1).State;

            % Mirror vehicle state coordinates for comparison
            vehicle_x = state(1);  % Mirror to -x
            vehicle_y = state(3);  % Mirror to -y

            % Apply 10% enlargement to the bounding box dimensions
            vehicle_length = state(9) * 1.1;  % Increase length by 10%
            vehicle_width = state(10) * 1.1;  % Increase width by 10%


            % Check if the peak is within the mirrored and enlarged bounding box
            %peak_x=peak_x*-1;
            %peak_y=peak_y*-1;
            % peak_y
            % figure(126789)
            % view(2)
            %     pcshow(ptCloud1.Location, ptCloud1.Intensity, 'MarkerSize', 50);
            %     view(2)
            %     hold on
            %     scatter(peak_y,peak_x);
            %     hold off
            if peak_x >= vehicle_x - vehicle_length/2 && peak_y <= vehicle_x + vehicle_length/2 && ... %abs
               peak_y >= vehicle_y - vehicle_width/2 && peak_y <= vehicle_y + vehicle_width/2
                % Print the headers once for clarity
%fprintf('%10s %10s %10s %10s %15s %15s\n', 'vehicle_x', 'peak_x', 'vehicle_y', 'peak_y', 'vehicle_length', 'vehicle_width');

%% Inside your loop where these variables are updated, use the following:
%fprintf('%10.2f %10.2f %10.2f %10.2f %15.2f %15.2f\n', vehicle_x, peak_x, vehicle_y, peak_y, vehicle_length, vehicle_width);
                %peak_y
               % peak_x
                %  figure(126789)
                % pcshow(ptCloud1.Location, ptCloud1.Intensity, 'MarkerSize', 50);
                % view(2)
                %  hold on
                % scatter(peak_y,peak_x);
                % hold off
                % figure(126789)
                % hold on
                % scatter(peak_y,peak_x);
                % hold off
                % figure(2555555)
                % hold on
                % scatter(peak_y,peak_x);
                % hold off
                %peak_x=peak_x*-1;
                %peak_y=peak_y*-1;
               % peak_x
                inVehicleBox = true;
                break; % Exit the loop if within a bounding box
            end
            %peak_x=peak_x*-1;
            %peak_y=peak_y*-1;
        end

        % Remove peak if within any vehicle bounding box
        if inVehicleBox
            allPeaks_x(idx) = [];
            allPeaks_y(idx) = [];
        end
    end
    peaks(:,2)=allPeaks_x;
    peaks(:,1)=allPeaks_y;
    degree=2;
    if size(allPeaks_x, 1) >= 3
        pLeft = polyfit( allPeaks_x,  allPeaks_y, degree);
        lanePolynomials(1,1:length(pLeft))=pLeft;
    else
        lanePolynomials(1,1:degree+1)=0;
    end

    % if size(rightPeaks, 1) >= 3
    %     pRight = polyfit(rightPeaks(:, 2), rightPeaks(:, 1), degree);
    %     lanePolynomials(1,1:length(pRight))=pRight;
    % else
    %     lanePolynomials(1,1:degree+1)=0;
    % end
end


    % 
    % 
    % 
    % 
    % 
    % 
    % 
    % 
    % for side=1:1:2
    %     if side==1
    %     allPeaks_y=leftPeaks(:,1);
    %     allPeaks_x=leftPeaks(:,2);
    %     else
    %     allPeaks_y=rightPeaks(:,1);
    %     allPeaks_x=rightPeaks(:,2);    
    %     end
    % 
    %      for idx = length(allPeaks_x):-1:1 % Loop backward to safely remove elements
    %         peak_x = allPeaks_x(idx);
    %         peak_y = allPeaks_y(idx);
    %         inVehicleBox = false;
    % 
    %         % Check each vehicle bounding box
    %         for t = 1:length(confirmedTracks)
    %             state = confirmedTracks(t,1).State;
    % 
    %             % Mirror vehicle state coordinates for comparison
    %             vehicle_x = state(1);  % Mirror to -x
    %             vehicle_y = state(3);  % Mirror to -y
    % 
    %             % Apply 10% enlargement to the bounding box dimensions
    %             vehicle_length = state(9) * 10;  % Increase length by 10%
    %             vehicle_width = state(10) * 10;  % Increase width by 10%
    % 
    % 
    %             % Check if the peak is within the mirrored and enlarged bounding box
    %             %peak_x=peak_x*-1;
    %             %peak_y=peak_y*-1;
    %             % peak_y
    %             % figure(126789)
    %             % view(2)
    %             %     pcshow(ptCloud1.Location, ptCloud1.Intensity, 'MarkerSize', 50);
    %             %     view(2)
    %             %     hold on
    %             %     scatter(peak_y,peak_x);
    %             %     hold off
    %             if peak_x >= vehicle_x - vehicle_length/2 && peak_y <= vehicle_x + vehicle_length/2 && ... %abs
    %                peak_y >= vehicle_y - vehicle_width/2 && peak_x <= vehicle_y + vehicle_width/2
    %                  peak_y
    %                  peak_x
    %                  figure(1256789)
    %                  pcshow(ptCloud1.Location, ptCloud1.Intensity, 'MarkerSize', 50);
    %                  view(2)
    %                  hold on
    %                  scatter(peak_y,peak_x);
    %                  hold off
    %                 % figure(2555555)
    %                 % hold on
    %                 % scatter(peak_y,peak_x);
    %                 % hold off
    %                 %peak_x=peak_x*-1;
    %                 %peak_y=peak_y*-1;
    %                % peak_x
    %                 inVehicleBox = true;
    %                 break; % Exit the loop if within a bounding box
    %             end
    %             %peak_x=peak_x*-1;
    %             %peak_y=peak_y*-1;
    %         end
    % 
    %         % Remove peak if within any vehicle bounding box
    %         if inVehicleBox
    %             allPeaks_x(idx) = [];
    %             allPeaks_y(idx) = [];
    %         end
    % 
    %      end
    % 
    %     degree=2;
    %     if side==1
    %         if size(leftPeaks, 1) >= 3
    %             pLeft = polyfit(leftPeaks(:, 2), leftPeaks(:, 1), degree);
    %             lanePolynomials(2,1:length(pLeft))=pLeft;
    %         else
    %             lanePolynomials(2,1:degree+1)=0;
    %         end
    %     end
    %     if side==2
    %         if size(rightPeaks, 1) >= 3
    %             pRight = polyfit(rightPeaks(:, 2), rightPeaks(:, 1), degree);
    %             lanePolynomials(1,1:length(pRight))=pRight;
    %         else
    %             lanePolynomials(1,1:degree+1)=0;
    %         end
    %     end
    % end
    % 
    % 
    % 
    % % degree=2;
    % % if size(leftPeaks, 1) >= 3
    % %     pLeft = polyfit(leftPeaks(:, 2), leftPeaks(:, 1), degree);
    % %     lanePolynomials(2,1:length(pLeft))=pLeft;
    % % else
    % %     lanePolynomials(2,1:degree+1)=0;
    % % end
    % % 
    % % if size(rightPeaks, 1) >= 3
    % %     pRight = polyfit(rightPeaks(:, 2), rightPeaks(:, 1), degree);
    % %     lanePolynomials(1,1:length(pRight))=pRight;
    % % else
    % %     lanePolynomials(1,1:degree+1)=0;
    % % end