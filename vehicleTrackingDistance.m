%% General Instructions
%this is the main code. It has the vehicle tracking and it calls functions
%for the lane tracking and distance calculating
%to run it, load 1 of the next 2 sections, depending on if you want all the
%data or just from 1 scenario. 
%the file that calculates the lanes is named "Attempt2LaneFindingFunction"
%the file that calculates distance to center line is called
%"distanceFinding"
%The distance to center line and the tracking id of the vehicle that made
%it for each point cloud are in the variable "distance"
%if you want to look at just a camera view of the driving, use the file
%"scenarioFinder" this fill is just the camera view and will run much
%faster. I used it to note down potential scenarios and the tested them
%after with this code
%if you need any other info, please ask Luke

% %% Run this section if you want a specific scene
% %before running this section, you need to download the scene data files
% %from the email I sent you
% clear all;close all;clc
% load("scene9.mat")
%% Run this section instead if you want all the data
%if you want to use this section, you will need to run "dataFormatting"
%first to format and save the data
tic
clearvars -except ptCloud nearIr lidarData imageData;
clc;
%close all;
%imageData=load("nearIr_130616.mat"); %file not needed
%if ( ~(exist('lidarData','var'))) | (~(exist('lidarDataOrganized','var')) )
if ( ~(exist('lidarData','var')))
 if ( ~(exist('ptCloud','var') ) )
 % croppedPtCloud=load("storage_130616");
 % croppedPtCloud=croppedPtCloud.storage;
 %ptCloud=load("storage_130040");
 ptCloud=load("storage_130616");
 ptCloud=ptCloud.storage;
 
 rotationAngles = [0 0 180];
 translation = [0 0 0];
 tform = rigidtform3d(rotationAngles,translation);
 i=1;
 while i<=length(ptCloud)
 ptCloud{1,i} = pctransform(ptCloud{1,i},tform);
 %croppedPtCloud{1,i} = pctransform(croppedPtCloud{1,i},tform);
 i=i+1;
 end
 %clearvars -except ptCloud nearIr;
 end
 %lidarDataOrganized=ptCloud;
 %clearvars -except croppedPtCloud ptCloud imageData lidarData;
 for i=1:1:length(ptCloud) % 350 is because that is how many camera pictures are in example data
 lidarData{i}=removeInvalidPoints(ptCloud{i}); % deorganizes it
 %imageData{i}=rand(144,192,3)*255; %makes fake data to display so no errors
 end
 lidarData=lidarData'; %needs to be x by 1
end
if ( ~(exist('imageData','var')) )
 if ( ~(exist('nearIr','var')) )
 %nearIr=load("nearIr_130040");
 nearIr=load("nearIr_130616");
 nearIr=nearIr.nearIr;
 end
 % Assuming 'nearInfrared' is your 1xM cell array with the NIR data
 M = length(nearIr); % Number of frames in the cell array
 outputArray = cell(1, M); % Initialize the output cell array
 % Define the output dimensions
 height = 144; % Output height
 width = 192; % Output width
 % Loop through each frame
 for frameIdx = 1:M
 % Extract the NIR data
 nirData = nearIr{frameIdx}; % Get the current NIR frame
 nirData = imresize(nirData, [height, width]); % Resize to desired output dimensions
 
 % Normalize intensity values to range [0, 1]
 nirDataNorm = double(nirData) *2/ 65535; % Assuming max intensity is 65535
 
 % Create a grayscale image
 grayscaleImage = uint8(nirDataNorm * 255); % Scale to 0-255 and convert to uint8
 
 % Store the grayscale image in the output array
 imageData{frameIdx} = grayscaleImage; % Store as 144x192 grayscale image
 %needs to be 1 x M
 end
end
toc
%clearvars -except ptCloud nearIr lidarData imageData; %trying to release some memory
%clearvars -except lidarData imageData updatedLanePolynomials lanes lanePolynomials;
%% Track Vehicles Using Lidar: From Point Cloud to Track List
% This example shows how to track vehicles using measurements from a
% lidar sensor mounted on top of an ego vehicle. Lidar sensors report
% measurements as a point cloud. The example illustrates the workflow in
% MATLAB(R) for processing the point cloud and tracking the objects. For a
% Simulink(R) version of the example, refer to
% <docid:fusion_ug#mw_8631e470-7b28-4996-ba40-6e24e44db931 Track
% Vehicles Using Lidar Data in Simulink>.The lidar data used in this
% example is recorded from a highway driving scenario. In this example, you
% use the recorded data to track vehicles with a joint probabilistic data
% association (JPDA) tracker and an interacting multiple model (IMM)
% approach. 
% Copyright 2019-2023 The MathWorks, Inc.
%% 3-D Bounding Box Detector Model
% Due to high resolution capabilities of the lidar sensor, each scan from
% the sensor contains a large number of points, commonly known as a point
% cloud. This raw data must be preprocessed to extract objects of interest,
% such as cars, cyclists, and pedestrians. In this example, you use a
% classical segmentation algorithm using a distance-based clustering
% algorithm. For more details about segmentation of lidar data into objects
% such as the ground plane and obstacles, refer to the
% <docid:driving_ug#mw_a3ea0987-6c9e-4c5c-8e16-24c527f9de23 Ground Plane
% and Obstacle Detection Using Lidar> example. For a deep learning
% segmentation workflow, refer to the
% <docid:lidar_ug#mw_6aaaa7c1-dcf1-4aac-8cf6-515edade3b81 Detect, Classify,
% And Track Vehicles Using Lidar> example. In this example, the point
% clouds belonging to obstacles are further classified into clusters using
% the |pcsegdist| function, and each cluster is converted to a bounding box
% detection with the following format:
%
% $[x\ y\ z\ {\theta}\ l\ w\ h]$
%
% $x$, $y$ and $z$ refer to the x-, y- and z-positions of the bounding box,
% ${\theta}$ refers to its yaw angle and $l$, $w$ and $h$ refer to its
% length, width, and height, respectively. The
% <docid:lidar_ref#mw_6b6e84fa-afc6-4ece-b0ef-cbbe6d9072e2 pcfitcuboid>
% function uses L-shape fitting algorithm to determine the yaw angle of
% the bounding box.
% 
% The detector is implemented by a supporting class
% |HelperBoundingBoxDetector|, which wraps around point cloud segmentation
% and clustering functionalities. An object of this class accepts a
% |pointCloud| input and returns a list of |objectDetection| objects with
% bounding box measurements.
%
% The diagram shows the processes involved in the bounding box detector
% model and the Lidar Toolbox(TM) functions used to implement
% each process. It also shows the properties of the supporting class that
% control each process.
%
% <<../HelperBBDetector.png>>
%
% The lidar data is available at the following location:
% <https://ssd.mathworks.com/supportfiles/lidar/data/TrackVehiclesUsingLidarExampleData.zip>
%
% Download the data files into your temporary directory, whose location is
% specified by MATLAB's |tempdir| function. If you want to place the files
% in a different folder, change the directory name in the subsequent
% instructions.
% Load data if unavailable. The lidar data is stored as a cell array of
% pointCloud objects.
% if ~exist('lidarData','var')
% dataURL = 'https://ssd.mathworks.com/supportfiles/lidar/data/TrackVehiclesUsingLidarExampleData.zip';
% datasetFolder = fullfile(tempdir,'LidarExampleDataset');
% if ~exist(datasetFolder,'dir') || numel(dir(fullfile(datasetFolder,"*.mat"))) ~= 6
% unzip(dataURL,datasetFolder);
% end
% % Specify initial and final time for simulation.
% initTime = 0;
% finalTime = 35;
% [lidarData, imageData] = loadLidarAndImageData(datasetFolder,initTime,finalTime);
% end
% Set random seed to generate reproducible results.
S = rng(2018);
 % 'XLimits',[-50 75],... % min-max
 % 'YLimits',[-5 5],... % min-max
 % 'ZLimits',[-2 5],... % min-max
% A bounding box detector model.
detectorModel = HelperBoundingBoxDetector(...
 'XLimits',[-50 75],... % min-max
 'YLimits',[-5 5],... % min-max
 'ZLimits',[-2 5],... % min-max
 'SegmentationMinDistance',1.8,... % minimum Euclidian distance
 'MinDetectionsPerCluster',1,... % minimum points per cluster
 'MeasurementNoise',blkdiag(0.25*eye(3),25,eye(3)),... % measurement noise in detection report
 'GroundMaxDistance',0.3); % maximum distance of ground points from ground plane
%% Target State and Sensor Measurement Model
% The first step in tracking an object is defining its state, and the
% models that define the transition of state and the corresponding
% measurement. These two sets of equations are collectively known as
% the state-space model of the target. To model the state of vehicles for
% tracking using lidar, this example uses a cuboid model with following
% convention:
%
% $x = [x_{kin}\ {\theta}\ l\ w\ h]$
%
% $x_{kin}$ refers to the portion of the state that controls the
% kinematics of the motion center, and $\theta$ is the yaw angle. The
% length, width, and height of the cuboid are modeled as constants, whose
% estimates evolve in time during correction stages of the filter.
%
% In this example, you use two state-space models: a constant velocity (cv)
% cuboid model and a constant turn-rate (ct) cuboid model. These models
% differ in the way they define the kinematic part of the state, as
% described below:
%
% $x_{cv} = [x\ {\dot{x}}\ y\ {\dot{y}}\ z\ {\dot{z}}\ {\theta}\ l\ w\ h]$
%
% $x_{ct} = [x\ {\dot{x}}\ y\ {\dot{y}}\ {\dot{\theta}}\ z\ {\dot{z}}\ {\theta}\ l\ w\ h]$
%
% For information about their state transition, refer to the
% |helperConstvelCuboid| and |helperConstturnCuboid| functions used in this
% example.
%
% The |helperCvmeasCuboid| and |helperCtmeasCuboid| measurement models
% describe how the sensor perceives the constant velocity and constant
% turn-rate states respectively, and they return bounding box measurements.
% Because the state contains information about size of the target, the
% measurement model includes the effect of center-point offset and bounding
% box shrinkage, as perceived by the sensor, due to effects like
% self-occlusion [1]. This effect is modeled by a shrinkage factor that is
% directly proportional to the distance from the tracked vehicle to the
% sensor.
%
% The image below demonstrates the measurement model operating at different
% state-space samples. Notice the modeled effects of bounding box shrinkage
% and center-point offset as the objects move around the ego vehicle.
%
% <<../LidarModel.gif>>
%% Set Up Tracker and Visualization
% The image below shows the complete workflow to obtain a list of tracks
% from a pointCloud input.
%
% <<../TrackVehiclesUsingLidarExamplePipeline.PNG>>
%
% Now, set up the tracker and the visualization used in the example.
%
% A joint probabilistic data association tracker (|trackerJPDA|)
% coupled with an IMM filter (|trackingIMM|) is used to track objects
% in this example. The IMM filter uses a constant velocity and constant
% turn-rate model and is initialized using the supporting function,
% |helperInitIMMFilter|, included with this example. The IMM approach helps
% a track to switch between motion models and thus achieve good estimation
% accuracy during events like maneuvering or lane changing. The animation
% below shows the effect of mixing the constant velocity and constant
% turn-rate model during prediction stages of the IMM filter.
%
% <<../immPrediction.gif>>
% 
%%
% The IMM filter updates the probability of each model when it is corrected
% with detections from the object. The animation below shows the estimated
% trajectory of a vehicle during a lane change event and the corresponding
% estimated probabilities of each model.
%
% <<../immLaneChange.gif>>
%
%%
% Set the |HasDetectableTrackIDsInput| property of the tracker as |true|,
% which enables you to specify a state-dependent probability of detection.
% The detection probability of a track is calculated by the
% |helperCalcDetectability| function, listed at the end of this example.
assignmentGate = [75 1000]; % Assignment threshold;
confThreshold = [7 10]; % Confirmation threshold for history logic
delThreshold = [8 10]; % Deletion threshold for history logic
Kc = 1e-9; %1e-9 % False-alarm rate per unit volume
% IMM filter initialization function
filterInitFcn = @helperInitIMMFilter;
% A joint probabilistic data association tracker with IMM filter
tracker = trackerJPDA('FilterInitializationFcn',filterInitFcn,...
 'TrackLogic','History',...
 'AssignmentThreshold',assignmentGate,...
 'ClutterDensity',Kc,...
 'ConfirmationThreshold',confThreshold,...
 'DeletionThreshold',delThreshold,...
 'HasDetectableTrackIDsInput',true,...
 'InitializationThreshold',0,...
 'HitMissThreshold',0.1);
%%
% The visualization is divided into these main categories:
%
% # Lidar Preprocessing and Tracking - This display shows the raw point
% cloud, segmented ground, and obstacles. It also shows the resulting
% detections from the detector model and the tracks of vehicles generated
% by the tracker.
% # Ego Vehicle Display - This display shows the 2-D bird's-eye view of the
% scenario. It shows the obstacle point cloud, bounding box detections, and
% the tracks generated by the tracker. For reference, it also displays the
% image recorded from a camera mounted on the ego vehicle and its field of
% view.
% # Tracking Details - This display shows the scenario zoomed around the
% ego vehicle. It also shows finer tracking details, such as error
% covariance in estimated position of each track and its motion model
% probabilities, denoted by cv and ct.
% Create display
displayObject = HelperLidarExampleDisplay(imageData{1},...
 'PositionIndex',[1 3 6],...
 'VelocityIndex',[2 4 7],...
 'DimensionIndex',[9 10 11],...
 'YawIndex',8,...
 'MovieName','',... % Specify a movie name to record a movie.
 'RecordGIF',false,...
 'lanePolynomials',[0 0 0;0 0 0]); % Specify true to record new GIFs
%% Loop Through Data
% Loop through the recorded lidar data, generate detections from the
% current point cloud using the detector model and then process the
% detections using the tracker.
time = 0; % Start time
dT = 0.1; % Time step
% Initiate all tracks.
allTracks = struct([]);
% Initiate variables for comparing MATLAB and MEX simulation.
numTracks = zeros(numel(lidarData),2);
% Loop through the data
start=6200;
endidx=length(lidarData);
for i = start:endidx
 % Update time
 time = time + dT;
 
 % Get current lidar scan
 currentLidar = lidarData{i};
 
 % Generator detections from lidar scan.
 [detections,obstacleIndices,groundIndices,croppedIndices] = detectorModel(currentLidar,time);
 
 % Calculate detectability of each track.
 detectableTracksInput = helperCalcDetectability(allTracks,[1 3 6]);
 
 % Pass detections to track.
 [confirmedTracks,tentativeTracks,allTracks,info] = tracker(detections,time,detectableTracksInput);
 numTracks(i,1) = numel(confirmedTracks);
 
 % Get model probabilities from IMM filter of each track using
 % getTrackFilterProperties function of the tracker.
 modelProbs = zeros(2,numel(confirmedTracks));
 for k = 1:numel(confirmedTracks)
 c1 = getTrackFilterProperties(tracker,confirmedTracks(k).TrackID,'ModelProbabilities');
 modelProbs(:,k) = c1{1};
 end
 %add lane detection
 %lanePolynomials=laneFinderFunction(currentLidar);
 %[lanePolys{i},leftPeaks,rightPeaks,peakPointsCellArray]=Attempt2LaneFindingFunction(currentLidar,confirmedTracks);
 [lanePolys{i},leftPeaks,rightPeaks,peakPointsCellArray]=Attempt2LaneFindingFunction(currentLidar,confirmedTracks);
 rightPeaksOld=rightPeaks;
 leftPeaksOld=leftPeaks;
 [lanePolys{i}(1,:),rightPeaks]=peakFiltering(rightPeaks,confirmedTracks);
 [lanePolys{i}(2,:),leftPeaks]=peakFiltering(leftPeaks,confirmedTracks);


 if i > start
 if abs(lanePolys{i}(2,1)) > .09/2
 lanePolys{i}(2,:) = lanePolys{i-1}(2,:);
 end
 if abs(lanePolys{i}(1,1)) > .09/2 
 lanePolys{i}(1,:) = lanePolys{i-1}(1,:);
 end
 if abs(lanePolys{i}(2,1)) == 0 && abs(lanePolys{i-1}(2,1)) ~= 0
 lanePolys{i}(2,:) = lanePolys{i-1}(2,:);
 end
 if abs(lanePolys{i}(1,1)) == 0 && abs(lanePolys{i-1}(1,1)) ~= 0 
 lanePolys{i}(1,:) = lanePolys{i-1}(1,:);
 end
 end
 
 %right lane is row 1 of lanePolys
 
 oncomingROI=[0 50 .5 10]; % -x +x -y +y 
 oncomingMinSpeed=-3;
 unicorns=1;
 c=1;
 while unicorns<=length(confirmedTracks)
 %check the position of each of the confirmed tracks
 %if position looks like a posible oncoming vehicle, call dist
 %function, store other data
 if ((confirmedTracks(unicorns,1).State(1,1)>= oncomingROI(1,1)) & (confirmedTracks(unicorns,1).State(1,1)<= oncomingROI(1,2)))
 if ((confirmedTracks(unicorns,1).State(3,1)>= oncomingROI(1,3)) & (confirmedTracks(unicorns,1).State(3,1)<= oncomingROI(1,4)))
 if (confirmedTracks(unicorns,1).State(2,1) <= oncomingMinSpeed)
 relPosition{i-(start-1)}(c,1)= confirmedTracks(unicorns,1).State(1,1);
 relPosition{i-(start-1)}(c,2)= confirmedTracks(unicorns,1).State(3,1);
 relPosition{i-(start-1)}(c,3)=confirmedTracks(unicorns,1).TrackID;
 distance{i-(start-1)}(c,1)=distanceFinding(lanePolys{i}(2,:),confirmedTracks(unicorns,1).State(1,1),confirmedTracks(unicorns,1).State(3,1),...
 confirmedTracks(unicorns,1).State(10,1),confirmedTracks(unicorns,1).State(9,1));
 distance{i-(start-1)}(c,2)=confirmedTracks(unicorns,1).TrackID;

 unassignedProb(i) = info.Clusters{1, 1}.MarginalProbabilities(end,1);

 xmin{i-(start-1)}(c,1) = confirmedTracks(unicorns,1).State(1,1) - ...
     confirmedTracks(unicorns,1).State(9,1)/2;
 xmax{i-(start-1)}(c,1) = confirmedTracks(unicorns,1).State(1,1) + ...
     confirmedTracks(unicorns,1).State(9,1)/2;

 ymin{i-(start-1)}(c,1) = confirmedTracks(unicorns,1).State(3,1) - ...
     confirmedTracks(unicorns,1).State(10,1)/2;
 ymax{i-(start-1)}(c,1) = confirmedTracks(unicorns,1).State(3,1) + ...
     confirmedTracks(unicorns,1).State(10,1)/2;

 zmin{i-(start-1)}(c,1) = confirmedTracks(unicorns,1).State(6,1) - ...
     confirmedTracks(unicorns,1).State(11,1)/2;
 zmax{i-(start-1)}(c,1) = confirmedTracks(unicorns,1).State(6,1) + ...
     confirmedTracks(unicorns,1).State(11,1)/2;
 
 roiNop = [xmin{i-(start-1)}(c,1) xmax{i-(start-1)}(c,1)...
     ymin{i-(start-1)}(c,1) ymax{i-(start-1)}(c,1) zmin{i-(start-1)}(c,1)...
     zmax{i-(start-1)}(c,1)];
 
 obsPC = pointCloud(currentLidar.Location(obstacleIndices, :));

 nop{i-(start-1)}(c,1) = length(findPointsInROI(obsPC,roiNop));
 nop{i-(start-1)}(c,2) = confirmedTracks(unicorns,1).TrackID;

 % Known initial conditions
xin = confirmedTracks(unicorns,1).State(1,1);  % initial x position (m)
yin = confirmedTracks(unicorns,1).State(3,1);  % initial y position (m)
zin = confirmedTracks(unicorns,1).State(6,1);  % initial z position (m)
thetain = confirmedTracks(unicorns,1).State(8,1);  % initial orientation (radians)

% Known velocities (derivatives)
dxdt = confirmedTracks(unicorns,1).State(2,1);  % velocity in x direction (m/s)
dydt = confirmedTracks(unicorns,1).State(4,1);  % velocity in y direction (m/s)
dzdt = confirmedTracks(unicorns,1).State(7,1);  % velocity in z direction (m/s)
dthetadt = confirmedTracks(unicorns,1).State(5,1);  % angular velocity (rad/s)

% Time parameters
dt = 0.01;  % time step (s)
T = 0.1;  % total time (500 ms = 0.5 s)
timetraj = 0:dt:T;  % time vector

% Initialize arrays to store trajectory
x_traj = zeros(size(timetraj));
y_traj = zeros(size(timetraj));
z_traj = zeros(size(timetraj));
theta_traj = zeros(size(timetraj));

% Set initial conditions in arrays
x_traj(1) = xin;
y_traj(1) = yin;
z_traj(1) = zin;
theta_traj(1) = thetain;



% Simulate the trajectory
for traji = 2:length(timetraj)
    % Update positions and orientation using simple Euler integration
    x_traj(traji) = x_traj(traji-1) + dxdt * dt;
    y_traj(traji) = y_traj(traji-1) + dydt * dt;
    z_traj(traji) = z_traj(traji-1) + dzdt * dt;
    theta_traj(traji) = theta_traj(traji-1) + dthetadt * dt;
end

pred{i-(start-1)}(c,1:5) = [x_traj(end), y_traj(end), theta_traj(end), z_traj(end), double(confirmedTracks(unicorns,1).TrackID)];
Inpred{i-(start-1)}(c,1:5) = [x_traj(1), y_traj(1), theta_traj(1), z_traj(1), double(confirmedTracks(unicorns,1).TrackID)];


 c=c+1;
 end
 end
 end
 unicorns=unicorns+1;
 end
 figure (123456789);
 %ADD THE LIVE POINT CLOUD HERE
 pcshow(currentLidar.Location, currentLidar.Intensity, 'MarkerSize', 30); % Display the point cloud colored by intensity
 title(['Lidar Frame: ' num2str(i)]);
 xlabel('X (m)');
 ylabel('Y (m)');
 zlabel('Z (m)');
 colormap('parula');
 colorbar; % Show a color bar to indicate intensity levels
 clim([0 40]);
 axis([-5 30 -7 7 -3 -1.5]); % Define axis limits based on your data range
 view(2); % Set view to top-down (XY plane)
 hold on; % Retain the point cloud plot for overlaying the line
 % Define and plot the line
 x = 1:.1:20; % X-values of the line
 y1= (lanePolys{i}(1,1).*x.^2+lanePolys{i}(1,2).*x+lanePolys{i}(1,3)); % Y-values for left lane
 y2= (lanePolys{i}(2,1).*x.^2+lanePolys{i}(2,2).*x+lanePolys{i}(2,3)); % Y-values for right lane
 z(1:length(x)) = -1.75; % Z-values (all zeros since it's in the XY plane)
 plot3(x, y1, z, 'r-', 'LineWidth', .1); % Plot the left lane in red with a width of .1
 plot3(x, y2, z, 'r-', 'LineWidth', .1); % Plot the right lane in red with a width of .1
 % Plot the left peaks
 if ~isempty(leftPeaks)
 scatter3(leftPeaks(:, 2), leftPeaks(:, 1), -1.75*ones(size(leftPeaks, 1), 1), 50, 'magenta', 'filled'); % Plot left lane points in magenta
 end
 % Plot the right peaks
 if ~isempty(rightPeaks)
 scatter3(rightPeaks(:, 2), rightPeaks(:, 1), -1.75*ones(size(rightPeaks, 1), 1), 50, 'cyan', 'filled'); % Plot right lane points in cyan
 end



 % Find points in rightPeaksOld that are not in rightPeaks
[~, idxRightNotInNew] = setdiff(rightPeaksOld, rightPeaks, 'rows');
rightPeaksRemoved = rightPeaksOld(idxRightNotInNew, :);

% Find points in leftPeaksOld that are not in leftPeaks
[~, idxLeftNotInNew] = setdiff(leftPeaksOld, leftPeaks, 'rows');
leftPeaksRemoved = leftPeaksOld(idxLeftNotInNew, :);
%rightPeaksRemoved=[1,1];

% Plot removed points in rightPeaksOld in red
scatter3(rightPeaksRemoved(:, 2), rightPeaksRemoved(:, 1), -1.75*ones(size(rightPeaksRemoved, 1), 1), 50, 'red', 'filled');

% Plot removed points in leftPeaksOld in red
scatter3(leftPeaksRemoved(:, 2), leftPeaksRemoved(:, 1), -1.75*ones(size(leftPeaksRemoved, 1), 1), 50, 'red', 'filled');

 


drawnow; % Update the figure with new data
 hold off; % Release the hold after plotting the line and peaks
 
 % 
 % x=1:.1:20;
 % y(1:191)=0;
 % %figure (123456789)
 % plot(x,y)
 
 % Update display
 if isvalid(displayObject.PointCloudProcessingDisplay.ObstaclePlotter)
 % Get current image scan for reference image
 currentImage = imageData{i};
 
 % Update display object
 displayObject(detections,confirmedTracks,currentLidar,obstacleIndices,...
 groundIndices,croppedIndices,currentImage,modelProbs);
 displayObject.lanePolynomials=lanePolys{i};
 end
 
 % Snap a figure at time = 18
 if abs(time - 18) < dT/2
 snapnow(displayObject);
 end
 
end
% Write movie if requested
if ~isempty(displayObject.MovieName)
 writeMovie(displayObject);
end
% Write new GIFs if requested.
if displayObject.RecordGIF
 % second input is start frame, third input is end frame and last input
 % is a character vector specifying the panel to record.
 writeAnimatedGIF(displayObject,10,170,'trackMaintenance','ego');
 writeAnimatedGIF(displayObject,310,330,'jpda','processing');
 writeAnimatedGIF(displayObject,120,140,'imm','details');
end
%%
% The figure above shows the three displays at time = 18 seconds. The
% tracks are represented by green bounding boxes. The bounding box
% detections are represented by orange bounding boxes. The detections also
% have orange points inside them, representing the point cloud segmented as
% obstacles. The segmented ground is shown in purple. The cropped or
% discarded point cloud is shown in blue.
%% Generate C Code
% You can generate C code from the MATLAB(R) code for the tracking and the
% preprocessing algorithm using MATLAB Coder(TM). C code generation enables
% you to accelerate MATLAB code for simulation. To generate C code, the
% algorithm must be restructured as a MATLAB function, which can be
% compiled into a MEX file or a shared library. For this purpose, the point
% cloud processing algorithm and the tracking algorithm is restructured
% into a MATLAB function, |mexLidarTracker|. Some variables are defined as
% |persistent| to preserve their state between multiple calls to the
% function (see <docid:matlab_ref#f76-920859 persistent>). The inputs and
% outputs of the function can be observed in the function description
% provided in the "Supporting Files" section at the end of this example.
%
% %%
% % MATLAB Coder requires specifying the properties of all the input
% % arguments. An easy way to do this is by defining the input properties by
% % example at the command line using the |-args| option. For more
% % information, see <docid:coder_ug#bq9_obs-1 Define Input Properties by
% % Example at the Command Line>. Note that the top-level input arguments
% % cannot be objects of the |handle| class. Therefore, the function accepts
% % the _x_, _y_ and _z_ locations of the point cloud as an input. From the
% % stored point cloud, this information can be extracted using the
% % |Location| property of the |pointCloud| object. This information is also
% % directly available as the raw data from the lidar sensor.
% 
% % Input lists
% inputExample = {lidarData{1}.Location, 0};
% 
% % Create configuration for MEX generation
% cfg = coder.config('mex');
% 
% % Replace cfg with the following to generate static library and perform
% % software-in-the-loop simulation. This requires an Embedded Coder license.
% %
% % cfg = coder.config('lib'); % Static library
% % cfg.VerificationMode = 'SIL'; % Software-in-the-loop
% 
% % Generate code if file does not exist.
% if ~exist('mexLidarTracker_mex','file')
% h = msgbox({'Generating code. This may take a few minutes...';'This message box will close when done.'},'Codegen Message');
% % -config allows specifying the codegen configuration
% % -o allows specifying the name of the output file
% codegen -config cfg -o mexLidarTracker_mex mexLidarTracker -args inputExample 
% close(h);
% else
% clear mexLidarTracker_mex;
% end
% %%
% % *Rerun simulation with MEX Code*
% %
% % Rerun the simulation using the generated MEX code, |mexLidarTracker_mex|.
% % Reset time
% time = 0;
% 
% for i = 1:numel(lidarData)
% time = time + dT;
% 
% currentLidar = lidarData{i};
% 
% [detectionsMex,obstacleIndicesMex,groundIndicesMex,croppedIndicesMex,...
% confirmedTracksMex, modelProbsMex] = mexLidarTracker_mex(currentLidar.Location,time);
% 
% % Record data for comparison with MATLAB execution.
% numTracks(i,2) = numel(confirmedTracksMex);
% end
% 
% %%
% % Compare results between MATLAB and MEX Execution
% disp(isequal(numTracks(:,1),numTracks(:,2)));
%%
% Notice that the number of confirmed tracks is the same for MATLAB and
% MEX code execution. This assures that the lidar preprocessing and
% tracking algorithm returns the same results with generated C code as with
% the MATLAB code.
%% Results
% Now, analyze different events in the scenario and understand how
% the combination of lidar measurement model, joint probabilistic data
% association, and interacting multiple model filter, helps achieve a good 
% estimation of the vehicle tracks.
%
%%
% *Track Maintenance*
%
% <<../trackMaintenance.gif>>
%
% The animation above shows the simulation between time = 3 seconds and
% time = 16 seconds. Notice that tracks such as T10 and T6 maintain their
% IDs and trajectory during the time span. However, track T9 is lost
% because the tracked vehicle was missed (not detected) for a long time by
% the sensor. Also, notice that the tracked objects are able to maintain
% their shape and kinematic center by positioning the detections onto the
% visible portions of the vehicles. For example, as Track T7 moves forward,
% bounding box detections start to fall on its visible rear portion and the
% track maintains the actual size of the vehicle. This illustrates the
% offset and shrinkage effect modeled in the measurement functions.
%%
% *Capturing Maneuvers*
%
% <<../imm.gif>>
%
% The animation shows that using an IMM filter helps the tracker to
% maintain tracks on maneuvering vehicles. Notice that the vehicle tracked
% by T4 changes lanes behind the ego vehicle. The tracker is able maintain
% a track on the vehicle during this maneuvering event. Also notice in the
% display that its probability of following the constant turn model,
% denoted by ct, increases during the lane change maneuver.
%%
% *Joint Probabilistic Data Association*
%
% <<../jpda.gif>>
%
% This animation shows that using a joint probabilistic data association
% tracker helps in maintaining tracks during ambiguous situations. Here,
% vehicles tracked by T43 and T73, have a low probability of detection due
% to their large distance from the sensor. Notice that the tracker is able
% to maintain tracks during events when one of the vehicles is not
% detected. During the event, the tracks first coalesce, which is a known
% phenomenon in JPDA, and then separate as soon as the vehicle was detected
% again.
%% Summary
% This example showed how to use a JPDA tracker with an IMM filter to track
% objects using a lidar sensor. You learned how a raw point cloud can be
% preprocessed to generate detections for conventional trackers, which
% assume one detection per object per sensor scan. You also learned
% how to define a cuboid model to describe the kinematics, dimensions, and
% measurements of extended objects being tracked by the JPDA tracker. In
% addition, you generated C code from the algorithm and verified its
% execution results with the MATLAB simulation.
%% Supporting Files
% This section highlights the code from some important supporting files
% used in this example. The complete list of supporting files can be found
% in the current working directory after opening the example in MATLAB.
% *helperLidarModel*
%
% This function defines the lidar model to simulate shrinkage of the
% bounding box measurement and center-point offset. This function is used
% in the |helperCvmeasCuboid| and |helperCtmeasCuboid| functions to obtain 
% bounding box measurement from the state.
%
% <include>helperLidarModel.m</include>
%
%%
% *helperInverseLidarModel*
%
% This function defines the inverse lidar model to initiate a tracking
% filter using a lidar bounding box measurement. This function is used in
% the |helperInitIMMFilter| function to obtain state estimates from a
% bounding box measurement.
%
% <include>helperInverseLidarModel.m</include>
%
%%
% *HelperBoundingBoxDetector*
%
% This is the supporting class |HelperBoundingBoxDetector| to accept a
% point cloud input and return a list of |objectDetection|
%
% <include>HelperBoundingBoxDetector.m</include>
%
%%
% *mexLidarTracker*
%
% This function implements the point cloud preprocessing display and the
% tracking algorithm using a functional interface for code generation.
%
% <include>mexLidarTracker.m</include>
%
%%
% *helperCalcDetectability*
%
% The function calculates the probability of detection for each track. This
% function is used to generate the "DetectableTracksIDs" input for the
% |trackerJPDA|.
%
% <include>helperCalcDetectability.m</include>
%
%%
% *loadLidarAndImageData*
%
% Stitches Lidar and Camera data for processing using initial and final
% time specified.
% function [lidarData,imageData] = loadLidarAndImageData(datasetFolder,initTime,finalTime)
% initFrame = max(1,floor(initTime*10));
% lastFrame = min(350,ceil(finalTime*10));
% load (fullfile(datasetFolder,'imageData_35seconds.mat'),'allImageData');
% imageData = allImageData(initFrame:lastFrame);
% 
% numFrames = lastFrame - initFrame + 1;
% lidarData = cell(numFrames,1);
% 
% % Each file contains 70 frames.
% initFileIndex = floor(initFrame/70) + 1;
% lastFileIndex = ceil(lastFrame/70);
% 
% frameIndices = [1:70:numFrames numFrames + 1];
% 
% counter = 1;
% for i = initFileIndex:lastFileIndex
% startFrame = frameIndices(counter);
% endFrame = frameIndices(counter + 1) - 1;
% load(fullfile(datasetFolder,['lidarData_',num2str(i)]),'currentLidarData');
% lidarData(startFrame:endFrame) = currentLidarData(1:(endFrame + 1 - startFrame));
% counter = counter + 1;
% end
% end
%% References
% [1] Arya Senna Abdul Rachman, Arya. "3D-LIDAR Multi Object Tracking for
% Autonomous Driving: Multi-target Detection and Tracking under Urban Road
% Uncertainties." (2017).
