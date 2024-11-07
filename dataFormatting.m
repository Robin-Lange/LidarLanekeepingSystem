clear
close all
clc
tic
%%ousterReader = ousterFileReader("OS-0-128_122313000402_1024x10_20240903_150017.pcap","OS-0-128_122313000402_1024x10_20240903_150017.json");
%%ousterReader = ousterFileReader("OS-0-128_122313000402_1024x10_20240903_145833.pcap","OS-0-128_122313000402_1024x10_20240903_145833.json");

%ousterReader = ousterFileReader("OS-0-128_122313000402_1024x10_20240909_130040.pcap","OS-0-128_122313000402_1024x10_20240909_130040.json");
%ousterReader = ousterFileReader("OS-0-128_122313000402_1024x10_20240909_130616.pcap","OS-0-128_122313000402_1024x10_20240909_130616.json");
ousterReader = ousterFileReader("OS-0-128_122313000402_1024x10_20240909_130616.pcap","OS-0-128_122313000402_1024x10_20240909_130616.json");


xlimits = [-6000 6000];  % to get cropped/uncropped ptcloud saved, adjust these
ylimits = [-6000 6000];  % and modify how the variable is saved at the end of 
zlimits = [-400 1500];   % this code

rotationAngles = [0 -13 0];
translation = [0 0 0];
tform = rigidtform3d(rotationAngles,translation);

%player = pcplayer(xlimits,ylimits,zlimits);

%xlabel(player.Axes,"X (m)");
%ylabel(player.Axes,"Y (m)");
%zlabel(player.Axes,"Z (m)");

ousterReader.CurrentTime = ousterReader.StartTime + seconds(0.3);

i = 1;
while(hasFrame(ousterReader) )%&& player.isOpen())
    [ptCloud, attributes] = readFrame(ousterReader);
    ptCloud = pctransform(ptCloud,tform);

    %view(player,ptCloud(1,1));
    storage{i} = ptCloud;
    nearIr{i} = attributes.NearInfrared;
    i = i+1;
end


% save("storage_131656.mat", 'storage', '-v7.3');
% 
save("nearIr_130616.mat", 'nearIr', '-v7.3');
toc