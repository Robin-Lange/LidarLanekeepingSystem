clearvars -except lidarData nearIr
close all
clc

pcidx = 1;

while pcidx <= length(nearIr)

    figure(2000)
    imshow(nearIr{1, pcidx} * 6.5)
    title(['Frame: ', num2str(pcidx)]) % Dynamically update the title with the current frame index
    pause(.05)

    pcidx = pcidx + 1;
end
