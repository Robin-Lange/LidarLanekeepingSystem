# LiDARLaneDetection
Installing pip packages: `pip3 install -r requirements.txt`
# Update: Large File system is not used anymore

## Papers
- https://ieeexplore.ieee.org/document/9827157
- https://data.uni-hannover.de/cs_CZ/dataset/lumpi
- https://www.thi.de/forschung/carissma/aktuelles-presse/einzelansicht/news/kuenstliche-intelligenz-soll-kreuzungen-in-ingolstadt-sicherer-machen/
- https://mediatum.ub.tum.de/doc/1145103/file.pdf
- https://www.fraunhofer.de/de/presse/presseinformationen/2022/februar-2022/kuenstliche-intelligenz-steuert-ampelanlagen.html
- https://www.diva-portal.org/smash/get/diva2:907048/FULLTEXT01.pdf

## Matlab Sites
- https://www.mathworks.com/help/lidar/ug/lane-detection-in-3d-lidar-point-cloud.html
- https://www.mathworks.com/help/driving/ug/lane-detection-in-lidar-data-using-deep-learning.html
- https://www.mathworks.com/help/driving/ug/get-started-with-lidar-lane-detection-using-deep-learning.html
- [detect function](https://www.mathworks.com/help/driving/ref/lidarlanedetector.detect.html)
- https://www.mathworks.com/help/driving/ref/trainlidarlanedetector.html

## Python Behavior Trees
- [Behavior Tree Planner](https://autowarefoundation.github.io/autoware.universe/pr-2609/planning/behavior_path_planner/)
- [Py-trees](https://py-trees.readthedocs.io/en/devel/)
-[UML visualizer](https://www.planttext.com/)

## Instructions to use vehicle detection
part 1: Formatting the data (skip this if you already have the "storage_######.mat" and "nearIr_######.mat" files.
   Open "dataFormatting" file and adjust the ousterFileReader line to be the desired files.
   At the bottom of dataFormatting, adjust it to save the nearIr and storage variables to the appropriate file name

Part 2: running the vehicle tracking.
   Run vehicleTrackingDistance file. If you want to use data from a specific scene, clear all variables then load that scene data and rerun the code.

   More instructions are given at the top of vehicleTrackingDistance.m file

## Instructions and Commands to connect the Ouster LiDAR with the Nvidia Jetson
Install the Ouster Sensor SDK on the Jetson. This URL has the instructions and link to the download page: https://static.ouster.dev/sdk-docs/index.html
Connect the LiDAR to the Jetson via LAN, then start up the Jetson. Go to Settings>Network and then click on the cogwheel in the line of the wires connection. Then check both the IPv4 and IPv6 tabs and set the method to Link-Local Only leave the rest as it is.
Then open a new terminal window with Ctrl+Alt+T. (Optional: Write ip addr show and hit enter then look at 3: eth0: and you will find the IP Address after inet)
To find the hostname, you can use the following methods:
Scan the QR Code on top of the lidar which should give you 122313000402.
Use ouster-cli discover: This will give you all the information exactly as you need it and will also check if it is properly connected. The contained number in the hostname should be identical to the one from the QR Code hostname: os-122313000402.local.
Then use the command ouster-cli source os-122313000402.local. viz with the correct number and the software should start so that you can see the live point of the lidar.

## How to find papers in other languages / from other countries
Search anything in Google, then click on `Tools` which should be right to the line with "All, Images, ..." In the new menu click on advanced search and then scroll down to the region and language filter.
In Google Scholar settings, you can filter the language as well.

## Setup Github with the Git Large File Storage
1. Install the most recent version of git from [here](https://git-scm.com/download/win)
2. Install the Git Large File Storage from [here](https://git-lfs.com/)
3. Those two steps should not require IT support!
4. Create a `sandbox` on your local device. A sandbox is an empty folder in a convenient space where you will locally access GitHub.
5. Navigate to this folder, right click and choose `Open Git Bash here`. The Git command window should pop up.
6. In it type: `git lfs install` and hit enter. The console should display the message `Git LFS initialized`.
7. Now clone the repository from GitHub in MATLAB
   a. Open GitHub and access the repository.
   b. Click on the top right, green icon `Code`. It opens a submenu, copy the link at the `https` window.
   c. Close and reopen MATLAB and navigate to your Sandbox (=your folder for the project). Right-click in the folder and click on `Source Control` and then `Manage Files`.
   d. Now a pop-up window should open. The `source control integration` field has to show `Git`, and the `Repository Path` is the field where you copy the URL from Github. Doublecheck the sandbox and click `Retrieve`.
   e. The cloning process is finished now.
8. Now you need to track the large file you have. You can only track certain file types, not individual files. In our case, the big files are .mat and .pcap. For both of them, you need to type: `git lfs track "*.mat"` and hit `enter` and the same with `git lfs track "*.pcap"`.
9. This finishes the LFS system setup and you should be good to clone the repository.

## Working with GitHub
### Education License
Access [this webpage](https://github.com/education) to retrieve your education license for Github. The free one also works for now but better safe than sorry.
### Rules
`Pull - (Add) - Commit - Push`

- When starting your work, right-click the folder `SourceControl>Pull`. This ensures that you work with the most recent version.
- When you have new files you can add them individually by right-clicking the file `Sourcecontrol>AddToGit` to add all of them at once, opening GitBash in that folder (the terminal should show some kind of longer path), and entering `git add .`. This adds all newly created files. WARNING⚠ Adding the .gitattributes file for the LFS system causes issues for new setups. Note: for now we try not to use this file.
- When you finish your work either right-click the folder and `Sourcecontrol>ViewAndCommitChanges` or again use GitBash in the folder and type `git commit -m"<your message>"` and hit enter.
- Then click `SourceControl>Push` or use the GitBash and type `git push`.
- While working on one branch communicate on who changes which files. NEVER change the same file in two different ways this will cause a MERGE ERROR.
- This can be fixed by working with multiple branches, but the cooperative part gets lost.

## Possible Problems
Sometimes maybe two people change a file at the same time. This could lead to pushing errors and one branch would have to different states. To fix this in the future when the project is set up finally, one can create a main branch and one for each person to push to so that the main branch only includes functional code and documentation.
General [information for setting up](https://www.mathworks.com/help/matlab/matlab_prog/set-up-git-source-control.html)
