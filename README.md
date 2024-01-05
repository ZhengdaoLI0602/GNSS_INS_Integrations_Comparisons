# Description
Based on the article titled [Intelligent Environment-Adaptive GNSS/INS Integrated Positioning with Factor Graph Optimization](https://www.mdpi.com/2072-4292/16/1/181) on [Remote Sensing](https://www.mdpi.com/journal/remotesensing) Journal, this repository is aimed for comparing 2D positioning solutions between different GNSS/INS integration methods, including Fixed-gain Kalman Filter (KF), Adaptive Kalmam Filter (AKF), Factor Graph Optimization (FGO), and Adaptive Factor Graph Optimization (AFGO). The scripts about FGO are modified based on MATLAB official example [Factor Graph-Based Pedestrian Localization with IMU and GPS Sensors](https://uk.mathworks.com/help/nav/ug/factor-graph-based-pedestrian-localization-imu-gps.html).

# How to run problem
## STEP 1: Navigate to folder "DataPrepare", run "Make_Format.m" inside folder "DataPrepare"
Before running all the scripts, add folder "Functions" as subfolders in MATLAB.
"EnvFolder" is the folder storing all the prepared datasets and the processed results. It is designed to contain:
- An empty folder named "IniResults" storing the intermediate and ultimate results
- MAT_FILE_OF_EXTRACTED_GNSS_FEATURES.mat
- MAT_FILE_OF_CNN_PREDICTION_FOR_EACH_AVAILABLE_GNSS_EPOCH.mat
- TXT_FILE_OF_GROUND_TRUTH_INFORMATION.txt
- CSV_FILE_OF_INS_DATA.csv
- MAT_FILE_OF_GNSS_DATA.mat

After running the codes, the following files are expected to be generated in folder "EnvFolder/IniResults" 
- PROCESSED_DATA_FOR_KALMAN_FILTER.mat
- PROCESSED_DATA_FOR_FACTOR_GRAPH_OPTIMIZATION.mat
- COMMOM_INFORMATION_FOR_BOTH_INTEGRATION_METHODS.mat

And the following files are expected to be generated in folder "DataPrepare" 
- MAT_FILE_STOREING_STRING_OF_PATH_OF_EnvFolder.mat

## STEP 2: Navigate to root folder, run "Main_GNSS_INS_Integration.m"
The variable "CurFolder" in the script exactly represents folder "EnvFolder/IniResults"

Configure the variables "KF_ON" and "FGO_ON" to execute the following four integration methods in sequence:
- KF_ON = [1 0], FGO_ON = [0 0]; run the script (KF)
- KF_ON = [0 1], FGO_ON = [0 0]; run the script (AKF)
- KF_ON = [0 0], FGO_ON = [1 0]; run the script (FGO)
- KF_ON = [0 0], FGO_ON = [0 1]; run the script (AFGO)

After running the script, the following files are expected to be generated in folder "EnvFolder/IniResults" 
- POSITIONING_SOLUTIONS_OF_ALL_INTEGRATION_METHODS.mat

And the following matlab figures are expected to be generated after in folder "EnvFolder/IniResults"
- After running KF
  - Traj_KF.fig (2D trajectory of Truth/GNSS/KF)
  - 2dE.fig (2D error of Truth/GNSS/KF throughout the time frame)
- After running AKF
  - Traj_AKF.fig (2D trajectory of Truth/GNSS/KF/AKF)
  - 2dE.fig (2D error of Truth/GNSS/KF/AKF throughout the time frame)
- After running FGO
  - Traj_FGO.fig (2D trajectory of Truth/GNSS/KF/AKF/FGO)
  - 2dE.fig (2D error of Truth/GNSS/KF/AKF/FGO throughout the time frame)
  - FGO_satelite_img.fig (2d trajectory of Truth/FGO shown with satellite image)
- After running AFGO
  - Traj_AFGO.fig (2D trajectory of Truth/GNSS/KF/AKF/FGO/AFGO)
  - 2dE.fig (2D error of Truth/GNSS/KF/AKF/FGO/AFGO throughout the time frame)
  - AFGO_satelite_img.fig (2d trajectory of Truth/AFGO shown with satellite image)

# Example
An example of u-blox receiver dataset collected from [Kowloon Bay, Hong Kong](https://www.google.com/maps/place/%E4%B9%9D%E9%BE%99%E6%B9%BE/@22.3207264,114.2052806,16z/data=!4m6!3m5!1s0x34040139ce5cb28d:0xebb076fb9d3032f4!8m2!3d22.3080749!4d114.2018982!16zL20vMDIyeng3?entry=ttu) has been provided in the repository. The collection was done by group in [Intelligent Positioning And Navigation Laboratory](https://github.com/IPNL-POLYU) using similar standards with [UrbanNav dataset](https://github.com/IPNL-POLYU/UrbanNavDataset). The target outcomes are depicted in the following plots and stored in folder "Target Plots".
- Summary
  ||KF|AKF|FGO|AFGO|
  |---|---|---|---|---|
  |2D Root Mean Square Error [m]|70.18|66.84|24.35|19.35|
  |2D Standard Deviation [m]|61.72|59.19|18.71|14.09|
- 2D Trajectory
  ![ublox_traj](https://github.com/ZhengdaoLI0602/GNSS_INS_Integrations_Comparisons/assets/80500317/ba8c2d80-bd2f-40f7-ac64-141f05f35818)
- 2D Error throughout the frame
  ![ublox_2dE](https://github.com/ZhengdaoLI0602/GNSS_INS_Integrations_Comparisons/assets/80500317/0daa70d9-9e24-4ee8-b670-b51fd9794b71)




