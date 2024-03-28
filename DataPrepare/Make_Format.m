% ------------------------------------------------------------------------
% Generate proper data format for GNSS/INS integration
%
% by ZD LI
% zhengdao.li@connect.polyu.hk
% ------------------------------------------------------------------------


clc
clearvars
close all
D2R = pi/180;
R2D = 180/pi;


%% CONFIGURATION
% "EnvFolder" is the folder storing the GNSS and INS dataset for a certain receiver
EnvFolder = 'THE_ABSOLUTE_PATH_OF_EnvFolder_IN_YOUR_LOCAL_DEVICE/';

load([EnvFolder,'MAT_FILE_OF_EXTRACTED_GNSS_FEATURES.mat']);

AvaEpochInd = Refined_data_normalized(:,1);
Stime = 457695;   % (user-defined) The start time of all receivers, in order to make comparisons
Etime = 923;      % (user-defined) Evaluation time

% CNN prediction for GNSS/INS integration
load([EnvFolder,'MAT_FILE_OF_CNN_PREDICTION_FOR_EACH_AVAILABLE_GNSS_EPOCH','.mat']);


%% LOAD GT DATA
CPT_data0 = readmatrix([EnvFolder,['TXT_FILE_OF_GROUND_TRUTH_INFORMATION','.txt']]);
for idt = 2:size(CPT_data0,1)   
    CPT_Data(idt-1,1) = CPT_data0(idt,3);
    CPT_Data(idt-1,3) = CPT_data0(idt,3);        %GPST
    CPT_Data(idt-1,4) = CPT_data0(idt,4) + CPT_data0(idt,5)/60 + CPT_data0(idt,6)/3600;
    CPT_Data(idt-1,5) = CPT_data0(idt,7) + CPT_data0(idt,8)/60 + CPT_data0(idt,9)/3600;
    CPT_Data(idt-1,6) = CPT_data0(idt,10);
    
    CPT_Data(idt-1,12) =  CPT_data0(idt,19) ;   % yaw 
    CPT_Data(idt-1,7) = CPT_data0(idt,12)   ;   % north velocity
    CPT_Data(idt-1,8) = CPT_data0(idt,11)   ;   % east velocity
    CPT_Data(idt-1,9) = -CPT_data0(idt,13)  ;   % down velocity  
end  

CPT_Data(:,20) = CPT_Data(:,3); % document CPT TIME
disp('===> CPT data loading complete <===');


%% LOAD IMU DATA
IMUFileName = ['/CSV_FILE_OF_INS_DATA','.csv'];
imu_data = IMU_dataConverter([EnvFolder , IMUFileName]);
% ---> Example Raw File:
% col 3: utc time
% col 5-8 : orientation (axis: x,y,z,w) 
% col 18-20 : angular velocity (axis: x, y, z)
% col 30-32 : linear acceleration (axis: x, y, z)
% ---> Example converted IMU_Data:
% col (1): UTC_time
% col (4-6): linear acceleration (axis: x, y, z)
% col (7-9): angular velocity (axis: x, y, z)
% col (10-13): orientation (axis: x,y,z,w)

IMU_Data = [imu_data(:,1), zeros(size(imu_data,1),2), imu_data(:,2:11)];  
IMU_Data(:,20) = IMU_Data(:,1); % document IMU GPST

All_Orient = IMU_Data(:,10:13);
disp('===> IMU data loading complete <===');


%% LOAD GNSS DATA
GNSSFileName = ['/MAT_FILE_OF_GNSS_DATA','.mat'];
load([EnvFolder, GNSSFileName], 'GNSS_data');
GNSS_Data = GNSS_data;
GNSS_Data(:,20) = GNSS_Data(:,1); % document GNSS GPST 
disp('===> GNSS data loading complete <===');


%% SYNCHRONIZATION
% ---- synchronization begins ----
% starting time
[~,temp.CPT_ids] = min(abs(CPT_Data(:,1)-Stime));
CPT_Data(1:(temp.CPT_ids-1),:)=[];
[~,temp.GNSS_ids] = min(abs([GNSS_Data{:,1}]-Stime));
GNSS_Data(1:(temp.GNSS_ids-1),:)=[];
% GNSS_Data(1:(temp.GNSS_ids),:)=[];
[~,temp.IMU_ids] = min(abs(IMU_Data(:,1)-Stime));
IMU_Data(1:(temp.IMU_ids-1),:)=[];

% index for CPT,IMU, GNSS starting from 1
CPT_Data(:,1) = CPT_Data(:,1)-Stime+1;
IMU_Data(:,1) = IMU_Data(:,1)-Stime+1;
for idt = 1:size(GNSS_Data,1)
    GNSS_Data{idt,12} = GNSS_Data{idt,1}-Stime+1;
end

% consider the evaluation period only
CPT_Data(CPT_Data(:,1)>Etime,:) = [];
IMU_Data(IMU_Data(:,1)>Etime,:) = [];
GNSS_Data([GNSS_Data{:,12}]'>Etime,:) = [];

% data synchronization
temp.GT_t0 = CPT_Data(2,1);
temp.GT_t = CPT_Data(2:end,1);
temp.GNSS_t = [GNSS_Data{:,12}]';
temp.IMU_t = IMU_Data(:,1);
[~,temp.GNSS_ids] = min(abs(temp.GNSS_t-temp.GT_t0));
[~,temp.IMU_ids] = min(abs(temp.IMU_t-temp.GT_t0));

% minor adjustment
minor_imu1 = IMU_Data(temp.IMU_ids);
minor_imu2 = IMU_Data(temp.IMU_ids + 1);
minor_round = ceil(IMU_Data(temp.IMU_ids));
if (minor_round > minor_imu1) && (minor_round < minor_imu2)
    temp.IMU_ids = temp.IMU_ids + 1;
end

CPT_Data(1,:) = [];
IMU_Data(1:(temp.IMU_ids-1),:)=[];
GNSS_Data(1:(temp.GNSS_ids-1),:)=[];
disp('===> Data synchronization complete <===');

% filter out points whose absolute altitude over 2000m %
num_gnss = size(GNSS_Data,1);
this_llh = zeros(num_gnss,3);
extreme_ind=[];
for i = 1: size(GNSS_Data,1)
    var_clock_error_est(i,:) = cell2mat(GNSS_Data(i, 13));
    this_dop(i,:) = cell2mat(GNSS_Data(i,18));
    if abs(this_dop(i,1)) >= 100 && abs(this_dop(i,2)) >= 100
        extreme_ind = [extreme_ind; i];
    end
end
if ~isempty(extreme_ind)
    extreme_gnssT = GNSS_Data{extreme_ind,1};
    GNSS_Data(extreme_ind,:)=[];
end
% ---- synchronization ends -----


%% MAKE SENSORDATA
lla0 = GNSS_data{find(cell2mat(GNSS_data(:,1))==(Stime+1)),2}([2,1,3]); 
                % the local origin to convert LLH to ENU

for idt = 1: size(GNSS_Data,1)
    Cur_GNSS_Epoch = cell2mat(GNSS_Data(idt, 20));
    corres_ids_in_imu = find(IMU_Data(:,20)>Cur_GNSS_Epoch & IMU_Data(:,20)<(Cur_GNSS_Epoch+1));
    GNSS_Data{idt,19} = corres_ids_in_imu;
end

% Filter out GNSS epochs not in imu data
EmptyToDelete = [];
for i =1:size(GNSS_Data,1)
    if size(cell2mat(GNSS_Data(i,19)),1)==0 % disgard null value epochs
        EmptyToDelete = [EmptyToDelete,i];
    end
end
GNSS_Data(EmptyToDelete,:) = [];

% get all the valid gnss epochs after feature extraction
valid_ts_in_pca = AvaEpochInd; 
% prediction -> GNSS_Data %
[~,temp.valid_pcaLoc_after_gnss] = ismember(valid_ts_in_pca,cell2mat(GNSS_Data(:,20)));

% Final predicted labels and the GPST %
labels_pred = labels_pred((temp.valid_pcaLoc_after_gnss~=0),:); % prediction
labels_test = labels_test';
labels_test = labels_test((temp.valid_pcaLoc_after_gnss~=0),:); % truth 2d error
valid_ts_in_pca = valid_ts_in_pca((temp.valid_pcaLoc_after_gnss~=0),:);
ml_prediction = double([valid_ts_in_pca, labels_pred, labels_test]);

% CPT_Data -> IMU data& GNSS data %
End_epoch = min(floor(IMU_Data(end,20)), GNSS_Data{end,1});
CPT_Data(find(CPT_Data(:,3)==End_epoch) +1: end, :) = []; 

disp('===> Valid epochs after feature selection complete <===');

% Number of gnss epochs theoretically
num_gnss_pseu_epoch = GNSS_Data{end,1} - GNSS_Data{1,1} + 1;

for idt = 1: 1: num_gnss_pseu_epoch
    sensorData {1,idt}. GyroReadings = IMU_Data (floor(IMU_Data(:,20))==(Stime+idt), 11:13);
    sensorData {1,idt}. AccelReadings = IMU_Data (floor(IMU_Data(:,20))==(Stime+idt), 8:10);
    sensorData {1,idt}. numIMUReadings = 400; %length(GNSS_Data{idt,19});
    numIMUSamplesPerGPS (idt,1) = 400; %length(GNSS_Data{idt,19});
    sensorData {1,idt}. GnssT = Stime+idt;
    GNSS_raw_index = find(cell2mat(GNSS_Data(:,1))==(Stime+idt));
    % not valid GNSS epoch after feature extraction
    if ~isempty(extreme_ind) && ismember((Stime+idt), extreme_gnssT) 
        sensorData {1,idt}. GNSSReading = nan;
    elseif ~isempty(GNSS_raw_index)
        % has GNSS ls solution
        sensorData {1,idt}. GNSSReading = GNSS_Data{GNSS_raw_index, 2}([2,1,3]);
    else
        % no GNSS ls solution; reset the index so that recovered in next iteration 
        sensorData {1,idt}. GNSSReading = nan;
    end   
end

disp('===> sensorData and ML prediction data set up <===');


% AHRS regulation
temp_imu_eul = zeros(size(IMU_Data,1), 4);
for idt = 1:size(IMU_Data,1)
    if mod(idt, 2e3)==0
        disp(['AHRS Processing==> ',num2str(idt),'/',num2str(size(IMU_Data,1))]);
    end
    temp_imu_eul(idt,1) = IMU_Data(idt,1);
    temp_imu_eul(idt,2:4) = quat2eul(IMU_Data(idt,10:13)).*[R2D,R2D,R2D];
    %yaw
    temp_imu_eul(idt,4) = -temp_imu_eul(idt,4);
    if temp_imu_eul(idt,4) < 0
        temp_imu_eul(idt,4) = temp_imu_eul(idt,4)+360;
    end
    %roll
    if temp_imu_eul(idt,2)<0
        temp_imu_eul(idt,2) = temp_imu_eul(idt,2)+360;
    end
    temp_imu_eul(idt,2) = -temp_imu_eul(idt,2)+180;
    %pitch
    temp_imu_eul(idt,3) = -temp_imu_eul(idt,3);
end
IMU_Data(:,14:16) = temp_imu_eul(:,[3,2,4]); %(pitch; roll; yaw)
disp('===> AHRS set up <===');



%% OTHER PARAMETERS
% Initial position
initPos = [0,0,0]; 
% IMU freq
imuFs = 400; 
% Freq ratio
numGPSSamplesPerOptim = 2;
% imuNoise
%   Gyro noise PSD (deg^2 per hour, converted to rad^2/s)  
imuNoise.GyroscopeNoise = 3.158273408348594e-04*eye(3); %0.005^2 * eye(3);
%   Accelerometer noise PSD (micro-g^2 per Hz, converted to m^2 s^-3)                
imuNoise.AccelerometerNoise = 1.23182208e-04*eye(3); %0.008^2 * eye(3);
%   Accelerometer bias random walk PSD (m^2 s^-5)
imuNoise.AccelerometerBiasNoise = diag([9.62361e-09  9.62361e-09 2.16531225e-08]); 
%   Gyro bias random walk PSD (rad^2 s^-3)
imuNoise.GyroscopeBiasNoise = 6.633890475354988e-09*eye(3)  ;% 4.0E-11 * eye(3);
% Initial orient
initOrient = quaternion(All_Orient(1,4),All_Orient(1,1),All_Orient(1,2),All_Orient(1,3));
% Ground Truth 
posLLH = CPT_Data(:,4:6);


% Parameters for KF
D2R = pi/180;
R2D = 180/pi;
deg_to_rad = 0.01745329252;
rad_to_deg = 1 / deg_to_rad;
micro_g_to_meters_per_second_squared = 9.80665E-6;
% Initial attitude uncertainty per axis (deg, converted to rad)
LC_KF_config.init_att_unc = deg2rad(2);
% Initial velocity uncertainty per axis (m/s)
LC_KF_config.init_vel_unc = 0.1;
% Initial position uncertainty per axis (m)
LC_KF_config.init_pos_unc = 10;
% Initial accelerometer bias uncertainty per instrument (micro-g, converted to m/s^2)
LC_KF_config.init_b_a_unc = 10000 * micro_g_to_meters_per_second_squared;
% Initial gyro bias uncertainty per instrument (deg/hour, converted to rad/sec)
LC_KF_config.init_b_g_unc = 200 * deg_to_rad / 3600;

% Gyro noise PSD (deg^2 per hour, converted to rad^2/s)                
LC_KF_config.gyro_noise_PSD = 0.005^2; 
% Gyro bias random walk PSD (rad^2 s^-3)
LC_KF_config.gyro_bias_PSD = 4.0E-11;
% Accelerometer noise PSD (micro-g^2 per Hz, converted to m^2 s^-3)
LC_KF_config.accel_noise_PSD = 0.008^2;
% Accelerometer bias random walk PSD (m^2 s^-5)
LC_KF_config.accel_bias_PSD = 1E-5; 

% Position measurement noise SD per axis (m)
LC_KF_config.pos_meas_SD = 25;
% Velocity measurement noise SD per axis (m/s)
LC_KF_config.vel_meas_SD = 10;
disp('===> Other parameters set up <===');



%% SAVE FILES
save ([EnvFolder,'IniResults/PROCESSED_DATA_FOR_FACTOR_GRAPH_OPTIMIZATION.mat'],'sensorData','imuFs','initPos',...
    'initOrient','imuNoise','LC_KF_config','numGPSSamplesPerOptim','numIMUSamplesPerGPS', ...
    'posLLH');

save ([EnvFolder,'IniResults/PROCESSED_DATA_FOR_KALMAN_FILTER.mat'],...
    'D2R','R2D','deg_to_rad','LC_KF_config','micro_g_to_meters_per_second_squared',...
    'rad_to_deg');

save ([EnvFolder,'IniResults/COMMOM_INFORMATION_FOR_BOTH_INTEGRATION_METHODS.mat'],'EnvFolder','GNSS_Data','GNSS_data','IMU_Data','CPT_Data','lla0','ml_prediction');

save ('MAT_FILE_STOREING_STRING_OF_PATH_OF_EnvFolder.mat','EnvFolder');

disp('===> All files saved <===');





