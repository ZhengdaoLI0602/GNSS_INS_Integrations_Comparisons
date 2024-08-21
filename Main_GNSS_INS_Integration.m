% --------------------------------------------------------------------------
% Main function for comparing GNSS/INS integration methods (KF/AKF/FGO/AFGO)
%
% by ZD LI
% zhengdao.li@connect.polyu.hk
% --------------------------------------------------------------------------


clc
clearvars
close all


%% Initialization
load('./DataPrepare/MAT_FILE_STOREING_STRING_OF_PATH_OF_EnvFolder.mat'); % Load the UrbanNav data folder
CurFolder = [EnvFolder,'IniResults']; 
load([CurFolder,'/COMMOM_INFORMATION_FOR_BOTH_INTEGRATION_METHODS.mat']); % Load the common information
if isfile(([CurFolder,'/POSITIONING_SOLUTIONS_OF_ALL_INTEGRATION_METHODS.mat']))
    load([CurFolder,'/POSITIONING_SOLUTIONS_OF_ALL_INTEGRATION_METHODS.mat']);
end

% Switching constants
% Step 1: KF_ON = [1 0], FGO_ON = [0 0]; run FKF
% Step 2: KF_ON = [0 1], FGO_ON = [0 0]; run AKF
% Step 3: KF_ON = [0 0], FGO_ON = [1 0]; run FGO
% Step 4: KF_ON = [0 0], FGO_ON = [0 1]; run AFGO
KF_ON = [0 0];  %[KF, AKF]
FGO_ON = [1 0]; %[FGO, AFGO] 

% (user-defined) KF and FGO uncertainty parameters
Fixed_KF_Uct = 30;
receiver = 'ublox';
min_uct = 1.5;
max_uct = 65; 


% (user-defined) Tuning factor on solution uncertainty for MATLAB FGO
tuning_factor_FGO = 0.2;

% FGO uncertainty parameters
Fixed_FGO_Uct = Fixed_KF_Uct * tuning_factor_FGO;

% (user-defined)Period for plotting and error calculation in both KF and FGO %
mannual_set_period = 920; 

 

%% CONFIGURATION 
disp('===> Data loading Complete <===');

num_CPT = size(CPT_Data,1);
GNSS_Ref = zeros(num_CPT, 4);
GT_Ref = [];

for i = 1: num_CPT
    this_llh = cell2mat(GNSS_Data(find(cell2mat(GNSS_Data(:,1)) == CPT_Data(i,3)), 2));
    if ~isempty(this_llh)
        GNSS_Ref(i,[2,1,3]) = this_llh;
    else
        GNSS_Ref(i,[2,1,3]) = [nan, nan, nan];
    end
    GT_Ref(i,:) = CPT_Data(i,4:6); 
end

% Add GPST labels %
GNSS_Ref(:, 4) = CPT_Data(:, 3);
GT_Ref(:, 4) = CPT_Data(:, 3); 

% Get ENU solution for GNSS % 
for i = 1: size(GNSS_Ref,1)
    Solu.GNSS_enu (i,1) = CPT_Data(i,1);
    Solu.GNSS_enu (i,2:4) =  lla2enu(GNSS_Ref(i,1:3),lla0,"ellipsoid");
end

% All error components
Error_2D_enu = zeros(num_CPT, 4);
Error_2D_xyz = zeros(num_CPT, 3);
Error_2D_enu_com = zeros(num_CPT, 10);
Error_2D_xyz_com = zeros(num_CPT, 7);


%% LCKF Method
if ismember(1, KF_ON)
    load([CurFolder,'/PROCESSED_DATA_FOR_KALMAN_FILTER.mat']);

    if isequal(KF_ON, [0 1])
        disp('Loosely-coupled AKF starts...');
    else
        disp('Loosely-coupled FKF starts...');
    end

    % KF timer starts... %
    tic  

    IMU_shift = floor(IMU_Data(1,1)) - 1;
    if IMU_shift > 0 
        IMU_Data(:,1) = IMU_Data(:,1) - IMU_shift;
    end
    
    % default adaptive mode
    whether_adaptive = 1;
    if isequal(KF_ON,[1 0])
        % set fixed uct mode %
          whether_adaptive = 0;
    end

    [SVs, LCKF_result, GNSS_llh, IMU_Data, Ucts] = LCKF_INS_GNSS_debug(GNSS_Data,IMU_Data,LC_KF_config, Fixed_KF_Uct, ml_prediction, whether_adaptive, receiver); 

    ProsTime(1) = toc; % document the processing time (@KF) 
    
    id_LCKF = zeros(num_CPT - 1, 1);  
    CPT_xyz = zeros(num_CPT, 3);
    CPT_enu = zeros(num_CPT, 3);

    % KF Error Evaluation
    for idt = 1:1:num_CPT
        CPT_xyz(idt,:) = llh2xyz(CPT_Data(idt,4:6).*[D2R,D2R,1]);
        CPT_enu(idt,:) = lla2enu(CPT_Data(idt,4:6),lla0,"ellipsoid");
        id_GNSS_pos = find(round([GNSS_Data{:,1}]') == CPT_Data(idt,3));  
        if ~isempty(id_GNSS_pos)
            id_GNSS_pos = id_GNSS_pos(1);
            if id_GNSS_pos == 1   % skip the first epoch 
                continue;
            end
            temp.GNSS_llh = GNSS_llh(id_GNSS_pos-1,:);  

            disp(['The obtained index: ', num2str(id_GNSS_pos), ...
                  '  The size of CPT_Data: ', num2str(num_CPT)]); 
        else
            temp.GNSS_llh = [nan,nan,nan]; % enter this choice : no GNSS_llh  % added on 2021.12.25
            disp('The index is missing...');
        end

        
        temp.GNSS_xyz = llh2xyz([temp.GNSS_llh(1),temp.GNSS_llh(2),CPT_Data(idt,6)]);
        temp.GNSS_enu = lla2enu([temp.GNSS_llh(1),temp.GNSS_llh(2),CPT_Data(idt,6)],lla0,"ellipsoid");
        temp.GNSS_ls_enu = Solu.GNSS_enu(idt,2:4); % [Solu.GNSS_enu(idt,2),Solu.GNSS_enu(idt,3),CPT_Data(idt,6)];
        
        %2d gnss (only WLS has xyz coordinates) %
        % WLS enu components (obtained from 'LCKF_INS_GNSS_lite') %
        temp.err2_GNSS_xyz_com = (CPT_xyz(idt,:)-temp.GNSS_xyz);
        temp.err2_GNSS_enu_com = (CPT_enu(idt,:)-temp.GNSS_enu);
        temp.err2_GNSS_xyz = norm(temp.err2_GNSS_xyz_com); 
        temp.err2_GNSS_enu = norm(temp.err2_GNSS_enu_com); % WLS enu

        % LS enu components %
        temp.err2_GNSS_ls_enu_com = (CPT_enu(idt,:)-temp.GNSS_ls_enu);
        temp.err2_GNSS_ls_enu = norm(temp.err2_GNSS_ls_enu_com(1,1:2)); % LS enu
        
        [~, id_LCKF_pos] = min(abs(LCKF_result(:,1) - CPT_Data(idt,1)));
        temp.LCKF_xyz = [SVs(idt, 1:2), CPT_Data(idt,6)];  
        temp.LCKF_enu = lla2enu([SVs(idt, 4:5), CPT_Data(idt,6)],lla0,"flat");
        id_LCKF(idt-1,1) = id_LCKF_pos;

        %2d lckf
        temp.err2_LCKF_xyz_com = (CPT_xyz(idt,:)-temp.LCKF_xyz);
        temp.err2_LCKF_enu_com = (CPT_enu(idt,:)-temp.LCKF_enu);
        temp.err2_LCKF_xyz = norm(temp.err2_LCKF_xyz_com);
        temp.err2_LCKF_enu = norm(temp.err2_LCKF_enu_com);
        
        % 2D: GNSS (in xyz) %% col2(WLS) %% col3(LCKF) %
        Error_2D_xyz(idt, 1:3) = [CPT_Data(idt,3),temp.err2_GNSS_xyz,temp.err2_LCKF_xyz];
        % 2D: GNSS (in enu) %% col2(LS) %% col3(WLS) %% col4(LCKF) %
        Error_2D_enu(idt, 1:4) = [CPT_Data(idt,3),temp.err2_GNSS_ls_enu,temp.err2_GNSS_enu,temp.err2_LCKF_enu]; 
        % 2D error components (in xyz) %% col2-4(WLS) %% col5-7(LCKF) %
        Error_2D_xyz_com(idt, 1:7) = [CPT_Data(idt,3),temp.err2_GNSS_xyz_com(1:3),temp.err2_LCKF_xyz_com(1:3)]; 
        % 2D error components (in enu) %% col2-4(LS) %% col5-7(WLS) %% col8-10(LCKF) %
        Error_2D_enu_com(idt, 1:10) = [CPT_Data(idt,3),temp.err2_GNSS_ls_enu_com(1:3),temp.err2_GNSS_enu_com(1:3),temp.err2_LCKF_enu_com(1:3)]; 
    end
    Error_2D_xyz(1,:)=[];
    Error_2D_enu(1,:)=[];
    Error_2D_xyz_com(1,:)=[];
    Error_2D_enu_com(1,:)=[];

    % Save the solutions in FKF
    if isequal(KF_ON,[1 0])
        Solu.Error_2D_enu(:,1:4) = Error_2D_enu;
        Solu.Error_2D_xyz(:,1:3) = Error_2D_xyz;
        Solu.Error_2D_enu_com(:,1:10) = Error_2D_enu_com;
        Solu.Error_2D_xyz_com(:,1:7) = Error_2D_xyz_com;
        Solu.id_LCKF = id_LCKF;
        % Save GNSS references %
        Solu.GNSS_Ref = GNSS_Ref;
        % Save Ground truth references %
        Solu.GT_Ref = GT_Ref;
        Solu.CPT_enu = CPT_enu;
        % Document the NaN epochs %
        id_not_nan_epoch = [];
        for ind = 1: size(Solu.Error_2D_enu, 1)
            % Check not NaN epochs in WLS column %
            if ~isnan(Solu.Error_2D_enu(ind, 3))
                id_not_nan_epoch = [id_not_nan_epoch; ind];
            end
        end
        Solu.id_not_nan_epoch = id_not_nan_epoch;
        
        % number of epochs existing in evaluation
        Solu.num_err_eva = size(Solu.Error_2D_enu,1);

        % Valid evaluation period: not nan; within data size boundaries %
        EvaPeriod = min([num_CPT, Solu.num_err_eva, Solu.id_not_nan_epoch(end,1), mannual_set_period]);
        EvaInd = Solu.id_not_nan_epoch(Solu.id_not_nan_epoch(:,1)<= EvaPeriod,1) ;
        Solu.EvaPeriod = EvaPeriod;
        Solu.EvaInd = EvaInd;

        
    elseif isequal(KF_ON,[0 1])
        % 2D: GNSS (in enu) %% col2(LS) %% col3(WLS) %% col4(LCKF) %% col5(AKF)
        Solu.Error_2D_enu(:,5) = Error_2D_enu(:,4);
        % 2D: GNSS (in xyz) %% col2(WLS) %% col3(LCKF) %% col4(AKF) %
        Solu.Error_2D_xyz(:,4) = Error_2D_xyz(:,3);
        % 2D error components (in enu) %% col2-4(LS) %% col5-7(WLS) %% col8-10(LCKF) %% col11-13(AKF) %
        Solu.Error_2D_enu_com(:,11:13) = Error_2D_enu_com(:,8:10);
        % 2D error components (in xyz) %% col2-4(WLS) %% col5-7(LCKF) %% col8-10(AKF) %
        Solu.Error_2D_xyz_com(:,8:10) = Error_2D_xyz_com(:,5:7);
    end
    
   
    disp('===> EVALUATION <===');
    disp(['===> Within ',num2str(Solu.EvaPeriod),' epochs are considered ']);
    disp(['===> Actually ',num2str(size(Solu.EvaInd,1)),' valid epochs are used ']);

    
    %=% 1.LS %=%
    LS_RMSE_2D = rms(Solu.Error_2D_enu(~isnan(Solu.Error_2D_enu(:,2)),2));
    LS_RMSE_ENU = [ rms(abs(Solu.Error_2D_enu_com(~isnan(Solu.Error_2D_enu_com(:,2)),2))),...
                    rms(abs(Solu.Error_2D_enu_com(~isnan(Solu.Error_2D_enu_com(:,3)),3)))];
    LS_STD_2D = std(Solu.Error_2D_enu(~isnan(Solu.Error_2D_enu(:,2)),2));
    LS_STD_ENU =  [ std(abs(Solu.Error_2D_enu_com(~isnan(Solu.Error_2D_enu_com(:,2)),2))),...
                    std(abs(Solu.Error_2D_enu_com(~isnan(Solu.Error_2D_enu_com(:,3)),3)))];
    fprintf('GNSS LS RMSE: 2D( %.2f m )\n',LS_RMSE_2D);
    fprintf("GNSS LS RMSE Components: E: %.2f, N: %.2f (m)\n",LS_RMSE_ENU(1),LS_RMSE_ENU(2));
    fprintf('GNSS LS STD: 2D( %.2f m )\n',LS_STD_2D);
    fprintf("GNSS LS STD Components: E: %.2f, N: %.2f(m)\n",LS_STD_ENU(1),LS_STD_ENU(2));
    Solu.Evaluation_LS = [LS_RMSE_2D, LS_RMSE_ENU, LS_STD_2D, LS_STD_ENU];


    %=% 2.WLS %=%
    WLS_RMSE_2D = rms(Solu.Error_2D_enu(~isnan(Solu.Error_2D_enu(:,3)),3));
    WLS_RMSE_ENU = [ rms(abs(Solu.Error_2D_enu_com(~isnan(Solu.Error_2D_enu_com(:,5)),5))),...
                    rms(abs(Solu.Error_2D_enu_com(~isnan(Solu.Error_2D_enu_com(:,6)),6)))];
    WLS_STD_2D = std(Solu.Error_2D_enu(~isnan(Solu.Error_2D_enu(:,3)),3));
    WLS_STD_ENU = [ std(abs(Solu.Error_2D_enu_com(~isnan(Solu.Error_2D_enu_com(:,5)),5))),...
                    std(abs(Solu.Error_2D_enu_com(~isnan(Solu.Error_2D_enu_com(:,6)),6)))];
    fprintf('GNSS WLS RMSE: 2D( %.2f m )\n',WLS_RMSE_2D );
    fprintf("GNSS WLS RMSE Components: E: %.2f, N: %.2f (m)\n",WLS_RMSE_ENU(1),WLS_RMSE_ENU(2));
    fprintf('GNSS WLS STD: 2D( %.2f m )\n',WLS_STD_2D );
    fprintf("GNSS WLS STD Components: E: %.2f, N: %.2f (m)\n",WLS_STD_ENU(1),WLS_STD_ENU(2));
    Solu.Evaluation_WLS = [WLS_RMSE_2D, WLS_RMSE_ENU, WLS_STD_2D, WLS_STD_ENU];


    %=% 3.LCKF %=%
    % FKF
    if isequal(KF_ON, [1 0])
        col_2D = 4;
        col_ENU = [8 9 10];
        LCKF_name = 'FKF';
    % AKF
    elseif isequal(KF_ON, [0 1])
        col_2D = 5;
        col_ENU = [11 12 13];
        LCKF_name = 'AKF';
    end
    LCKF_RMSE_2D = rms(Solu.Error_2D_enu(~isnan(Solu.Error_2D_enu(:,col_2D)),col_2D));
    LCKF_RMSE_ENU = [rms(abs(Solu.Error_2D_enu_com(~isnan(Solu.Error_2D_enu_com(:,col_ENU(1))), col_ENU(1)))),...
                    rms(abs(Solu.Error_2D_enu_com(~isnan(Solu.Error_2D_enu_com(:,col_ENU(2))), col_ENU(2))))];
    LCKF_STD_2D = std(Solu.Error_2D_enu(~isnan(Solu.Error_2D_enu(:,col_2D)),col_2D));
    LCKF_STD_ENU = [std(abs(Solu.Error_2D_enu_com(~isnan(Solu.Error_2D_enu_com(:,col_ENU(1))), col_ENU(1)))),...
                    std(abs(Solu.Error_2D_enu_com(~isnan(Solu.Error_2D_enu_com(:,col_ENU(2))), col_ENU(2))))];
    fprintf([LCKF_name,' RMSE: 2D( %.2f m ) \n'], LCKF_RMSE_2D );
    fprintf([LCKF_name,' RMSE Components: E: %.2f, N: %.2f (m)\n'], LCKF_RMSE_ENU(1), LCKF_RMSE_ENU(2) );
    fprintf([LCKF_name,' STD: 2D( %.2f m )  \n'], LCKF_STD_2D );
    fprintf([LCKF_name,' STD Components: E: %.2f, N: %.2f (m)\n'], LCKF_STD_ENU(1), LCKF_STD_ENU(2) );
    if isequal(KF_ON, [1 0])
        Solu.Evaluation_KF = [LCKF_RMSE_2D, LCKF_RMSE_ENU, LCKF_STD_2D, LCKF_STD_ENU];
    elseif isequal(KF_ON, [0 1])
        Solu.Evaluation_AKF = [LCKF_RMSE_2D, LCKF_RMSE_ENU, LCKF_STD_2D, LCKF_STD_ENU];
    end


    disp('===> Start saving solutions <===');
    % Save the KF results %
    if isequal(KF_ON,[1 0])
        Solu.FKF_result(:,1) = IMU_Data(:,20);
        Solu.FKF_result(:,2:4) = LCKF_result(:,2:4);
        Solu.FKF_SVs = SVs;

        for i = 1 :size(Solu.Error_2D_enu, 1)
            % Find all the imu data in this second %
            imu_data_this_epoch = find(floor(Solu.FKF_result(:,1))== Solu.Error_2D_enu(i,1));
            % document GNSS time %
            Solu.FKF_result_in_second(i,1) = Solu.Error_2D_enu(i,1);
            % select the first imu data point in this second as the KF LLH result %
            Solu.FKF_result_in_second(i,2:4) =  Solu.FKF_SVs(i,4:6);
        end
        
        % Get ENU solution for LCKF % (time-consuming)
        Solu.FKF_enu = zeros(size(LCKF_result,1),4);
        for i = 1: size(LCKF_result,1)
            Solu.FKF_enu (i,1) =  LCKF_result(i,1);
            Solu.FKF_enu (i,2:4) =  lla2enu(LCKF_result(i,2:4),lla0,"ellipsoid");
        end
        
        disp(['===> Fixed LCKF Complete ,',num2str(ProsTime),' seconds in total <===']);
        save([CurFolder,'/POSITIONING_SOLUTIONS_OF_ALL_INTEGRATION_METHODS.mat'],'Solu');
        disp('...FKF results saved');
    elseif isequal(KF_ON,[0 1])
        Solu.AKF_result(:,1) = IMU_Data(:,20);
        Solu.AKF_result(:,2:4) = LCKF_result(:,2:4);
        Solu.AKF_SVs = SVs;
        
        for i = 1 :size(Solu.Error_2D_enu, 1)
            % Find all the imu data in this second %
            imu_data_this_epoch = find(floor(Solu.AKF_result(:,1))== Solu.Error_2D_enu(i,1));
            % document GNSS time %
            Solu.AKF_result_in_second(i,1) = Solu.Error_2D_enu(i,1);
            % select the first imu data point in this second as the KF LLH result %
            Solu.AKF_result_in_second(i,2:4) =  Solu.AKF_SVs(i,4:6);

            ucts_data_this_epoch = find(Ucts(:,1)== Solu.Error_2D_enu(i,1));
            if ~isempty(ucts_data_this_epoch)
                Solu.AKF_result_in_second(i,5) = Ucts(ucts_data_this_epoch, 2);
            else
                Solu.AKF_result_in_second(i,5) = nan;
            end
        end

        % Get ENU solution for LCKF %
        Solu.AKF_enu = zeros(size(LCKF_result,1),4);
        for i = 1: size(LCKF_result,1)
            Solu.AKF_enu (i,1) =  LCKF_result(i,1);
            Solu.AKF_enu (i,2:4) =  lla2enu(LCKF_result(i,2:4),lla0,"ellipsoid");
        end

        Solu.AKF_enu(:,5) = IMU_Data(:,20); % GNSS time
        Solu.AKF_enu(:,6) = LCKF_result(:,11); % GNSS solution uncertainty applied
        
        disp(['===> Adaptive LCKF Complete ,',num2str(ProsTime),' seconds in total <===']);
        save([CurFolder,'/POSITIONING_SOLUTIONS_OF_ALL_INTEGRATION_METHODS.mat'],'Solu');
        disp('...AKF results saved');
    end
end



%% FGO Method
if ismember(1,FGO_ON)
    load([CurFolder,'/PROCESSED_DATA_FOR_FACTOR_GRAPH_OPTIMIZATION.mat']);

    tic % Timer starts (FGO)

    numGPSSamplesPerOptim = 2;
    if isequal(FGO_ON,[0 1])
        disp('AFGO starts...');
    else
        disp('FGO starts...');
    end

    nodesPerTimeStep = [exampleHelperFactorGraphNodes.Pose; ...
                        exampleHelperFactorGraphNodes.Velocity; ...
                        exampleHelperFactorGraphNodes.IMUBias];
    numNodesPerTimeStep = numel(nodesPerTimeStep);
    currNodeIDs = nodesPerTimeStep + [0 numNodesPerTimeStep];
    currNodeIDs = currNodeIDs(:).';
    
    
    t0IndexPose = exampleHelperFactorGraphNodes.Pose;
    t1IndexPose = t0IndexPose + numNodesPerTimeStep;
    t0IndexVel = exampleHelperFactorGraphNodes.Velocity;
    t1IndexVel = t0IndexVel + numNodesPerTimeStep;
    t0IndexBias = exampleHelperFactorGraphNodes.IMUBias;
    t1IndexBias = t0IndexBias + numNodesPerTimeStep;

    %%
    G = factorGraph;
    prevPose =  [initPos compact(initOrient)]; % x y z | qw qx qy qz
    prevVel = [0 0 0]; % m/s
    prevBias = [0 0 0 0 0 0]; % Gyroscope and accelerometer bias      
    
    fPosePrior = factorPoseSE3Prior(currNodeIDs(t0IndexPose), ...
                                    Measurement=prevPose, ...
                                    Information=diag([4e4 4e4 4e4 1e4 1e4 1e4]));
    fVelPrior = factorVelocity3Prior(currNodeIDs(t0IndexVel), ...
                                    Measurement=prevVel, ...
                                    Information=100*eye(3));
    fBiasPrior = factorIMUBiasPrior(currNodeIDs(t0IndexBias), ...
                                    Measurement=prevBias, ...
                                    Information=1e6*eye(6));
    
    addFactor(G,fPosePrior);
    addFactor(G,fVelPrior);
    addFactor(G,fBiasPrior);
    
    %%
    nodeState(G,currNodeIDs(t0IndexPose),prevPose);
    nodeState(G,currNodeIDs(t0IndexVel),prevVel);
    nodeState(G,currNodeIDs(t0IndexBias),prevBias);
    
    opts = factorGraphSolverOptions(MaxIterations=100);     
    
    numGPSSamples = size(sensorData,2); 
    poseIDs = zeros(1,numGPSSamples);
    
    %%
    visHelper = exampleHelperVisualizeFactorGraphAndFilteredPosition;

    % To document the uncertainty used
    document_uct = zeros(numGPSSamples,2);
    for ii = 1:numGPSSamples
        % 1. Store the current pose ID. 
        poseIDs(ii) = currNodeIDs(t0IndexPose);
        

        % 2. Create an IMU factor and add to factor graph. 
        fIMU = factorIMU(currNodeIDs, ...                        % current node IDs (e.g., [1,2,3,4,5,6])
                         imuFs, ...                              % imu sampling frequency (i.e.,400)
                         LC_KF_config.gyro_bias_PSD*eye(3),...   % gyro bias PSD (same in LCKF)
                         LC_KF_config.accel_bias_PSD*eye(3),...  % accel bias PSD
                         LC_KF_config.gyro_noise_PSD*eye(3),...  % gyro noise PSD
                         LC_KF_config.accel_noise_PSD*eye(3),... % accel noise PSD
                         sensorData{ii}.GyroReadings,...         % gyro & accel readings
                         sensorData{ii}.AccelReadings);
        addFactor(G,fIMU);


        % 3. Create a GPS factor if there is GPS solution and add to factor graph. 
        if ~isnan(sensorData{ii}.GNSSReading)
            if isequal(FGO_ON,[1 0])
                adaptiveUct = Fixed_FGO_Uct;
                fGPS = factorGPS(currNodeIDs(t0IndexPose), ...
                                 HDOP = adaptiveUct, ...
                                 VDOP = adaptiveUct, ...
                                 ReferenceLocation = lla0, ...
                                 Location = sensorData{ii}.GNSSReading);     % Fixed FGO
            
            elseif isequal(FGO_ON,[0 1]) 
                ml_uct = ml_prediction(find(ml_prediction(:,1) == sensorData {1,ii}.GnssT), 2) ;
    
                if ~isempty(ml_uct)
                    if ml_uct < min_uct
                        ml_uct = min_uct;
                    end
                    this_uct = ml_uct ; 
                else
                    this_uct = max_uct ; 
                end

                adaptiveUct = tuning_factor_FGO * this_uct;


                fGPS = factorGPS(currNodeIDs(t0IndexPose), ...
                                 HDOP=adaptiveUct, ...
                                 VDOP=adaptiveUct, ...
                                 ReferenceLocation=lla0, ...
                                 Location=sensorData{ii}.GNSSReading);      % Adaptive FGO
            end
            addFactor(G,fGPS);

        else
            % set uct to be 0, indicating no GNSS factor added (no GNSS/INS integration)%
            adaptiveUct = 0;
        end


        % 5. Set initial node states using previous state estimates and a state prediction from the IMU measurements.
        [predictedPose,predictedVel] = predict(fIMU,prevPose,prevVel,prevBias);
        nodeState(G,currNodeIDs(t1IndexPose),predictedPose);
        nodeState(G,currNodeIDs(t1IndexVel),predictedVel);
        nodeState(G,currNodeIDs(t1IndexBias),prevBias);
        

        % 6. Optimize the factor graph.
        if (mod(ii,numGPSSamplesPerOptim) == 0) || (ii == numGPSSamples)
            if ii~=1
                solnInfo = optimize(G,opts);
                % Visualize the current position estimate.
                FGO_LLH = updatePlot(visHelper,ii,G,poseIDs,posLLH,lla0,numIMUSamplesPerGPS);
                drawnow
            end
        end
    

        % 7. Advance one time step by updating the previous state estimates to the current ones and incrementing the current node IDs.
        prevPose = nodeState(G,currNodeIDs(t1IndexPose));
        prevVel = nodeState(G,currNodeIDs(t1IndexVel));
        prevBias = nodeState(G,currNodeIDs(t1IndexBias));
    
        currNodeIDs = currNodeIDs + numNodesPerTimeStep;
        if isequal(FGO_ON,[1 0])
            disp(['===> FGO Progress ',num2str(ii), ' of ', num2str(numGPSSamples),'   ', 'GNSS solution uct: ', num2str(adaptiveUct)]);
        else
            disp(['===> AFGO Progress ',num2str(ii), ' of ', num2str(numGPSSamples),'   ', 'GNSS solution uct: ', num2str(adaptiveUct)]);
        end

        % Document GNSS time
        document_uct(ii,1) = sensorData {1,ii}.GnssT;
        % Document GNSS solution uncertainty
        document_uct(ii,2) = adaptiveUct; 
    end


    % get the estimated positions in advanced 20231210
    FGO_LLH = zeros(921,3);
    for ii = 1:numel(poseIDs)
        this_G_enu = nodeState(G, poseIDs(ii));
        % Estimated current ENU position %
        FGO_LLH(ii,1:3) = enu2lla(this_G_enu(1:3), lla0, "flat");
    end %
    FGO_LLH(1,:)=[];


    % Extract all the GNSS epochs %
    FGO_GNSST = [];
    for i =1:numGPSSamples
        if i==1
            continue
        end
        FGO_GNSST = [FGO_GNSST;sensorData{1,i}.GnssT];
    end
    FGO_LLH = [FGO_GNSST, FGO_LLH];

    FGO_LLH_fit = zeros(Solu.num_err_eva, 3);
    % added on 2023.04.21 %
    for i = 1:Solu.num_err_eva
        this_llh = FGO_LLH(find(FGO_LLH(:,1) == Solu.Error_2D_enu(i,1)), 2:4);
        if ~isempty(this_llh)
            FGO_LLH_fit(i,:) = this_llh;
        else
            FGO_LLH_fit(i,:) = [nan,nan,nan];
        end
    end


    ProsTime(1) = toc; % document the processing time (@FGO)
    if isequal(FGO_ON,[1 0])
        Solu.FGO_LLH = FGO_LLH_fit;
        disp(['===> Fixed FGO Complete ,',num2str(ProsTime),' seconds in total <===']);
        Solu.Time_FGO = ProsTime;
        Solu.solnInfo_FGO = solnInfo;

    elseif isequal(FGO_ON,[0 1])
        Solu.AFGO_LLH = FGO_LLH_fit;
        disp(['===> Adaptive FGO Complete ,',num2str(ProsTime),' seconds in total <===']);
        Solu.Time_AFGO = ProsTime;
        Solu.solnInfo_AFGO = solnInfo;
    end
    
    % posLLH is GT position in llh %
    % GT ENU of all epochs
    onBoardPos = lla2enu(posLLH, lla0,"flat"); 
    % Estimated ENU positions of GNSS epochs
    estPos = zeros(numel(poseIDs),3); 
    for ii = 1:numel(poseIDs)
        currPose = nodeState(G, poseIDs(ii));
        % Estimated current ENU position %
        estPos(ii,1:3) = currPose(1:3);
        % Document GNSS time 
        estPos(ii,4) = sensorData{1,ii}.GnssT;
    end
    

    for ind = 1: size(onBoardPos,1)
        % Document GNSS time (col1)
        posDiff(ind, 1) = CPT_Data(ind, 3);
        % Check if FGO time exists in CPT data, add enu (col2-4)
        this_enu = estPos(find(estPos(:,4) == CPT_Data(ind, 3)), 1:3);
        if ~isempty(this_enu) % error differences (col2-4); position enu (col6-8)
            posDiff(ind, 2:4) = this_enu - onBoardPos(ind,1:3);
            posDiff(ind, 6:8) = this_enu;
        else
            posDiff(ind, 2:4) = [nan,nan,nan];
            posDiff(ind, 6:8) = [nan,nan,nan];
        end
        posDiff(ind,5) = norm(posDiff(ind,2:3)); %2D norm (col5)
    end

    % Combine posDiff into Error_3D_enu_com
    [~,valid_fgo_time] = ismember(posDiff(:,1), Solu.Error_2D_enu(:,1));
    posDiff = posDiff(find(valid_fgo_time~=0), :); % posDiff -> Error_2D_enu

    if isequal(FGO_ON, [1 0])
        Solu.Error_2D_enu_com(:,14:16) = posDiff(:, 2:4);
        Solu.Error_2D_enu(:, 6) = posDiff(:, 5);
        FGO_Name = 'FGO';
    elseif isequal(FGO_ON, [0 1])
        Solu.Error_2D_enu_com(:,17:19) = posDiff(:, 2:4);
        Solu.Error_2D_enu(:, 7) = posDiff(:, 5);
        FGO_Name = 'AFGO';
    end
    
    
    % Some epochs for check
    disp('===> EVALUATION <===');
    disp(['===> Within ',num2str(Solu.EvaPeriod),' epochs are considered ']);
    disp(['===> Actually ',num2str(length(Solu.EvaInd)),' valid epochs are used ']);
    FGO_RMSE_2D = rms(posDiff(~isnan(posDiff(:,5)),5));
    FGO_RMSE_ENU = [rms(abs(posDiff(~isnan(posDiff(:,2)),2))),...
                    rms(abs(posDiff(~isnan(posDiff(:,3)),3)))];
    FGO_STD_2D = std(posDiff(~isnan(posDiff(:,5)),5));
    FGO_STD_ENU = [std(abs(posDiff(~isnan(posDiff(:,2)),2))),...
                   std(abs(posDiff(~isnan(posDiff(:,3)),3)))];


    fprintf([FGO_Name,' RMSE: 2D( %.2f m ) \n'],FGO_RMSE_2D );
    fprintf([FGO_Name,' RMSE Components: E: %.2f, N: %.2f (m)\n'],FGO_RMSE_ENU(1),FGO_RMSE_ENU(2) );
    fprintf([FGO_Name,' STD: 2D( %.2f m ) \n'],FGO_STD_2D );
    fprintf([FGO_Name,' STD Components: E: %.2f, N: %.2f (m)\n'],FGO_STD_ENU(1),FGO_STD_ENU(2) );
    
    if isequal(FGO_ON, [1 0])
        Solu.Evaluation_FGO = [FGO_RMSE_2D, FGO_RMSE_ENU, FGO_STD_2D, FGO_STD_ENU];
        for i = 1: size(posDiff,1)
            Solu.FGO_enu (i,1) = CPT_Data(i,1);
            Solu.FGO_enu (i,2:4) =  posDiff(i, 6:8);
        end
        % document FGO uct
        for ind =1:size(posDiff,1)
            this_uct = document_uct(find(document_uct(:,1) == posDiff(ind,1)), 2);
            if ~isempty(this_uct)
                Solu.FGO_enu(ind,5) = posDiff(ind,1);
                Solu.FGO_enu(ind,6) = this_uct;
            else
                Solu.FGO_enu(ind,5) = posDiff(ind,1);
                Solu.FGO_enu(ind,6) = nan;
            end
        end

    elseif isequal(FGO_ON, [0 1])
        Solu.Evaluation_AFGO = [FGO_RMSE_2D, FGO_RMSE_ENU, FGO_STD_2D, FGO_STD_ENU];
        for i = 1: size(posDiff,1)
            Solu.AFGO_enu (i,1) = CPT_Data(i,1);
            Solu.AFGO_enu (i,2:4) =  posDiff(i, 6:8);
        end
        % document FGO uct
        for ind =1:size(posDiff,1)
            this_uct = document_uct(find(document_uct(:,1) == posDiff(ind,1)), 2);
            if ~isempty(this_uct)
                Solu.AFGO_enu(ind,5) = posDiff(ind,1);
                Solu.AFGO_enu(ind,6) = this_uct;
            else
                Solu.AFGO_enu(ind,5) = posDiff(ind,1);
                Solu.AFGO_enu(ind,6) = nan;
            end
        end
    end

    disp('===> Start saving solutions <===');
    % Save the FGO results %
    if isequal(FGO_ON,[1 0])
        save([CurFolder,'/POSITIONING_SOLUTIONS_OF_ALL_INTEGRATION_METHODS.mat'],'Solu');
        savefig([CurFolder,'/FGO_satellite_img.fig']);
        disp('...FGO results saved');
    elseif isequal(FGO_ON,[0 1])
        save([CurFolder,'/POSITIONING_SOLUTIONS_OF_ALL_INTEGRATION_METHODS.mat'],'Solu');
        savefig([CurFolder,'/AFGO_satellite_img.fig']);
        disp('...AFGO results saved');
    end
end


%% MAKING PLOTS 
EvaPeriod = 920; % (user-defined)
%=% (Fig 1: Trajectory of different solutions) %=%
figure; 
hold on; 
% Ground Truth %
plot(Solu.GT_Ref(1:EvaPeriod,2),Solu.GT_Ref(1:EvaPeriod,1), ...
    '-g','LineWidth',2);
% LS %
plot(Solu.GNSS_Ref(1:EvaPeriod,2),Solu.GNSS_Ref(1:EvaPeriod,1), ...
    'o-','MarkerSize',5,'MarkerFaceColor',[0.95, 0.85, 0.1],'MarkerEdgeColor','k','LineWidth',0.5,'Color','k');
% KF and AKF %
if isequal(KF_ON, [1 0])
    ending_ind = Solu.id_LCKF(EvaPeriod,1);
    plot(Solu.FKF_result_in_second(1:EvaPeriod,3), Solu.FKF_result_in_second(1: EvaPeriod,2), ...
        'o','MarkerSize',5,'MarkerFaceColor','b','MarkerEdgeColor','k','LineWidth',0.5);
end
if isequal(KF_ON, [0 1])
    ending_ind = Solu.id_LCKF(EvaPeriod,1);
    plot(Solu.FKF_result_in_second(1:EvaPeriod,3), Solu.FKF_result_in_second(1:EvaPeriod,2), ...
        'o','MarkerSize',5,'MarkerFaceColor','b','MarkerEdgeColor','k','LineWidth',0.5);
    plot(Solu.AKF_result_in_second(1:EvaPeriod,3), Solu.AKF_result_in_second(1:EvaPeriod,2), ...
        'o','MarkerSize',5,'MarkerFaceColor','c','MarkerEdgeColor','k','LineWidth',0.5);
end
% FGO and AFGO %
if isequal(FGO_ON, [1 0])
    ending_ind = Solu.id_LCKF(EvaPeriod,1);
    plot(Solu.FKF_result_in_second(1:EvaPeriod,3), Solu.FKF_result_in_second(1:EvaPeriod,2), ...
        'o','MarkerSize',5,'MarkerFaceColor','b','MarkerEdgeColor','b','LineWidth',0.5);
    plot(Solu.AKF_result_in_second(1:EvaPeriod,3), Solu.AKF_result_in_second(1:EvaPeriod,2), ...
        'o','MarkerSize',5,'MarkerFaceColor','c','MarkerEdgeColor','c','LineWidth',0.5);
    plot(Solu.FGO_LLH(1: EvaPeriod,2), Solu.FGO_LLH(1: EvaPeriod,1), ...
        'o','MarkerSize',5,'MarkerFaceColor','r','MarkerEdgeColor','k','LineWidth',0.5);
end
if isequal(FGO_ON, [0 1])
    ending_ind = Solu.id_LCKF(EvaPeriod,1);
    plot(Solu.FKF_result_in_second(1:EvaPeriod,3), Solu.FKF_result_in_second(1:EvaPeriod,2), ...
        'o','MarkerSize',5,'MarkerFaceColor','b','MarkerEdgeColor','b','LineWidth',0.5);
    plot(Solu.AKF_result_in_second(1:EvaPeriod,3), Solu.AKF_result_in_second(1:EvaPeriod,2), ...
        'o','MarkerSize',5,'MarkerFaceColor','c','MarkerEdgeColor','c','LineWidth',0.5);
    plot(Solu.FGO_LLH(1: EvaPeriod,2), Solu.FGO_LLH(1: EvaPeriod,1), ...
        'o','MarkerSize',5,'MarkerFaceColor','r','MarkerEdgeColor','k','LineWidth',0.5);
    plot(Solu.AFGO_LLH(1: EvaPeriod,2), Solu.AFGO_LLH(1: EvaPeriod,1), ...
        'o','MarkerSize',5,'MarkerFaceColor','m','MarkerEdgeColor','k','LineWidth',0.5);
end

% add labels %
ylabel('Latitude (deg)');
xlabel('Longitude (deg)');

% add legends %
if isequal(KF_ON, [1 0]) && isequal(FGO_ON, [0 0])
    legend('Truth','LS','KF');
    savefig([CurFolder,'/Traj_KF.fig']);
elseif isequal(KF_ON, [0 1]) && isequal(FGO_ON, [0 0])
    legend('Truth','LS','KF','AKF');
    savefig([CurFolder,'/Traj_AKF.fig']);
elseif isequal(KF_ON, [0 0]) && isequal(FGO_ON, [1 0])
    legend('Truth','LS','KF','AKF','FGO');
    savefig([CurFolder,'/Traj_FGO.fig']);
elseif isequal(KF_ON, [0 0]) && isequal(FGO_ON, [0 1])
    legend('Truth','LS','KF','AKF','FGO','AFGO');
    savefig([CurFolder,'/Traj_AFGO.fig']);
end

%=% (Fig 2: Solution 2D errors) %=%
figure;
hold on;
if ismember(1, KF_ON)
    Error_ind_size = min([Solu.num_err_eva, EvaPeriod]);
    Error_ind = linspace(1,Error_ind_size,Error_ind_size)';
    if isequal(KF_ON, [1 0])
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,2),'-','LineWidth',1.5,'Color',[0.95, 0.85, 0.1]); % error GNSS LS
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,4),'b-','LineWidth',1.5); % error LCKF
        legend('LS','KF');
    elseif isequal(KF_ON, [0 1])
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,2),'-','LineWidth',1.5,'Color',[0.95, 0.85, 0.1]); % error GNSS LS
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,4),'b-','LineWidth',1.5); % error LCKF
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,5),'c-','LineWidth',1.5); % error AKF
        legend('LS','KF','AKF');
    end
    ylabel('2D Error (m)');
    xlabel('epoch');

elseif ismember(1, FGO_ON)
    Error_ind_size = min([Solu.num_err_eva, EvaPeriod]);
    Error_ind = linspace(1,Error_ind_size,Error_ind_size)';
    if isequal(FGO_ON, [1 0])
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,2),'-','LineWidth',1.5 ,'Color',[0.95, 0.85, 0.1]); % error GNSS LS
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,4),'b-','LineWidth',1.5); % error LCKF
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,5),'c-','LineWidth',1.5); % error AKF
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,6),'r-','LineWidth',1.5); % error FGO
        legend('LS','KF','AKF','FGO');
    elseif isequal(FGO_ON, [0 1])
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,2),'-','LineWidth',1.5 ,'Color',[0.95, 0.85, 0.1]); % error GNSS LS
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,4),'b-','LineWidth',1.5); % error LCKF
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,5),'c-','LineWidth',1.5); % error AKF
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,6),'r-','LineWidth',1.5); % error FGO
        plot(Error_ind, Solu.Error_2D_enu(1:Error_ind_size,7),'g-','LineWidth',1.5); % error AFGO
        legend('LS','KF','AKF','FGO','AFGO');
    end
    ylabel('2D Error (m)');
    xlabel('epoch');
end
savefig([CurFolder,'/2dE.fig']);






