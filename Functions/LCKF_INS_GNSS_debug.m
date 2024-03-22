function [SVs, LCKF_result, GNSS_llh, IMU_data, Ucts] = LCKF_INS_GNSS_debug(GNSS_data, IMU_data,...
                                     LC_KF_config, fixed_uct, ml_prediction, whether_adaptive, receiver)

% ------------------------------------------------------------------------
% Main function for LCKF
% modified by Paul's example
%
% by ZD. LI
% zhengdao.li@connect.polyu.hk
%
% by GH.Zhang 
% guo-hao.zhang@connect.polyu.hk
% ------------------------------------------------------------------------

D2R = pi/180;
R2D = 180/pi;


%% Begins
% Loosely coupled ECEF Inertial navigation and GNSS integrated navigation
no_epochs_IMU = size(IMU_data,1);
no_epochs_GNSS = size(GNSS_data,1);
user_pos_xyz0 = llh2xyz(GNSS_data{1,2}([2,1,3]).*[D2R,D2R,1]);

% True Velocity
ini_v_eb_n = GNSS_data{1,13};
for id = 1:1:no_epochs_GNSS
    GNSS_data{id,15} = [GNSS_data{id,10}(:,1),GNSS_data{id,11}(:,1),...
                        GNSS_data{id,10}(:,3:5),GNSS_data{id,11}(:,2:5)];
end

%% Initialize true navigation solution
% by XSENS
old_time = IMU_data(1,1);
ini_L_b = GNSS_data{1,2}(2)*D2R;
ini_lambda_b = GNSS_data{1,2}(1)*D2R;
ini_h_b = GNSS_data{1,2}(3);
ini_eul_nb = IMU_data(1,14:16)'.*D2R;
ini_C_b_n = Euler_to_CTM(ini_eul_nb)';
[~,~,c_e_n,old_ini_C_b_e] =...
    NED_to_ECEFdebug(ini_L_b,ini_lambda_b,ini_h_b,ini_v_eb_n,ini_C_b_n); %coordinate transformation [what is NED?]



% Determine Least-squares GNSS position solution and use to initialize INS
GNSS_epoch = 1;
[GNSS_r_eb_e,~,~,~] = WLS_positioning(GNSS_data{1,10}(:,1:5),...
                                                  GNSS_data{1,9}(:,5),...
                                                  GNSS_data{1,9}(:,4),...
                                                  user_pos_xyz0);
[Doppler_V,~] = WLS_velocity(GNSS_data{GNSS_epoch,15},...
                                     GNSS_data{1,9}(:,5),...
                                     GNSS_data{1,9}(:,4),...
                                     GNSS_r_eb_e);
GNSS_xyz(GNSS_epoch,:) = GNSS_r_eb_e';
GNSS_llh(GNSS_epoch,:) = xyz2llh(GNSS_r_eb_e);
GNSS_llh(GNSS_epoch,:) = GNSS_llh(1,:).*[R2D,R2D,1];
GNSS_r_eb_e = GNSS_r_eb_e';
GNSS_v_eb_e = Doppler_V;
old_est_r_eb_e = GNSS_r_eb_e;
old_est_v_eb_e = GNSS_v_eb_e;
[old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n] =...
    pv_ECEF_to_NED(old_est_r_eb_e,old_est_v_eb_e);
est_L_b = old_est_L_b;

time_last_GNSS = old_time; % Initialize GNSS model timing


% Initialize estimated attitude solution
% old_est_C_b_n = Euler_to_CTM(CPT_data(1,10:12)'.*[D2R,D2R,D2R])';% insert imu 1st data (now CPT!!!)
old_est_C_b_n = Euler_to_CTM(IMU_data(1,14:16)'.*[D2R,D2R,D2R])';% insert imu 1st data (now AHRS)
[~,~,old_est_C_b_e] = NED_to_ECEF(old_est_L_b,...
    old_est_lambda_b,old_est_h_b,old_est_v_eb_n,old_est_C_b_n);

% Initialize output profile record and errors record
LCKF_result = zeros(no_epochs_IMU,13);

% Generate output profile record
LCKF_result(1,1) = old_time;
LCKF_result(1,2) = old_est_L_b*R2D;
LCKF_result(1,3) = old_est_lambda_b*R2D;
LCKF_result(1,4) = old_est_h_b;
LCKF_result(1,5:7) = old_est_v_eb_n';
LCKF_result(1,8:10) = CTM_to_Euler(old_est_C_b_n')'.*[R2D,R2D,R2D];

% Initialize Kalman filter P matrix and IMU bias states
P_matrix = Initialize_LC_P_matrix(LC_KF_config);
est_IMU_bias = zeros(6,1);

% ---initial bias correction of IMU---
temp.g_xyz = llh2xyz([GNSS_data{1,2}(2),GNSS_data{1,2}(1),GNSS_data{1,2}(3)].*[D2R,D2R,1]);
temp.g = Gravity_ECEF(temp.g_xyz');
IMU_bias0_epoch = 500; % use the first 500 data points to calculate zero shift
IMU_data(:,7) = IMU_data(:,7) - mean(IMU_data(1:IMU_bias0_epoch,7));
IMU_data(:,8) = IMU_data(:,8) - mean(IMU_data(1:IMU_bias0_epoch,8));
IMU_data(:,9) = IMU_data(:,9) - mean(IMU_data(1:IMU_bias0_epoch,9));
IMU_data(:,4) = IMU_data(:,4) - mean(IMU_data(1:IMU_bias0_epoch,4));
IMU_data(:,5) = IMU_data(:,5) - mean(IMU_data(1:IMU_bias0_epoch,5));
IMU_data(:,6) = IMU_data(:,6) - mean(IMU_data(1:IMU_bias0_epoch,6)) + norm(temp.g).*1;
% -------------------------------------

% True GNSS debug

% Generate IMU bias and clock output records
out_IMU_bias_est(1,1) = old_time;
out_IMU_bias_est(1,2:7) = est_IMU_bias';

% Generate KF uncertainty record
out_KF_SD(1,1) = old_time;
for i =1:15
    out_KF_SD(1,i+1) = sqrt(P_matrix(i,i));
end % for i


%% Main loop
Ucts = [];
if whether_adaptive == 1
    adaptiveR = ml_prediction(1,2);
elseif whether_adaptive == 0
    adaptiveR = fixed_uct;
end

% Manually set to the next epoch %
GNSS_epoch = 2;
whether_integrate = 'Yes';
SVs = [];
P_matrix_s = {};
Phi_matrix_s = {};

test_ind = 7:9; % index of positions for RTS smoother
P_matrix_s(1,:) = {P_matrix(test_ind, test_ind), P_matrix(test_ind, test_ind)};
SVs(1,:) = old_est_r_eb_e;

for epoch = 2:no_epochs_IMU 
    float_GNSS_epoch = round(epoch/400, 2);
    floor_epoch = floor(float_GNSS_epoch);
    ceil_epoch = ceil(float_GNSS_epoch);

    if abs(float_GNSS_epoch-floor_epoch)<4e-3 || abs(float_GNSS_epoch-ceil_epoch)<4e-3 
        disp(['GNSS Epoch ==> ',num2str(float_GNSS_epoch),'/',num2str(round(no_epochs_IMU/400)),'  ','GNSS solution uct: ',num2str(adaptiveR), ' Integrate: ', whether_integrate]);
    end

    % Input data from motion profile
    time = IMU_data(epoch,1);

    % Time interval
    tor_i = time - old_time;
    
    % Correct IMU errors
    meas_f_ib_b = [IMU_data(epoch,5);...
                   IMU_data(epoch,4);...
                   -IMU_data(epoch,6)];
    meas_omega_ib_b = [IMU_data(epoch,8)*0;...
                       IMU_data(epoch,7)*0;...
                       -IMU_data(epoch,9)];
    meas_f_ib_b = meas_f_ib_b - est_IMU_bias(1:3); % accelerometer
    meas_omega_ib_b = meas_omega_ib_b - est_IMU_bias(4:6); % gyroscope
    
    % linear acc in imu %
    LCKF_result(epoch, 12:14) = meas_f_ib_b';
    % angular vel in imu %
    LCKF_result(epoch, 15:17) = meas_omega_ib_b';

    
    % Update estimated navigation solution
    [est_r_eb_e, est_v_eb_e, est_C_b_e, f_ib_e_debug] = Nav_equations_ECEFdelete(tor_i,...
        old_est_r_eb_e, old_est_v_eb_e, old_est_C_b_e, meas_f_ib_b, meas_omega_ib_b);

    float_GNSS_time = time + GNSS_data{1,1};

    % Determine whether to update GNSS simulation and run Kalman filter
    if  GNSS_epoch<size(GNSS_data,1) && float_GNSS_time >= GNSS_data{GNSS_epoch, 1} 
    % Add GNSS ----- starts integration
        temp.prn = GNSS_data{GNSS_epoch,10}(:,1);
        temp.nsv0 = 3+any(temp.prn>0& temp.prn<=32) +...
                    any(temp.prn>32& temp.prn<=56) +...
                    any(temp.prn>56& temp.prn<=86) +...
                    any(temp.prn>86& temp.prn<=123);
        
        if size(GNSS_data{GNSS_epoch,10},1)>=temp.nsv0 %&& GNSS_epoch~=259 % enough SV
            tor_s = time - time_last_GNSS;  % KF time interval
            time_last_GNSS = time;

            % Determine GNSS position solution
            [GNSS_r_eb_e,~,~,~] = WLS_positioning(GNSS_data{GNSS_epoch,10}(:,1:5),...
                                                  GNSS_data{GNSS_epoch,9}(:,5),...
                                                  GNSS_data{GNSS_epoch,9}(:,4),...
                                                  user_pos_xyz0);
            [Doppler_V,~] = WLS_velocity(GNSS_data{GNSS_epoch,15},...
                                         GNSS_data{GNSS_epoch,9}(:,5),...
                                         GNSS_data{GNSS_epoch,9}(:,4),...
                                         GNSS_r_eb_e);
            GNSS_xyz(GNSS_epoch,:) = GNSS_r_eb_e';
            GNSS_llh(GNSS_epoch,:) = xyz2llh(GNSS_r_eb_e);
            GNSS_llh(GNSS_epoch,:) = GNSS_llh(GNSS_epoch,:).*[R2D,R2D,1];
            GNSS_r_eb_e = GNSS_r_eb_e';
            GNSS_v_eb_e = Doppler_V;


            % -------------------------
            if whether_adaptive == 1
                % modified by LZD on 2024.03.22 % Adaptive KF
                adaptiveR = AdaptiveR(GNSS_data{GNSS_epoch, 1}, ml_prediction, receiver);   
            elseif whether_adaptive == 0
                % modified by LZD on 2021.12.27 % Fixed KF
                adaptiveR = fixed_uct;
            end 
            % -------------------------
            Ucts(GNSS_epoch-1, 1) = GNSS_data{GNSS_epoch, 1};
            Ucts(GNSS_epoch-1, 2) = adaptiveR;

            
            % Run Integration Kalman filter
            [est_C_b_e,est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix_propagated, P_matrix, Phi_matrix] =...
                LC_KF_Epoch(GNSS_r_eb_e,GNSS_v_eb_e,tor_s,est_C_b_e,...
                est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix,meas_f_ib_b,...
                est_L_b,LC_KF_config, adaptiveR);


            % Generate IMU bias and clock output records
            out_IMU_bias_est(GNSS_epoch,1) = time;
            out_IMU_bias_est(GNSS_epoch,2:7) = est_IMU_bias';

            % Generate KF uncertainty output record
            out_KF_SD(GNSS_epoch,1) = time;
            for i =1:15
                out_KF_SD(GNSS_epoch,i+1) = sqrt(P_matrix(i,i));
            end % for i

        else
            GNSS_llh(GNSS_epoch,:) = [nan,nan,nan];
            GNSS_xyz(GNSS_epoch,:) = [nan,nan,nan];
        end

        GNSS_epoch = GNSS_epoch + 1;
        whether_integrate = 'Yes!!!!!!!!!!!!!';

        % Add GNSS ----- ends integration
    else
        % add zero indicating there is no GNSS/INS integration %
        whether_integrate = 'No';
    end % if time 


    % Convert navigation solution to NED
    [est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n] =...
        ECEF_to_NED(est_r_eb_e,est_v_eb_e,est_C_b_e);

    est_C_b_n = c_e_n * est_C_b_e; % "c_e_n: ECEF to NED"; "est_C_b_e: estimated body to ECEF"; 

    % Generate output profile record
    LCKF_result(epoch,1) = time;
    LCKF_result(epoch,2) = est_L_b*R2D; % latitude in body frame
    LCKF_result(epoch,3) = est_lambda_b*R2D; % longitude in body frame
    LCKF_result(epoch,4) = est_h_b; % altitude (height) in body frame
    LCKF_result(epoch,5:7) = est_v_eb_n'; 
    LCKF_result(epoch,8:10) = CTM_to_Euler(est_C_b_n')'.*[R2D,R2D,R2D]; % orientation in body frame
    % col9 has problems: complex number results %
    if LCKF_result(epoch,10) < 0
        LCKF_result(epoch,10) = LCKF_result(epoch,10)+360;
    end
    LCKF_result(epoch,11) = adaptiveR; % document the uncertainty of GNSS solutions


    %% ---------------- Save essential elements in RTS smoother 

    if mod(epoch,400)==0
        P_matrix_s = [P_matrix_s;[{P_matrix_propagated(test_ind, test_ind)},{P_matrix(test_ind, test_ind)}]];
        Phi_matrix_s = [Phi_matrix_s; {Phi_matrix(test_ind, test_ind)}];
        this_SV = est_r_eb_e';
        
        SVs  = [SVs; this_SV];
    end


    % Reset old values
    old_time = time;
    old_est_r_eb_e = est_r_eb_e; % position
    old_est_v_eb_e = est_v_eb_e; % velocity
    old_est_C_b_e = est_C_b_e;   % orientation (with transformation matrix)
end


%% ---------------- RTS smoother
for iii = size(SVs,1): -1 :2
    K_matrix_smoothed{iii,1} = P_matrix_s{iii-1,2} * Phi_matrix_s{iii-1}' * inv(P_matrix_s{iii,1});
    P_matrix_s{iii-1, 2} = P_matrix_s{iii-1, 2} + K_matrix_smoothed{iii,1} * (P_matrix_s{iii, 2} - P_matrix_s{iii, 1}) * K_matrix_smoothed{iii,1}';   
    SVs(iii-1,:) = SVs(iii-1,:) + (K_matrix_smoothed{iii,1} * (SVs(iii,:)' - SVs(iii-1,:)'))';
end

for i = 1:size(SVs,1)
    SVs(i,4:6)=ECEF_to_NED_r_only(SVs(i,1:3)); % col 1-3: smoothed ECEF; col 4-6: smoothed LLH
end

