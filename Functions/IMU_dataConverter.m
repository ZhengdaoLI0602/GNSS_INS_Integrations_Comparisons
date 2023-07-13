% Imu_Urbannav_Data_Example
function imu_data = IMU_dataConverter(IMU_directory)
% clc;
% clear;
% close all;

imu_data0 = readmatrix(IMU_directory);

%--->imu_data0
% Col-3: UTC_time      
% Col-30-32: acc- xyz
% Col-18-20: angular-xyz
% Col-5-8: orientation in quaternion

%--->imu_data
% Col-1: UTC_time      
% Col-2-4: acc- xyz
% Col-5-7: angular-xyz
% Col-8-11: orientation in quaternion
imu_data = imu_data0(:,[3,30:32,18:20,5:8]);

for idt = 1:size(imu_data,1)
    [~,tow_sec] = time2weektow(imu_data(idt,1)*10^-9+18-315964800);
    imu_data(idt,1) = tow_sec;
end


% imu_time = 1621218775548635005;
% 
% imu_time_cov = imu_time*10^-9;
% [A,B] = time2weektow(imu_time_cov+18-315964800);
% 
% utc2datenum(x)












