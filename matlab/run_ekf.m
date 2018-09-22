close all
clear all

load('data.mat')

% this will limit the used data and simplify the model
%   - only x,y GNSS data is used
%   - only z IMU rotation is used
%   - only x IMU acceleratoin is used (in driving direction)
ekf.settings.two_dimensional = true;
ekf.settings.sync = false;

x0 = init_ekf(ekf);

t = 0;

% these indices keep track up to which sample the data has been processed for 
% the given sensor so far
gnss_index = 0;
speedometer_index = 0;
imu_index = 0;

while true
    try
        gnss_next_time = in_data.GNSS.t(gnss_index+1);
        speedometer_next_time = in_data.SPEEDOMETER.t(speedometer_index+1);
        imu_next_time = in_data.IMU.t(imu_index+1);
    catch 
        % once index out of bounds, one kind of measurement has been completely processed
        % stop then
        break;
    end
        
    if gnss_next_time < speedometer_next_time && gnss_next_time < imu_next_time
        gnss_index = gnss_index + 1;
    elseif speedometer_next_time < imu_next_time
        speedometer_index = speedometer_index + 1;
    else
        imu_index = imu_index + 1;
    end
end