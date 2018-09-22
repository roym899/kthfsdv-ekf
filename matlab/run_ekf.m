close all
clear all

load('data.mat')

in_data.IMU.acc(1,:) = in_data.IMU.acc(1,:)+1;

% this will limit the used data and simplify the model
%   - only x,y GNSS data is used
%   - only z IMU rotation is used
%   - only x IMU acceleratoin is used (in driving direction)
ekf.settings.two_dimensional = true;
ekf.settings.sync = false;

[x, P] = init_ekf(ekf);
Q_IMU = [0.5 0;
         0 0.1];
R_GNSS = [2 0;
          0 2];
R_SPEEDOMETER = [0.01];

t = 0;

% these indices keep track up to which sample the data has been processed for 
% the given sensor so far
gnss_index = 0;
speedometer_index = 0;
imu_index = 0;

x_history = [x];
P_history = [];
t_history = [0];

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
        t_history = [t_history; gnss_next_time];
        [x, P] = gnss_update(ekf, x, P, R_GNSS, [in_data.GNSS.pos_ned(1,gnss_index), in_data.GNSS.pos_ned(2,gnss_index)]);
        x_history = [x_history; x];
        P_history(:,:,size(x_history,1)) = P;
        
    elseif speedometer_next_time < imu_next_time
        speedometer_index = speedometer_index + 1;
        t_history = [t_history; speedometer_next_time];
        [x, P] = speedometer_update(ekf, x, P, R_SPEEDOMETER, [in_data.SPEEDOMETER.speed(speedometer_index)]);
        x_history = [x_history; x];
        P_history(:,:,size(x_history,1)) = P;
    else
        imu_index = imu_index + 1;
        t_history = [t_history; imu_next_time];
        try
            dt = imu_next_time - in_data.IMU.t(imu_index-1);
        catch
            dt = 0.01;
        end
        [x, P] = imu_update(ekf, dt, x, P, Q_IMU, [in_data.IMU.gyro(3,imu_index) in_data.IMU.acc(1,imu_index)]);
        x_history = [x_history; x];
        P_history(:,:,size(x_history,1)) = P;
    end
end

figure(1)
hold on
grid on
xlabel('x')
ylabel('y')
zlabel('t')
plot3(x_history(:,1), x_history(:,2), t_history);
plot3(in_data.GNSS.pos_ned(1,:), in_data.GNSS.pos_ned(2,:), in_data.GNSS.t);
hold off