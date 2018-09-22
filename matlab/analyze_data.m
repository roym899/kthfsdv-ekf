close all
clear all

load('data.mat')

total_distance_gps = 0;

gps_steps = length(in_data.GNSS.pos_ned);

for i=2:gps_steps
    total_distance_gps = sqrt(sum((in_data.GNSS.pos_ned(:,i)-in_data.GNSS.pos_ned(:,i-1)).^2)) + total_distance_gps;
end

total_distance_speedometer = 0;
speedometer_steps = length(in_data.SPEEDOMETER.speed);
for i=2:speedometer_steps 
    total_distance_speedometer = in_data.SPEEDOMETER.speed(i) * (in_data.SPEEDOMETER.t(i)-in_data.SPEEDOMETER.t(i-1)) + total_distance_speedometer;
end


hold on
figure(1)
grid on
% plot(in_data.GNSS.t, in_data.GNSS.pos_ned(1,:));
% plot(in_data.GNSS.t, in_data.GNSS.pos_ned(2,:));
% plot(in_data.GNSS.t, in_data.GNSS.pos_ned(3,:));
plot(in_data.SPEEDOMETER.t, in_data.SPEEDOMETER.speed);
plot(in_data.IMU.t, in_data.IMU.acc(1,:));
plot(in_data.IMU.t, in_data.IMU.acc(2,:));
xlabel('time /s')
plot(in_data.IMU.t, in_data.IMU.gyro(1,:));
plot(in_data.IMU.t, in_data.IMU.gyro(2,:));
plot(in_data.IMU.t, in_data.IMU.gyro(3,:));
legend('v', 'x', 'y', 'rotx', 'roty', 'rotz')
hold off
