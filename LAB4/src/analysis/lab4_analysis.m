clc
clear all
close all

% Import Bag Files using ROS Toolbox and select gps topic
bagfile = rosbag('2022-03-04-09-21-00.bag');
magData = readMessages(select(bagfile,'Topic','/vn100_mag'),'DataFormat','struct');
imuData = readMessages(select(bagfile,'Topic','/vn100_imu'),'DataFormat','struct');
gpsData = readMessages(select(bagfile,'Topic','/gps'),'DataFormat','struct');

% 
magData = magData(16000:end);
imuData = imuData(16000:end);
gpsData = gpsData(400:end);

% IMU Data
mag = cell2mat(cellfun(@(m) [m.MagneticField_.X m.MagneticField_.Y m.MagneticField_.Z],magData,'UniformOutput',false));
quat = cell2mat(cellfun(@(m) [m.Orientation.X m.Orientation.Y m.Orientation.Z m.Orientation.W],imuData,'UniformOutput',false));
angVel = cell2mat(cellfun(@(m) [m.AngularVelocity.X m.AngularVelocity.Y m.AngularVelocity.Z],imuData,'UniformOutput',false));
linAccel = cell2mat(cellfun(@(m) [m.LinearAcceleration.X m.LinearAcceleration.Y m.LinearAcceleration.Z],imuData,'UniformOutput',false));
imu_time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-imuData{1,1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),imuData);

% There is a weird problem with the imu time... created my own time var
time = transpose(0:0.025:(49422*.025));

% GPS Data
pos = cell2mat(cellfun(@(m) [m.UtmEasting m.UtmNorthing m.Alt],gpsData,'UniformOutput',false));
gps_time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-imuData{1,1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),gpsData);

%% Uncorrected Velocity Comparison
% Get IMU Velocity
imu_a = linAccel(:,1);
imu_v = cumtrapz(time,imu_a);
% Get GPS Velocity
gps_vx = gradient(pos(:,1))./gradient(gps_time);
gps_vy = gradient(pos(:,2))./gradient(gps_time);
gps_v = sqrt(gps_vx.^2 + gps_vy.^2);

% Plot Both Velocities
figure(1)
hold on; grid on;
plot(time,imu_v,'.')
plot(gps_time,gps_v,'.')
title("Forward Velocity Comparison")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
legend("IMU","GPS")



%% Corrected Velocity Comparison
% subtract the bias from the stationary vehicle
imu_a_mean = (mean(imu_a(1:3500))+mean(imu_a(46000:end)))/2;
imu_a_adjust = imu_a-imu_a_mean;

% Remove the datapoints when we know we were stationary
a1 = smoothdata(imu_a,'gaussian',200);
a2 = gradient(a1);
for i = 1:length(a2)
    if abs(a2(i)) < 4*10^-4
        a11(i) = 0;
    else
        a11(i) = imu_a_adjust(i);
    end
end
% get a corrected velocity
imu_v = transpose(cumtrapz(time,a11));
% Plot Again
figure(2)
hold on; grid on;
plot(time,imu_v,'.')
plot(gps_time,gps_v,'.')
title("Forward Velocity Comparison - Corrected IMU")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
legend("IMU","GPS")

%% Dead Reckoning with IMU

% IMU Position
imu_p = cumtrapz(time,imu_v);
gps_p = cumtrapz(gps_time,gps_v);
gps_px = pos(:,1) - pos(1,1);
gps_py = pos(:,2) - pos(1,2);
% gps_p = sqrt(gps_px.^2+gps_py.^2);

figure(3)
hold on;grid on
plot(time,imu_p,'.')
plot(gps_time,gps_p,'.')
title("Forward Displacement Comparison")
xlabel("Time (s)")
ylabel("Displacement (m)")
legend("IMU","GPS")

% W is the rotation rate in the vehicle frame
w = angVel(:,3);
X_dot = gps_v;
wX_dot = w.*imu_v;
y_ddot = linAccel(:,2);

figure(4)
hold on;grid on
plot(time,y_ddot,'.')
plot(time,wX_dot,'.')
title("Lateral Acceleration Comparison")
xlabel("Time (s)")
ylabel("Acceleration (m/s^2)")
legend("y''","wX'")

%% Compare GPS EvsN to IMU EvsN

% This will be magnetometer data in the future!
% [roll0 pitch0 yaw0] = quat2angle([quat(:,4),quat(:,1),quat(:,2),quat(:,3)],'ZYX');

% Load magnetometer data from part1 (lab4_circles.m)
load("filtered_magnetometer_heading.mat");
yaw0 = filtered;

% do more with this? Use the equation from lab instructions?
imu_vE = -imu_v.*cos(yaw0);
imu_vN = imu_v.*sin(yaw0);
imu_pE = cumtrapz(time,imu_vE);
imu_pN = cumtrapz(time,imu_vN);

% Estimate IMU location relative to Vehicle CM
w = angVel(:,3);
w_dot = diff(w);
A = -1*w_dot;
B = wX_dot(1:end-1);
Xc_mat = linsolve(A,B);
Xc = mean(Xc_mat);

% filtered
figure(5)
hold on;grid on
plot(time,imu_vE,'.')
plot(gps_time,gps_vx,'.')
title("Easting velocity comparison?")

figure(6)
hold on;grid on
plot(time,imu_pE,'.')
plot(gps_time,gps_px,'.')
title("Easting position comparison")

figure(7)
hold on; grid on;
plot(gps_px,gps_py,'.')
plot(imu_pE,imu_pN,'.')
title("Drive Path Comparison")
xlabel("East Displacement (m)")
ylabel("North Displacement (m)")
