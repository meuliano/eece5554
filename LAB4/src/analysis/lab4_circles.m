clc
clear all
close all

% Import Bag Files using ROS Toolbox and select gps topic 18000
bagfile = rosbag('2022-03-04-09-21-00.bag');
magData = readMessages(select(bagfile,'Topic','/vn100_mag'),'DataFormat','struct');
magData = magData(10880:15000);
imuData = readMessages(select(bagfile,'Topic','/vn100_imu'),'DataFormat','struct');
imuData = imuData(10880:15000);

% Raw Heading from IMU
T = readtable('raw_csv.xlsx');
head = table2array(T(:,3));

% There is a problem with the imu header time, so created my own time var
time = 0:0.025:(4120*.025);

% IMU Data
mag = cell2mat(cellfun(@(m) [m.MagneticField_.X m.MagneticField_.Y m.MagneticField_.Z],magData,'UniformOutput',false));
quat = cell2mat(cellfun(@(m) [m.Orientation.X m.Orientation.Y m.Orientation.Z m.Orientation.W],imuData,'UniformOutput',false));
angVel = cell2mat(cellfun(@(m) [m.AngularVelocity.X m.AngularVelocity.Y m.AngularVelocity.Z],imuData,'UniformOutput',false));
linAccel = cell2mat(cellfun(@(m) [m.LinearAcceleration.X m.LinearAcceleration.Y m.LinearAcceleration.Z],imuData,'UniformOutput',false));
imu_time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-imuData{1,1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),imuData);

% Store Mag
magX = mag(:,1); magY = mag(:,2); magZ = mag(:,3);

%% Hard Iron and Soft Iron Corrections
% The fit_ellipse function fits an ellipse to a dataset and returns the angle,
% scaling, and offset from the origin
ellipse = fit_ellipse(magX,magY)
phi = -ellipse.phi;
si_scale = ellipse.long_axis/ellipse.short_axis;
hix = ellipse.X0_in;
hiy = ellipse.Y0_in;

% Soft Iron Rotation Correction
Ca = [cos(phi) sin(phi);...
     -sin(phi) cos(phi)];
% Soft Iron Scaling Correction
Cb = [si_scale 0;0 1];
% Combined Soft Iron Correction
Cc = Cb*Ca;

% Calculate Corrected Magnetometer
B = transpose([magX-hix magY-hiy]);
magc = Cc*B;
magcX = magc(1,:);
magcY = magc(2,:);

% Plot original and corrected magnetometer data
figure(2)
hold on; grid on;
plot(magX,magY,'.')
plot(magcX,magcY,'.')
axis equal
title("Soft Iron and Hard Iron Magnetometer Correction")
xlabel("Magnetic Field, X (Gauss)")
ylabel("Magnetic Field, Y (Gauss)")
legend("Raw Magnetometer","Corrected Magnetometer")



%% Yaw Angle Comparison

% Yaw Angle from Corrected Magnetometer
magc_yaw = transpose(atan2(magcX,magcY));

% YAW FROM GYRO: take integral of yaw rate and wrap it to +-pi
yaw_rate = angVel(:,3);
yaw_angle = cumtrapz(time,yaw_rate);

% Complementary Filter with alpha value of 0.02
alpha = 0.02;
% correct yaw_angle using initial magnetometer estimate
compFilter = alpha*unwrap(magc_yaw) + (1-alpha)*(yaw_angle + magc_yaw(1));
filtered = wrapToPi(compFilter);

% plot gyro and mag yaw
figure(3)
hold on; grid on;
plot(time,wrapToPi(yaw_angle)*180/pi,'.')
plot(time,magc_yaw*180/pi,'.')
plot(time,head(10880:15000),'.')
plot(time,filtered*180/pi,'.')
title("Yaw Angle Comparison")
xlabel("time (s)")
ylabel("Yaw (degrees)")
legend("Gyro","Magnetometer","IMU EKS","Fused Gyro+Mag")

