clc
clear all
close all

% Import Bag Files using ROS Toolbox - We don't need GPS Data Here
bagfile = rosbag('2022-03-04-09-21-00.bag');
magData = readMessages(select(bagfile,'Topic','/vn100_mag'),'DataFormat','struct');
magData = magData(10880:15000);
imuData = readMessages(select(bagfile,'Topic','/vn100_imu'),'DataFormat','struct');
imuData = imuData(10880:15000);

% create standard time variable
time = transpose(0:0.025:(4120*.025));

% IMU Data
mag = cell2mat(cellfun(@(m) [m.MagneticField_.X m.MagneticField_.Y m.MagneticField_.Z],magData,'UniformOutput',false));
angVel = cell2mat(cellfun(@(m) [m.AngularVelocity.X m.AngularVelocity.Y m.AngularVelocity.Z],imuData,'UniformOutput',false));
linAccel = cell2mat(cellfun(@(m) [m.LinearAcceleration.X m.LinearAcceleration.Y m.LinearAcceleration.Z],imuData,'UniformOutput',false));
imu_time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-imuData{1,1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),imuData);

% Store Mag
magX = mag(:,1); magY = mag(:,2); magZ = mag(:,3);

%% Hard Iron and Soft Iron Corrections from Part 1

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

% Final Filtered Yaw!
filtered = wrapToPi(compFilter);


%% Calculating Forward Velocity

% Repeating steps from part 2 to find forward velocity
imu_a = linAccel(:,1);
imu_a_mean = mean(imu_a);
imu_a_adjust = imu_a-imu_a_mean;
% Get IMU Velocity
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

% IMU Position
imu_p = cumtrapz(time,imu_v);

% W is the rotation rate in the vehicle frame
w = angVel(:,3);
wX_dot = w.*imu_v;
y_ddot = linAccel(:,2);


%% Calculating Sensor Offset in Vehicle Frame
yaw = filtered;

% do more with this? Use the equation from lab instructions?
imu_vE = -imu_v.*cos(yaw);
imu_vN = imu_v.*sin(yaw);
imu_pE = cumtrapz(time,imu_vE);
imu_pN = cumtrapz(time,imu_vN);

% Estimate IMU location relative to Vehicle CM
w = angVel(:,3);
w_dot = gradient(w)./gradient(time);
A = w_dot;
B = y_ddot(1:end) - wX_dot(1:end);
% Use linsolve to find AX = B
Xc = linsolve(A,B);
display(Xc)