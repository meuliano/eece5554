clc
clear all
close all

% Import Bag Files using ROS Toolbox and select gps topic
emBag = rosbag('imu_short.bag');
magBag = select(emBag,'Topic','/mag');
imuBag = select(emBag,'Topic','/imu');

% Format data as a Struct
magData = readMessages(magBag,'DataFormat','struct');
imuData = readMessages(imuBag,'DataFormat','struct');

% Store data as arrays for easting, northing, altitude, and time
mag = cell2mat(cellfun(@(m) [m.MagneticField_.X m.MagneticField_.Y m.MagneticField_.Z],magData,'UniformOutput',false));
quat = cell2mat(cellfun(@(m) [m.Orientation.X m.Orientation.Y m.Orientation.Z m.Orientation.W],imuData,'UniformOutput',false));
angVel = cell2mat(cellfun(@(m) [m.AngularVelocity.X m.AngularVelocity.Y m.AngularVelocity.Z],imuData,'UniformOutput',false));
linAccel = cell2mat(cellfun(@(m) [m.LinearAcceleration.X m.LinearAcceleration.Y m.LinearAcceleration.Z],imuData,'UniformOutput',false));
time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-imuData{1,1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),imuData);

% figure(1)
xyz = ["(X)","(Y)","(Z)","(\omega)"];
for i = 1:4
    figure(i)
    plot(time(1:end-500),quat(1:end-500,i),'k.')
    title("Quaternion Orientation "+xyz(i))
    xlabel("Time (s)")
    ylabel("Quaternion")
end

figure(2)
plot(time(1:end-1),mag,'.')
title("Magnetic Field")
xlabel("Time (s)")
ylabel("Magnetic Field (T)")

figure(3)
plot(time,angVel,'.')
title("Angular Velocity")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")

figure(4)
plot(time, quat,'.')
title("Quaternion")
xlabel("Time (s)")
ylabel("Quaternion Orientation")
