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
[roll0 pitch0 yaw0] = quat2angle([quat(:,4),quat(:,1),quat(:,2),quat(:,3)],'ZYX');

% There is a weird problem with the imu time... created my own time var
time = 0:0.025:(49422*.025);

% GPS Data
pos = cell2mat(cellfun(@(m) [m.UtmEasting m.UtmNorthing m.Alt],gpsData,'UniformOutput',false));
gps_time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-imuData{1,1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),gpsData);


%plot(time,smoothdata(linAccel(:,1),'gaussian',10000),'.')
% Get IMU Velocity
imu_a = linAccel(:,1);
imu_a_mean = (mean(imu_a(1:3500))+mean(imu_a(46000:end)))/2;
imu_a_adjust = imu_a-imu_a_mean;


% REMOVE STOPPED POINTS
a1 = smoothdata(imu_a,'gaussian',200);
a2 = gradient(a1);
for i = 1:length(a2)
    if abs(a2(i)) < 4*10^-4
        a11(i) = 0;
    else
        a11(i) = imu_a_adjust(i);
    end
end
figure(5)
plot(imu_a_adjust,'.')
hold on
plot(a11,'.')

imu_v = cumtrapz(time,imu_a_adjust);
imu_v1 = cumtrapz(time,a11);
% figure(6)
% hold on
% plot(imu_v)
% plot(imu_v1)


% if magnitude absolute value of next X values is within value
%if d 

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


% REMOVE STOPPED POINTS - GPS
X_dot = gps_v;
X_dot_resampled = interp(X_dot,40);
X_dot_resampled = X_dot_resampled(1:length(time));
% a1 = smoothdata(imu_a,'gaussian',200);
% a2 = gradient(a1);
for i = 1:length(X_dot_resampled)
    if abs(X_dot_resampled(i)) < 0.1
        val(i,2) = imu_a_adjust(i);
        val(i,1) = time(i);

        a12(i) = 0;
    else
        val(i,2) = NaN;
        val(i-200:i,2) = NaN;
        val(i,1) = time(i);
        a12(i) = imu_a_adjust(i);
    end
end
figure(7)
plot(imu_a_adjust,'.')
hold on
plot(a12,'.')

imu_v = cumtrapz(time,imu_a_adjust);
imu_v1 = cumtrapz(time,a11);
imu_v2 = cumtrapz(time,a12);
figure(6)
hold on
plot(imu_v)
plot(imu_v1)
plot(imu_v2)

[F,TF] = fillmissing((movmean(val(:,2),50)),'linear','SamplePoints',time);

% aaa = interp(val(:,2),time);

figure(8)
hold on
plot(val(:,2),'.')
plot(F)
% hold on
% smVal = smooth(val(2,i),val(1,i));
% plot(smVal)
aa = imu_a_adjust - F;
for i = 1:length(X_dot_resampled)
    if abs(X_dot_resampled(i)) < 0.1
        aa1(i) = 0;
    else
        aa1(i) = aa(i);
    end
end
vv = cumtrapz(time,aa1);
figure(6)
plot(vv)

aaa = highpass(imu_a,0.2);
vvv = cumtrapz(time,aaa);


% 
% % pos_sm = smoothdata(pos(:,1),'gaussian',20);
% %plot(time,smoothdata(linAccel(:,3),'gaussian',10000),'.')
% 
% % PITCH COMPENSATION
% pitch = smoothdata(pitch0,'gaussian',500);
% roll = smoothdata(roll0,'gaussian',500);
% 
% pitch_mean = (mean(pitch(1:3500))+mean(pitch(46000:end)))/2;
% pitch_adjust = pitch-pitch_mean;
% pitch_accel = sin(pitch_adjust)*-9.8;
% 
% 
% 
% figure(2)
% plot(pitch)
% hold on
% plot(pitch_adjust)
% 
% imu_a_mean = (mean(imu_a(1:3500))+mean(imu_a(46000:end)))/2;
% imu_a_adjust = imu_a-imu_a_mean;
% 
% imu_accel = imu_a_adjust+pitch_accel;
% 
% a1 = smoothdata(imu_accel,'gaussian',200);
% a2 = gradient(a1);
% for i = 1:length(a1)
%     if abs(a2(i)) < 4*10^-4
%         a11(i) = 0;
%     else
%         a11(i) = imu_accel(i);
%     end
% end
% figure(3)
% plot(imu_accel)
% hold on
% plot(a11)
% 
% % imu_accel = imu_a+pitch_adjust;
% imu_vel = cumtrapz(time,imu_accel);
% imu_vel2 = cumtrapz(time,a11);
% figure(4)
% plot(imu_vel)
% hold on
% plot(imu_vel2)
