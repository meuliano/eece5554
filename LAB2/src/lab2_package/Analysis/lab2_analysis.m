clc
clear all

% Remove scientific notation
format longG

roofBag = rosbag('columbus_moving.bag');
roofMove_t = roofBag.MessageList.Time(3:end);

% Import Bag Files using ROS Toolbox and select gps topic
roofStillBag = select(rosbag('2022-02-16-17-12-02.bag'),'Topic','/gps');
roofMoveBag = select(rosbag('columbus_moving.bag'),'Topic','/GNSS');
groundStillBag = select(rosbag('2022-02-16-17-36-15.bag'),'Topic','/gps');
groundMoveBag = select(rosbag('2022-02-16-17-48-00.bag'),'Topic','/gps');

% Format data as a Struct
roofStill = readMessages(roofStillBag,'DataFormat','struct');
roofMove = readMessages(roofMoveBag,'DataFormat','struct');
groundStill = readMessages(groundStillBag,'DataFormat','struct');
groundMove = readMessages(groundMoveBag,'DataFormat','struct');

% Store data as arrays for easting, northing, altitude, and time
roofStill_E = cellfun(@(m) double(m.UtmEasting),roofStill);
roofStill_N = cellfun(@(m) double(m.UtmNorthing),roofStill);
roofStill_A = cellfun(@(m) double(m.Alt),roofStill);
roofStill_t = cellfun(@(m) double(m.Header.Stamp.Sec),roofStill);
roofStill_Q = cellfun(@(m) double(m.Fix),roofStill);

roofMove_E = cellfun(@(m) double(m.UtmEasting),roofMove);
roofMove_N = cellfun(@(m) double(m.UtmNorthing),roofMove);
roofMove_A = cellfun(@(m) double(m.Altitude),roofMove);
% roofMove_t = cellfun(@(m) double(m.Header.Stamp.Sec),roofMove);
roofMove_Q = cellfun(@(m) double(m.Quality),roofMove);

groundStill_E = cellfun(@(m) double(m.UtmEasting),groundStill);
groundStill_E = groundStill_E(500:end);
groundStill_N = cellfun(@(m) double(m.UtmNorthing),groundStill);
groundStill_N = groundStill_N(500:end);
groundStill_A = cellfun(@(m) double(m.Alt),groundStill);
groundStill_A = groundStill_A(500:end);
groundStill_t = cellfun(@(m) double(m.Header.Stamp.Sec),groundStill);
groundStill_t = groundStill_t(500:end);
groundStill_Q = cellfun(@(m) double(m.Fix),groundStill);
groundStill_Q = groundStill_Q(500:end);

groundMove_E = cellfun(@(m) double(m.UtmEasting),groundMove);
groundMove_N = cellfun(@(m) double(m.UtmNorthing),groundMove);
groundMove_A = cellfun(@(m) double(m.Alt),groundMove);
groundMove_t = cellfun(@(m) double(m.Header.Stamp.Sec),groundMove);
groundMove_Q = cellfun(@(m) double(m.Fix),groundMove);

figure(1)
sgtitle("Stationary Recording - Clear")
idx = roofStill_Q == 4;
subplot(2,2,1)
plot(roofStill_E(idx)-min(roofStill_E), roofStill_N(idx)-min(roofStill_N), 'b.', ...
    roofStill_E(~idx)-min(roofStill_E), roofStill_N(~idx)-min(roofStill_N), 'r.')
grid on
title("Easting vs. Northing")
xlabel("Easting (m) from " + round(min(roofStill_E)))
ylabel("Northing (m) from " + round(min(roofStill_N)))
subplot(2,2,2)
plot(roofStill_t(idx)-min(roofStill_t), roofStill_A(idx), 'b.', ...
    roofStill_t(~idx)-min(roofStill_t), roofStill_A(~idx), 'r.')
grid on
title("Altitude")
xlabel("Time (s)")
ylabel("Altitude (m)")
subplot(2,2,3)
plot(roofStill_t(idx)-min(roofStill_t), roofStill_N(idx)-min(roofStill_N), 'b.', ...
    roofStill_t(~idx)-min(roofStill_t), roofStill_N(~idx)-min(roofStill_N), 'r.')
grid on
title("Northing")
xlabel("Time (s)")
ylabel("Northing (m) from " + round(min(roofStill_N)))
subplot(2,2,4)
plot(roofStill_t(idx)-min(roofStill_t), roofStill_E(idx)-min(roofStill_E), 'b.', ...
    roofStill_t(~idx)-min(roofStill_t), roofStill_E(~idx)-min(roofStill_E), 'r.')
grid on
title("Easting")
xlabel("Time (s)")
ylabel("Easting (m) from " + round(min(roofStill_E)))

figure(2)
sgtitle("Moving Recording - Clear")
idx = roofMove_Q == 4;
subplot(2,2,1)
plot(roofMove_E(idx)-min(roofMove_E), roofMove_N(idx)-min(roofMove_N), 'b.', ...
    roofMove_E(~idx)-min(roofMove_E), roofMove_N(~idx)-min(roofMove_N), 'r.')
grid on
title("Easting vs. Northing")
xlabel("Easting (m) from " + round(min(roofMove_E)))
ylabel("Northing (m) from " + round(min(roofMove_N)))
subplot(2,2,2)
plot(roofMove_t(idx)-min(roofMove_t), roofMove_A(idx), 'b.', ...
    roofMove_t(~idx)-min(roofMove_t), roofMove_A(~idx), 'r.')
grid on
title("Altitude")
xlabel("Time (s)")
ylabel("Altitude (m)")
subplot(2,2,3)
plot(roofMove_t(idx)-min(roofMove_t), roofMove_N(idx)-min(roofMove_N), 'b.', ...
    roofMove_t(~idx)-min(roofMove_t), roofMove_N(~idx)-min(roofMove_N), 'r.')
grid on
title("Northing")
xlabel("Time (s)")
ylabel("Northing (m) from " + round(min(roofMove_N)))
subplot(2,2,4)
plot(roofMove_t(idx)-min(roofMove_t), roofMove_E(idx)-min(roofMove_E), 'b.', ...
    roofMove_t(~idx)-min(roofMove_t), roofMove_E(~idx)-min(roofMove_E), 'r.')
grid on
title("Easting")
xlabel("Time (s)")
ylabel("Easting (m) from " + round(min(roofMove_E)))

figure(3)
sgtitle("Stationary Recording - Occluded")
idx = groundStill_Q == 4;
subplot(2,2,1)
plot(groundStill_E(idx)-min(groundStill_E), groundStill_N(idx)-min(groundStill_N), 'b.', ...
    groundStill_E(~idx)-min(groundStill_E), groundStill_N(~idx)-min(groundStill_N), 'r.')
grid on
title("Easting vs. Northing")
xlabel("Easting (m) from " + round(min(groundStill_E)))
ylabel("Northing (m) from " + round(min(groundStill_N)))
subplot(2,2,2)
plot(groundStill_t(idx)-min(groundStill_t), groundStill_A(idx), 'b.', ...
    groundStill_t(~idx)-min(groundStill_t), groundStill_A(~idx), 'r.')
grid on
title("Altitude")
xlabel("Time (s)")
ylabel("Altitude (m)")
subplot(2,2,3)
plot(groundStill_t(idx)-min(groundStill_t), groundStill_N(idx)-min(groundStill_N), 'b.', ...
    groundStill_t(~idx)-min(groundStill_t), groundStill_N(~idx)-min(groundStill_N), 'r.')
grid on
title("Northing")
xlabel("Time (s)")
ylabel("Northing (m) from " + round(min(groundStill_N)))
subplot(2,2,4)
plot(groundStill_t(idx)-min(groundStill_t), groundStill_E(idx)-min(groundStill_E), 'b.', ...
    groundStill_t(~idx)-min(groundStill_t), groundStill_E(~idx)-min(groundStill_E), 'r.')
grid on
title("Easting")
xlabel("Time (s)")
ylabel("Easting (m) from " + round(min(groundStill_E)))

figure(4)
sgtitle("Moving Recording - Occluded")
idx = groundMove_Q == 4;
subplot(2,2,1)
plot(groundMove_E(idx)-min(groundMove_E), groundMove_N(idx)-min(groundMove_N), 'b.', ...
    groundMove_E(~idx)-min(groundMove_E), groundMove_N(~idx)-min(groundMove_N), 'r.')
grid on
title("Easting vs. Northing")
xlabel("Easting (m) from " + round(min(groundMove_E)))
ylabel("Northing (m) from " + round(min(groundMove_N)))
subplot(2,2,2)
plot(groundMove_t(idx)-min(groundMove_t), groundMove_A(idx), 'b.', ...
    groundMove_t(~idx)-min(groundMove_t), groundMove_A(~idx), 'r.')
grid on
title("Altitude")
xlabel("Time (s)")
ylabel("Altitude (m)")
subplot(2,2,3)
plot(groundMove_t(idx)-min(groundMove_t), groundMove_N(idx)-min(groundMove_N), 'b.', ...
    groundMove_t(~idx)-min(groundMove_t), groundMove_N(~idx)-min(groundMove_N), 'r.')
grid on
title("Northing")
xlabel("Time (s)")
ylabel("Northing (m) from " + round(min(groundMove_N)))
subplot(2,2,4)
plot(groundMove_t(idx)-min(groundMove_t), groundMove_E(idx)-min(groundMove_E), 'b.', ...
    groundMove_t(~idx)-min(groundMove_t), groundMove_E(~idx)-min(groundMove_E), 'r.')
grid on
title("Easting")
xlabel("Time (s)")
ylabel("Easting (m) from " + round(min(groundMove_E)))
