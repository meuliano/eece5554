clc
clear all

% Remove scientific notation
format longG

% Import Bag Files using ROS Toolbox
stillBag = rosbag('2022-02-07-13-58-34.bag');
walkBag = rosbag('2022-02-07-14-10-04.bag');

% Select the data from the GPS topic
stillSel = select(stillBag, 'Topic','/gps');
walkSel = select(walkBag, 'Topic','/gps');

% Format data as a Struct
stillMsgStructs = readMessages(stillSel,'DataFormat','struct');
walkMsgStructs = readMessages(walkSel,'DataFormat','struct');

% Store data as arrays for easting, northing, altitude, and time
stillEastingPoints = cellfun(@(m) double(m.UtmEasting),stillMsgStructs);
stillNorthingPoints = cellfun(@(m) double(m.UtmNorthing),stillMsgStructs);
stillAltitudePoints = cellfun(@(m) double(m.Altitude),stillMsgStructs);
stillTimePoints = cellfun(@(m) double(m.Header.Stamp.Sec),stillMsgStructs);
walkEastingPoints = cellfun(@(m) double(m.UtmEasting),walkMsgStructs);
walkNorthingPoints = cellfun(@(m) double(m.UtmNorthing),walkMsgStructs);
walkAltitudePoints = cellfun(@(m) double(m.Altitude),walkMsgStructs);
walkTimePoints = cellfun(@(m) double(m.Header.Stamp.Sec),walkMsgStructs);

% Zero Easting, Northing, and Time for visual clarity
stillEastingPoints1 = stillEastingPoints-min(stillEastingPoints);
stillNorthingPoints1 = stillNorthingPoints-min(stillNorthingPoints);
stillTimePoints1 = stillTimePoints - min(stillTimePoints);
walkEastingPoints1 = walkEastingPoints-min(walkEastingPoints);
walkNorthingPoints1 = walkNorthingPoints-min(walkNorthingPoints);
walkTimePoints1 = walkTimePoints - min(walkTimePoints);

% Plot
figure(1)
scatterhist(stillEastingPoints1, stillNorthingPoints1)
grid on
title("Stationary GPS UTM Recording")
xlabel("Easting (m) zeroed at " + round(min(stillEastingPoints)) + " m")
ylabel("Northing (m) zeroed at " + round(min(stillNorthingPoints)) + " m")

figure(2)
plot(walkEastingPoints1, walkNorthingPoints1, 'x')
grid on
title("Straight-Line Walking GPS UTM Recording")
xlabel("Easting (m) zeroed at " + round(min(walkEastingPoints)) + " m")
ylabel("Northing (m) zeroed at " + round(min(walkNorthingPoints)) + " m")

figure(3)
plot(stillTimePoints1,stillAltitudePoints, 'x')
grid on
title("Stationary GPS Altitude Recording")
xlabel("Time (s)")
ylabel("Altitude (m)")

figure(4)
plot(walkTimePoints1,walkAltitudePoints, 'x')
grid on
title("Straight-Line Walking GPS Altitude Recording")
xlabel("Time (s)")
ylabel("Altitude (m)")
