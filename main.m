
clear; close all; clc;
global robot
global lineMapLoc
global mapPose
global MAP_LOCALIZATION
MAP_LOCALIZATION = true;
mapPose = pose(0.22,0.22, -pi/2);
robot = raspbot();
robot.stopLaser();
pause(1.)
robot.startLaser();
pause(1.)
%define map
OFFSET = 0.0;%0.308;
WIDTH  = 1.23;
p1 = [0; 0]-OFFSET;
p2 = [WIDTH; 0]-OFFSET;
p3 = [0; WIDTH]-OFFSET;
p4 = [WIDTH; WIDTH]-OFFSET;

lines_p1 = [p1 p1 p2];
lines_p2 = [p2 p3 p4];

gain = 0.3;
errThresh = 0.00001;
gradThresh = 0.0005;

lineMapLoc = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);

robot.laser.NewMessageFcn=@laserEventListener;

pause(3.)

vmax = 0.15;
mrpl = mrplSystem(robot, true);
pause(3.);
xp = 0.3048; yp = 0.3048*3; thp = -pi/2;
obj_pos =  [xp; yp; thp];
xd = 0.5334; yd = 0.3048/2+0.1;   thd = -pi/2;
 
drop_pos = [xd; yd; thd];
%mrpl.turnRelAngle(pi/2, 0);
mrpl.pickDropObject(obj_pos,drop_pos, vmax, 1, 1)
pause(3.)
xp = 0.3048*2; yp = 0.3048*3; thp = -pi/2;
obj_pos =  [xp; yp; thp];
xd = 0.5334 + 0.3048/2; yd = 0.3048/2+0.1;thd = -pi/2;
drop_pos = [xd; yd; thd];
mrpl.pickDropObject(obj_pos,drop_pos, vmax, 1, 1)
 
pause(3.)
xp = 0.3048*3; yp = 0.3048*3; thp = -pi/2;
obj_pos =  [xp; yp; thp];
xd = 0.5334 + 0.3048; yd = 0.3048/2 + 0.1;thd = -pi/2;
drop_pos = [xd; yd; thd];
%mrpl.turnRelAngle(pi/2, 0);
mrpl.pickDropObject(obj_pos,drop_pos, vmax, 1, 1)
