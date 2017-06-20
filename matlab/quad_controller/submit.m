close all;
clear;

addpath('utils');
addpath('../../utilitycode');

% This is your controller
controlhandle = @controller;

% This is your trajectory generator
trajhandle = @traj_generator;
waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0]';
trajhandle([],[],waypoints);

evaluate(controlhandle, trajhandle);
