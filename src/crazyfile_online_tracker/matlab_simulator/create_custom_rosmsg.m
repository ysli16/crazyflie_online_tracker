% This scripts generates the customized ros messages/services/actions so
% that they are accessible to matlab.

% path to the ros package
folderpath = '/home/ysli/Desktop/MT/catkin_ws/src';
% generate all ros messages defined in package/msg(The process may take a few minutes.)
rosgenmsg(folderpath);
% Add matlab path and refresh all class defination
addpath('/home/ysli/Desktop/MT/catkin_ws/src/matlab_msg_gen_ros1/glnxa64/install/m')
savepath
clear classes
rehash toolboxcache