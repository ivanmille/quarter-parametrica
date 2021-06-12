%%intro
% clear
close all

%% folder selection
addpath('analysis');
addpath('Parameters');
addpath('slprj');
addpath('Toolbox');
addpath('start model');
addpath(genpath('Post-Process'));

%% variable initialization

Structural_Parameters;
General_Parameters;
Graphical_Parameters;
Calculate_Dimensions_FL;
% Calculate_Dimensions_FR;
% Calculate_Dimensions_RL;
% Calculate_Dimensions_RR;
Elastic_Parameters;


