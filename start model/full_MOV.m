%%intro
clear
clc
close all

%% folder selection
addpath('analysis');
addpath('Parameters');
addpath('slprj');
addpath('Toolbox');


%% initialization
DATA;

open_system('Parametrica');

%% selecting necessary subsystems.
set_param('Parametrica/FIXED CHASSIS','Commented','on');
set_param('Parametrica/MOVING CHASSIS','Commented','off');

set_param('Parametrica/MOVING CHASSIS/BODY','Commented','off');
set_param('Parametrica/MOVING CHASSIS/BODY/FL HARDPOINTS','Commented','off');
set_param('Parametrica/MOVING CHASSIS/BODY/FR HARDPOINTS','Commented','off');
set_param('Parametrica/MOVING CHASSIS/BODY/RL HARDPOINTS','Commented','off');
set_param('Parametrica/MOVING CHASSIS/BODY/RR HARDPOINTS','Commented','off');
set_param('Parametrica/MOVING CHASSIS/BODY/Monocoque','Commented','off');


set_param('Parametrica/MOVING CHASSIS/FL SUSPENSION','Commented','off');
set_param('Parametrica/MOVING CHASSIS/FR SUSPENSION','Commented','off');
set_param('Parametrica/MOVING CHASSIS/RL SUSPENSION','Commented','off');
set_param('Parametrica/MOVING CHASSIS/RR SUSPENSION','Commented','off');

set_param('Parametrica/MOVING CHASSIS/ACTUATOR FL','Commented','off');
set_param('Parametrica/MOVING CHASSIS/ACTUATOR FR','Commented','off');
set_param('Parametrica/MOVING CHASSIS/ACTUATOR RL','Commented','off');
set_param('Parametrica/MOVING CHASSIS/ACTUATOR RR','Commented','off');

set_param('Parametrica/FIXED CHASSIS/ARB FORCE FRONT','Commented','off');
set_param('Parametrica/FIXED CHASSIS/ARB FORCE REAR','Commented','off');