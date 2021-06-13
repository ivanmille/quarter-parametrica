%% folder selection
addpath('analysis');
addpath('Parameters');
addpath('slprj');
addpath('Toolbox');


%% initialization
DATA;

open_system('Parametrica');

%% selecting necessary subsystems
set_param('Parametrica/FIXED CHASSIS','Commented','off');
set_param('Parametrica/MOVING CHASSIS','Commented','on');

set_param('Parametrica/FIXED CHASSIS/BODY','Commented','off');
set_param('Parametrica/FIXED CHASSIS/BODY/FL HARDPOINTS','Commented','off');
set_param('Parametrica/FIXED CHASSIS/BODY/FR HARDPOINTS','Commented','off');
set_param('Parametrica/FIXED CHASSIS/BODY/RL HARDPOINTS','Commented','on');
set_param('Parametrica/FIXED CHASSIS/BODY/RR HARDPOINTS','Commented','on');
set_param('Parametrica/FIXED CHASSIS/BODY/Monocoque','Commented','on');

set_param('Parametrica/FIXED CHASSIS/FL SUSPENSION','Commented','off');
set_param('Parametrica/FIXED CHASSIS/FR SUSPENSION','Commented','off');
set_param('Parametrica/FIXED CHASSIS/RL SUSPENSION','Commented','on');
set_param('Parametrica/FIXED CHASSIS/RR SUSPENSION','Commented','on');

set_param('Parametrica/FIXED CHASSIS/ACTUATOR FL','Commented','off');
set_param('Parametrica/FIXED CHASSIS/ACTUATOR FR','Commented','off');
set_param('Parametrica/FIXED CHASSIS/ACTUATOR RL','Commented','on');
set_param('Parametrica/FIXED CHASSIS/ACTUATOR RR','Commented','on');

set_param('Parametrica/FIXED CHASSIS/ARB FORCE FRONT','Commented','off');
set_param('Parametrica/FIXED CHASSIS/ARB FORCE REAR','Commented','on');