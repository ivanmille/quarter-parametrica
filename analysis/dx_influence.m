% Calculates the influence of dx on tire forces


%% Intro
clear
close all;
clc

%% sweep parameters

dx_initial          =   0;                                      %[mm]
dx_final            =   1;                                      %[mm]
dx_step             =   1;                                      %[mm]
dx                  =   dx_initial:dx_step:dx_final;            %range of steering angle tested [deg]
n_dx                =   1+(dx_final-dx_initial)/dx_step;

steering_initial    =   0;                                                      %[deg]
steering_final      =   40;                                                       %[deg]
steering_step       =   40;                                                       %[deg]
imposed_steering    =   steering_initial:steering_step:steering_final;            %range of steering angle tested [deg]
n_steering          =   1+(steering_final-steering_initial)/steering_step;

%% motion ratios

[motion_ratio,static_force,equilibrium_position] = calculate_motion_ratio(dx);


%% model parameters

% half_FIX;
quarter_FIX;

model = 'Parametrica';

load_system(model);

save_system(model);

Smi.general.jounce_mode     =  1;           %jounce ramp mode
Smi.general.simulation_time =  1;           %[s]
Smi.general.ramp_slope      =  30;          %[mm/s]
dt                          =  0.001;       %[s]

min_jounce                  =  -Smi.general.ramp_slope/2;
max_jounce                  =  Smi.general.ramp_slope/2;

% sweep parameters


Rack_initial        =   imposed_steering*Smi.Car.FRONT.LEFT.rack_ratio;

n_sim               =   numel(imposed_steering)*numel(dx);       %number of simulation
n_steps             =   1+Smi.general.simulation_time/dt;

jounce              =   zeros(n_sim,n_steps);
steering            =   zeros(n_sim,n_steps);     %pre-allocating memory
force               =   zeros(n_sim,n_steps);
displacement        =   zeros(n_sim,n_steps);

%% simulate each set

simIn(1:n_sim) = Simulink.SimulationInput(model);                    % pre-allocates simIn memory 
simIn(:) = simIn(:).setModelParameter('SolverType', 'Fixed-step');
simIn(:) = simIn(:).setModelParameter('FixedStep' ,  num2str(dt));

tic
for ii = 1:numel(dx)
    
    for jj = 1:numel(imposed_steering)
          
          kk=((ii-1)*numel(imposed_steering)+jj);              % sequential index regardless of the presence of two for cycles

          Smi = move_dx(dx(ii));                               % modifies the x coordinates by dx(ii)
          
          Smi.general.jounce_mode     =  1;           %jounce ramp mode
          Smi.general.simulation_time =  1;           %[s]
          Smi.general.ramp_slope      =  30;          %[mm/s]
          
          Smi.Car.FRONT.elastic.ammo_motion_ratio         =    motion_ratio(ii);             %[N*s/mm]
          Smi.Car.FRONT.elastic.ammo_equilibrium_position =    equilibrium_position(ii);     %[N*s/mm]
          Smi.Car.FRONT.elastic.ammo_static_force         =    static_force(ii);             %[N]
          
          Smi.Car.FRONT.LEFT.Dimension.Rack_initial = Rack_initial(jj) ;
          
          
          simIn(kk)=simIn(kk).setVariable('Smi' , Smi);        % imports the modified coordinates in the simulation
          
          
          disp(['ciclo ' num2str(kk) ' di ' num2str(n_sim)])   % GUI
          
    end
end
toc


tic
disp('------------- calculating dx influence -------------')
out = parsim(simIn,'TransferBaseWorkspaceVariables','on');     % parallel simulation
disp('------------- dx influence calculated -------------')
toc


%% data processing

hh=1;
kk=1;
jj=1;

for ii=1:n_sim          %creates matrix A(i,j) where i is the simulation number and j the value at the j-step  
    
    jounce(ii,:)    = (out(1,ii).FL_jounce_pos.Data)'; 
    force(ii,:)    = (out(1,ii).FL_WHEEL_LOAD.Data)';
    
    if (n_steering-kk+1)~=0
       displacement(ii,:)  = linspace(dx(hh),dx(hh),n_steps);
       kk=kk+1;
    else
        kk=2;
        hh=hh+1;
        displacement(ii,:)  = linspace(dx(hh),dx(hh),n_steps);
    end
    
    if (n_steering-jj+1)~=0
        steering(ii,:)  = linspace(imposed_steering(jj),imposed_steering(jj),n_steps);
        jj=jj+1;
    else
        jj=2;
        steering(ii,:)  = linspace(imposed_steering(1),imposed_steering(1),n_steps);
    end
    
end


%% post process 

text_dx=uicontrol('style','text','String','dx','Position',[502,7,60,18]);
slider_dx=uicontrol('style','slider','InnerPosition',[75,4,432,22],'min',dx_initial,'max',dx_final,'SliderStep',[dx_step/(dx_final-dx_initial), dx_step*10/(dx_final-dx_initial)]);

text_steering=uicontrol('style','text','String','steering','Position',[12,395,60,20]);
slider_steering=uicontrol('style','slider','InnerPosition',[76,395,432,22],'min',steering_initial,'max',steering_final,'SliderStep',[steering_step/(steering_final-steering_initial), steering_step*10/(steering_final-steering_initial)]);

stop_UI=uicontrol('style','pushbutton','InnerPosition',[4.4,3,60,25],'String','close','Callback',@stop_button);

dxListener       = addlistener(slider_dx      , 'Value', 'PostSet', @plot_jounce_force);
steeringListener = addlistener(slider_steering, 'Value', 'PostSet', @plot_jounce_force);







