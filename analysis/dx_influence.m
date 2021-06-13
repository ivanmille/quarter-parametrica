% Calculates the influence of dx on tire forces


%% Intro
clear
close all;
clc

%% sweep parameters

dx                  =   0:1:1;                                  %range of steering angle tested [deg]
imposed_steering    =   0:80:80;                              %range of steering angle tested [deg]

%% motion ratios

[motion_ratio,static_force,equilibrium_position] = calculate_motion_ratio(dx);


%% model parameters

% half_FIX;
quarter_FIX;

model = 'Parametrica';

load_system(model);

save_system(model);

Smi.general.simulation_time =  1;           %[s]
dt                          =  0.001;       %[s]


% sweep parameters


Rack_initial        =   imposed_steering*Smi.Car.FRONT.LEFT.rack_ratio;

n_sim               =   numel(imposed_steering)*numel(dx);       %number of simulation
n_steps             =   1+Smi.general.simulation_time/dt;

% jounce              =   zeros(n_sim,n_steps);
% steering            =   zeros(n_sim,n_steps);     %pre-allocating memory
% force               =   zeros(n_sim,n_steps);
% displacement        =   zeros(n_sim,n_steps);

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


