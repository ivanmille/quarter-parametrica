% Calculates the influence of steering angle and jounce on camber


%% Intro

% half_FIX;
quarter_FIX;

model = 'Parametrica';

load_system(model);

save_system(model);

Smi.general.jounce_mode     =  1;           %jounce ramp mode
Smi.general.simulation_time =  1;           %[s]
Smi.general.ramp_slope      =  50;          %[mm/s]
dt                          =  0.001;       %[s]


% sweep parameters

% dx                  =   0:1:70;                                  %range of steering angle tested [deg]
% imposed_steering    =   -160:5:160;                              %range of steering angle tested [deg]

dx                  =   0:1:10;                                  %range of steering angle tested [deg]
imposed_steering    =   -160:80:160;                              %range of steering angle tested [deg]

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
          Smi.Car.FRONT.LEFT.Dimension.Rack_initial = imposed_steering(jj)*0.2065 ;
          simIn(kk)=simIn(kk).setVariable('Smi' , Smi);        % imports the modified coordinates in the simulation
          
          disp(['ciclo ' num2str(kk) ' di ' num2str(n_sim)])   % GUI
    end
end
toc

tic
out = parsim(simIn,'TransferBaseWorkspaceVariables','on');     % parallel simulation
toc


