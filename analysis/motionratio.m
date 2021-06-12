% calculates the motion ratio given a pushrod x-offset dx



% half_FIX;
quarter_FIX;

model = 'Cinematica';

load_system(model);

save_system(model);

Smi.general.jounce_mode     =  1;           %jounce ramp mode
Smi.general.simulation_time =  1;           %[s]
Smi.general.ramp_slope      =  50;          %[mm/s]
dt                          =  0.001;       %[s]
dx                  =   0:1:10; 
% dx                  =   0:1:70; 


% sweep parameters

n_sim               =   numel(dx);       %number of simulation
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
for ll = 1:numel(dx)
   
          
              
          
          Smi = move_dx(dx(ll));                               % modifies the x coordinates by dx(ii)
          simIn(ll)=simIn(ll).setVariable('Smi' , Smi);        % imports the modified coordinates in the simulation
          
          disp(['ciclo ' num2str(ll) ' di ' num2str(n_sim)])   % GUI
end
toc

tic
out = parsim(simIn,'TransferBaseWorkspaceVariables','on');     % parallel simulation
toc
%% 
motion_ratio = zeros(1,numel(dx));
pos_stat = zeros(1,numel(dx));
forza_stat = zeros(1,numel(dx));
for mm = 1:numel(dx)
    motion_ratio(1,mm) = 50/(abs(max(out(1,mm).damper_excursion.data)- min(out(1,mm).damper_excursion.data)));
    forza_stat(1,mm) = (65.5*9.81)/motion_ratio(1,mm);
    pos_stat(1,mm) = forza_stat(1,mm)/52.538;
end
