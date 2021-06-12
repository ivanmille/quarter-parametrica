% Calculates the influence of steering angle and jounce on camber


%% Intro

quarter_FIX;

model = 'Parametrica';

load_system(model);

save_system(model);

Smi.general.jounce_mode     = 1;           %jounce ramp mode
Smi.general.simulation_time = 1;
Smi.general.ramp_slope      = 50;
dt                          =  0.001;     %[s]


% sweep parameters

dx                  =   0:1:70;                                  %range of steering angle tested [deg]
imposed_steering    =   -160:5:160;                              %range of steering angle tested [deg]

n_sim               =   numel(imposed_steering)*numel(dx);                 %number of simulation



%% simulate each set

simIn(1:n_sim) = Simulink.SimulationInput(model);
% simIn(:) = simIn(:).setModelParameter('SolverType', 'Fixed-step');
% simIn(:) = simIn(:).setModelParameter('FixedStep' ,  num2str(dt));

tic
for ii = 1:numel(dx)
    for jj = 1:numel(imposed_steering)
          
          kk=((ii-1)*numel(imposed_steering)+jj);
            
          Smi = move_dx(dx(ii));
          simIn(kk)=simIn(kk).setVariable('Smi' , Smi);
          
          disp(['ciclo ' num2str(kk) ' di ' num2str(n_sim)])
    end
end
toc

tic
out = parsim(simIn,'TransferBaseWorkspaceVariables','on');
toc
