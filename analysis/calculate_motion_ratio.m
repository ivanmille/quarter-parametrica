function [motion_ratio,static_force,equilibrium_position] = calculate_motion_ratio(dx)

% calculates the motion ratio given a pushrod x-offset dx


quarter_cinematica;

model = 'Cinematica';

load_system(model);

save_system(model);

n_sim               =   numel(dx);       %number of simulation


%% simulate each set

simIn(1:n_sim) = Simulink.SimulationInput(model);                    % pre-allocates simIn memory 


tic
for ii = 1:numel(dx)
           
          Smi = move_dx(dx(ii));                               % modifies the x coordinates by dx(ii)
          
          Smi.general.jounce_mode     =  1;           %jounce ramp mode
          Smi.general.simulation_time =  1;           %[s]
          Smi.general.ramp_slope      =  50;          %[mm/s]

          simIn(ii)=simIn(ii).setVariable('Smi' , Smi);        % imports the modified coordinates in the simulation
          
          disp(['ciclo ' num2str(ii) ' di ' num2str(n_sim)])   % GUI
end
toc

tic
disp('------------- calculating MOTION RATIOS -------------')
out = parsim(simIn,'TransferBaseWorkspaceVariables','on');     % parallel simulation
disp('------------- MOTION RATIOS calculated -------------')
toc

%% output calculation

motion_ratio           = zeros(1,numel(dx));
static_force           = zeros(1,numel(dx));
equilibrium_position   = zeros(1,numel(dx));

for mm = 1:numel(dx)
    motion_ratio(1,mm)              =   Smi.general.simulation_time*Smi.general.ramp_slope/(abs(max(out(1,mm).FL_damper_excursion.data)- min(out(1,mm).FL_damper_excursion.data)));
    static_force(1,mm)              =   Smi.Car.FRONT.LEFT.static_mass*9.81/motion_ratio(1,mm);
    equilibrium_position(1,mm)      =   static_force(1,mm)/(Smi.Car.FRONT.elastic.ammo_spring_stiffness/1000);   %[mm]
end

close_system(model);

end

