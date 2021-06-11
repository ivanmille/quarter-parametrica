% Calculates the influence of steering angle and jounce on camber


%% Intro

quarter_FIX;

model = 'Parametrica';

load_system(model);

open_system(model);

save_system(model);

Smi.general.jounce_mode = 1; %jounce ramp mode

% sweep parameters

dt                  =   0.001;                                   %[s]
% dx                  =   0:1:70;                                  %range of steering angle tested [deg]
% imposed_steering    =   -150:5:150;                              %range of steering angle tested [deg]
dx                  =   0:1:1;                                  %range of steering angle tested [deg]
imposed_steering    =   -5:5:5;                              %range of steering angle tested [deg]
imposed_rack        =   Smi.Car.FRONT.LEFT.rack_ratio*imposed_steering;      %ratio corrected input   [mm]
n_sim               =   numel(imposed_steering)*numel(dx);                 %number of simulation

% jounce              =   zeros(n_sim,1001);
% steering            =   zeros(n_sim,1001);     %pre-allocating memory
% camber              =   zeros(n_sim,1001);


%% simulate each set

simIn(1:n_sim) = Simulink.SimulationInput(model);
% simIn(:) = simIn(:).setModelParameter('SolverType', 'Fixed-step');
% simIn(:) = simIn(:).setModelParameter('FixedStep' ,  num2str(dt));

for ii = 1:numel(dx)
    for jj = 1:numel(imposed_steering)
          simIn((ii-1)*numel(imposed_steering)+jj) = simIn((ii-1)*numel(imposed_steering)+jj).setBlockParameter([model '/FIXED CHASSIS/BODY/FL HARDPOINTS/IMPOSED STEERING'], 'TranslationStandardOffset', num2str(imposed_rack(jj)));
        
          crank = [Smi.Car.FRONT.LEFT.CRANK_CENTEREVOLUTE(1)+dx(ii),Smi.Car.FRONT.LEFT.CRANK_CENTEREVOLUTE(2),Smi.Car.FRONT.LEFT.CRANK_CENTEREVOLUTE(3)];
          simIn((ii-1)*numel(imposed_steering)+jj) = simIn((ii-1)*numel(imposed_steering)+jj).setBlockParameter([model '/FIXED CHASSIS/BODY/FL HARDPOINTS/crank'], 'TranslationCartesianOffset', ['[' num2str(crank) ']']);
          
          damper = [Smi.Car.FRONT.LEFT.DAMPER_CENTEREVOLUTE(1)+dx(ii),Smi.Car.FRONT.LEFT.DAMPER_CENTEREVOLUTE(2),Smi.Car.FRONT.LEFT.DAMPER_CENTEREVOLUTE(3)];
          simIn((ii-1)*numel(imposed_steering)+jj) = simIn((ii-1)*numel(imposed_steering)+jj).setBlockParameter([model '/FIXED CHASSIS/BODY/FL HARDPOINTS/damper'], 'TranslationCartesianOffset', ['[' num2str(damper) ']']);
          
          arb = [Smi.Car.FRONT.LEFT.ARB_CENTER(ii)+dx(ii),Smi.Car.FRONT.LEFT.ARB_CENTER(2),Smi.Car.FRONT.LEFT.ARB_CENTER(3)];
          simIn((ii-1)*numel(imposed_steering)+jj) = simIn((ii-1)*numel(imposed_steering)+jj).setBlockParameter([model '/FIXED CHASSIS/BODY/FL HARDPOINTS/arb'], 'TranslationCartesianOffset', ['[' num2str(arb) ']']);
          
          Smi.Car.FRONT.LEFT.UW_PUSHROD = [Smi.Car.FRONT.LEFT.UW_PUSHROD(1)+dx(ii) Smi.Car.FRONT.LEFT.UW_PUSHROD(2) Smi.Car.FRONT.LEFT.UW_PUSHROD(3)];
          Calculate_Dimensions_FL;
          
          uw_push = [Smi.Car.FRONT.LEFT.Dimension.plane_UW.PS_t_coord-Smi.Car.FRONT.LEFT.Dimension.plane_UW.A_t_coord];
          simIn((ii-1)*numel(imposed_steering)+jj) = simIn((ii-1)*numel(imposed_steering)+jj).setBlockParameter([model '/FIXED CHASSIS/FL SUSPENSION/UPPER WISHBONE/A-PS'], 'TranslationCartesianOffset', 'uw_push');
    end
end

tic
out = parsim(simIn,'TransferBaseWorkspaceVariables','on');
toc

% for ii=1:n_sim          %creates matrix A(i,j) where i is the simulation number and j the value at the j-step  
%     
%     jounce(ii,:)    = (out(1,ii).FL_jounce_pos.Data)';
%     steering(ii,:)  = linspace(imposed_steering(ii),imposed_steering(ii),length(out(1,ii).FL_jounce_pos.Data));   
%     camber(ii,:)    = (out(1,ii).FL_camber.Data)';
%     
% end
% 
% 
% %% Post-Process
% 
% figure('Name', 'Camber map')
% 
% surface(jounce,steering,camber,'EdgeColor','none');
% 
% xlabel('Jounce (mm)')
% ylabel('Steering angle (deg)')
% zlabel('camber angle  (deg)')
% colorbar
% title('$\gamma  = f\left( {x,\delta } \right)$','interpreter','Latex')
% 
% 
%  %% saving files
%  
%  save('Post-Process/Camber_Map_Steering_Jounce/output.mat')
%  
%  savefig('Post-Process/Camber_Map_Steering_Jounce/Camber_Map_Steering_Jounce')
