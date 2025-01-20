function error = cost_adopted(param)
%--------------------------------------------------------------------------
%   Function: Population initialization.
%   Author: Fernando Martin Monar.
%   Date: April, 2013
%--------------------------------------------------------------------------
% -> Description:   
%   It calculates the cost function given a process Gp and a PID controller. 
%   The DE-based scan matching algorithm calls it to estimate the cost.
%   The PID Controller is defined by the following Expression:
%       PID(s)=Kp + Ki/s + Kd*s
%--------------------------------------------------------------------------
% -> Usage:
%       Inputs: Gp -> transfer function of the process to control
%               param = [Kp Ki Kd]; parameters of the PID
% Output:  error. The error is computed as the average squared error.
%--------------------------------------------------------------------------
% -> See also: Control_PID
%--------------------------------------------------------------------------

% Initialization parameters
warning('OFF');

% Configuración para ejecución en background
load_system('cst_robotarm_optimized'); % Carga el modelo sin abrirlo
set_param('cst_robotarm_optimized', 'SimulationMode', 'accelerator');
set_param('cst_robotarm_optimized', 'FastRestart', 'on'); % Habilita inicio rápido

% Asignación de parámetros del controlador PID
Kp = param(1);
Ki = param(2);
Kd = param(3);

% Crear estructura con los parámetros para evitar múltiples assignin
params = struct('Kp', Kp, 'Ki', Ki, 'Kd', Kd);
fields = fieldnames(params);
for i = 1:length(fields)
    assignin('base', fields{i}, params.(fields{i}));
end
set_param('cst_robotarm_optimized','SimulationCommand','Update')

% Configuración y ejecución de la simulación
simOut = sim('cst_robotarm_optimized', ...
    'SrcWorkspace', 'base','FastRestart', 'on');

% Extracción eficiente de datos
signals = simOut.simout.signals.values;
s = size(signals);
m = s(1); % Número de muestras

% Extraer columnas de datos de una vez
t = signals(:,1);        % Tiempo
ErrInst = signals(:,2);  % Error Instantáneo
d = signals(:,3);        % Respuesta

% Cálculos de métricas
TempFin = t(end);
Maximo = max(d);
ErrInt = sum(abs(ErrInst));
error = ErrInt/m;

% Asignación eficiente de variables al workspace
varsToAssign = struct('s', s, 'm', m, 't', t, 'd', d, ...
    'ErrInst', ErrInst, 'TempFin', TempFin, ...
    'Maximo', Maximo, 'error', error);

fields = fieldnames(varsToAssign);
for i = 1:length(fields)
    assignin('base', fields{i}, varsToAssign.(fields{i}));
end
                      
end