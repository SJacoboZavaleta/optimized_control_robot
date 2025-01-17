%--------------------------------------------------------------------------
% Class: PIDOptimizer
% Author: Sergio Jacobo Zavaleta
% Date: Enero, 2024
%--------------------------------------------------------------------------
% -> Description: 
%   Implementa un optimizador de controladores PID utilizando el algoritmo 
%   Differential Evolution (DE). La clase toma una función de transferencia 
%   de planta y optimiza los parámetros del controlador PID (Kp, Ki, Kd) 
%   para minimizar el error integral absoluto de la respuesta del sistema.
%   La optimización se realiza mediante simulación en Simulink.
%--------------------------------------------------------------------------
% -> Usage:
%   plant = tf(num, den);  % Función de transferencia de la planta
%   config = struct('populationSize', 20, 'maxIterations', 20, ...
%                  'F', 0.7, 'CR', 0.5, 'maxPIDValue', 1);
%   optimizer = PIDOptimizer(plant, config);
%   Solution = optimizer.optimize();
%--------------------------------------------------------------------------
% -> Input Parameters:
%   plant: Función de transferencia de la planta en bucle abierto
%   config: Estructura con parámetros de configuración (opcional)
%     - populationSize: Tamaño de la población DE (default: 20)
%     - maxIterations: Número máximo de iteraciones (default: 20)
%     - F: Factor de mutación diferencial [0-2] (default: 0.7)
%     - CR: Tasa de recombinación [0-1] (default: 0.5)
%     - maxPIDValue: Valor máximo para parámetros PID (default: 1)
%--------------------------------------------------------------------------
% -> Output: Solution = 
%   - bestmem: [Kp Ki Kd] Mejores parámetros PID encontrados
%   - error: Valor de la función de costo final
%   - population: Población final con costos asociados
%   - CONV: Historial de convergencia (MIN, MAX, GLOBAL)
%   - M: Función de transferencia del sistema en lazo cerrado con PID
%--------------------------------------------------------------------------
% -> Requirements:
%   - MATLAB R2019b o superior
%   - Simulink
%   - Control System Toolbox
%--------------------------------------------------------------------------
% Solution utilizando programación orientada a objetos. Basado en el código
% compartido en el curso Control Inteligente 2024 UC3M.
%--------------------------------------------------------------------------

classdef PIDOptimizer
    properties (Access = private)
        % System parameters
        plantTF
        simulinkModel = 'cst_robotarm'
        system_name = ": motor"

        % DE parameters
        populationSize
        dimensions
        maxIterations
        F  % Differential weight
        CR % Crossover rate
        maxPIDValue
        
        % Results storage
        convergenceHistory
    end
    
    methods (Access = public)
        function obj = PIDOptimizer(plant, config, sys_name)
            % Constructor
            if nargin < 2
                config = struct();
            end
            
            % Initialize system parameters
            obj.plantTF = plant;
            obj.system_name = sys_name;

            % Set DE parameters with defaults
            obj.populationSize = getOpt(config, 'populationSize', 20);
            obj.dimensions = 3; % [Kp, Ki, Kd]
            obj.maxIterations = getOpt(config, 'maxIterations', 20);
            obj.F = getOpt(config, 'F', 0.7);
            obj.CR = getOpt(config, 'CR', 0.5);
            obj.maxPIDValue = getOpt(config, 'maxPIDValue', 1);
            
            % Verify Simulink model exists
            if ~isfile([obj.simulinkModel, '.slx'])
                error('Simulink model %s.slx not found', obj.simulinkModel);
            end
        end
        
        function Solution = optimize(obj)
            % Initialize population and history
            population = obj.initializePopulation();
            obj.convergenceHistory = struct('MIN', zeros(obj.maxIterations, 1), ...
                                          'MAX', zeros(obj.maxIterations, 1), ...
                                          'GLOBAL', zeros(obj.maxIterations, 1));
            
            % Main DE loop
            for iter = 1:obj.maxIterations
                population = obj.evolvePopulation(population);
                obj = obj.updateConvergenceHistory(population, iter);
                
                % Display progress
                if mod(iter, 5) == 0
                    fprintf('Iteration %d: Best=%.4f, Worst=%.4f, Global=%.4f\n', ...
                        iter, obj.convergenceHistory.MIN(iter), ...
                        obj.convergenceHistory.MAX(iter), ...
                        obj.convergenceHistory.GLOBAL(iter));
                end
            end
            
            % Get best solution
            [~, bestIdx] = min(population(:,1));
            bestParams = population(bestIdx, 2:end);
            
            % Create final PID controller
            pidController = pid(bestParams(1), bestParams(2), bestParams(3));
            M = feedback(pidController * obj.plantTF, 1);
            
            % Prepare solution structure
            Solution = struct();
            Solution.bestmem = bestParams;
            Solution.error = population(bestIdx, 1);
            Solution.population = population;
            Solution.CONV = obj.convergenceHistory;
            Solution.M = M;

            % Plot results
            obj.plotResults(Solution);
        end
    end
    
    methods (Access = private)
        function population = initializePopulation(obj)
            population = zeros(obj.populationSize, obj.dimensions + 1);
            
            for i = 1:obj.populationSize
                % Generate random PID parameters
                params = obj.maxPIDValue * rand(1, obj.dimensions);
                
                % Evaluate cost using Simulink model
                population(i, 2:end) = params;
                population(i, 1) = obj.evaluateCost(params);
            end
        end
        
        function error = evaluateCost(obj, params)
            % Prepare parameters for Simulink
            warning('OFF');
            
            % Extract PID parameters
            Kp = params(1);
            Ki = params(2);
            Kd = params(3);
            
            % Crear estructura con los parámetros para evitar múltiples assignin
            params = struct('Kp', Kp, 'Ki', Ki, 'Kd', Kd);
            fields = fieldnames(params);
            for i = 1:length(fields)
                assignin('base', fields{i}, params.(fields{i}));
            end
            
            % Configuración y ejecución de la simulación
            simOut = sim(obj.simulinkModel, ...
                'SrcWorkspace', 'current', ...
                'FastRestart', 'on');
            
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
        
        function population = evolvePopulation(obj, population)
            newPopulation = population;
            
            for i = 1:obj.populationSize
                % Generate trial vector
                trial = obj.generateTrialVector(population, i);
                
                % Evaluate cost
                trialCost = obj.evaluateCost(trial);
                
                % Selection
                if trialCost < population(i, 1)
                    newPopulation(i, 1) = trialCost;
                    newPopulation(i, 2:end) = trial;
                end
            end
            
            % Sort and apply elitism
            population = obj.applyElitism(newPopulation);
        end
        
        function trial = generateTrialVector(obj, population, targetIdx)
            % Select random indices
            idx = randperm(obj.populationSize);
            idx(idx == targetIdx) = [];
            a = idx(1); b = idx(2); c = idx(3);
            
            % Mutation and crossover
            trial = zeros(1, obj.dimensions);
            for j = 1:obj.dimensions
                if rand <= obj.CR
                    trial(j) = population(c, j+1) + obj.F * (population(a, j+1) - population(b, j+1));% donor vector
                else
                    trial(j) = population(targetIdx, j+1);% target vector
                end
                
                % Ensure non-negative values
                trial(j) = max(0, trial(j));
            end
        end
        
        function population = applyElitism(obj, population)
            [~, idx] = sort(population(:,1));
            population = population(idx,:);
            
            % DISCARDING - The 5% of worst individuals are discarded
            % When the population size is above 4 individuals
            if obj.populationSize > 4
                nWorst = floor(obj.populationSize * 0.05);
                for i = 1:nWorst
                    randIdx = randi([1, floor(obj.populationSize/2)]);
                    population(end-i+1,:) = population(randIdx,:);
                end
            end
        end
        
        function obj = updateConvergenceHistory(obj, population, iter)
            %It is already sorted
            %obj.convergenceHistory.MIN(iter) = population(1,1);
            obj.convergenceHistory.MIN(iter) = min(population(:,1));
            obj.convergenceHistory.MAX(iter) = max(population(:,1));
            obj.convergenceHistory.GLOBAL(iter) = sum(population(:,1));
        end
        
        function plotResults(obj, Solution)
            % Step response
            figure("Name","PID Optimization for position control "+obj.system_name);
            y = step(Solution.M, 0:0.05:5);
            plot(0:0.05:5, y, '-r', 'DisplayName', 'Estimated tf')
            grid on
            xlabel('Time (s)', 'FontSize', 12)
            ylabel('Position', 'FontSize', 12)
            title("Optimized Closed-loop Step Response "+obj.system_name)
            legend('Location', 'best')
            
            % Convergence history
            figure('Name', "Convergence History for PID optimization "+obj.system_name);
            plot(1:config.maxIterations, Solutionv1_1.CONV.MIN, 'b-', 'LineWidth', 2);
            hold on;
            plot(1:config.maxIterations, Solutionv1_1.CONV.MAX, 'r--', 'LineWidth', 2);
            plot(1:config.maxIterations, Solutionv1_1.CONV.GLOBAL, 'g:', 'LineWidth', 2);
            grid on
            xlabel('Iteration', 'FontSize', 12);
            ylabel('Cost', 'FontSize', 12);
            title("Convergence History "+obj.system_name);
            legend('Best', 'Worst', 'Global', 'Location', 'NorthEast');
        end
    end
end

function value = getOpt(config, field, default)
    if isfield(config, field)
        value = config.(field);
    else
        value = default;
    end
end