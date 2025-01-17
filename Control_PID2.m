function Solution = Control_PID2(plant, config, sys_name)
%% PID Controller Parameter Optimization using Differential Evolution
% Optimizes PID controller parameters for a given plant transfer function
% using Differential Evolution (DE) algorithm.
%
% Inputs:
%   plant  - Transfer function of the plant in open loop
%            Example: plant = tf(K, [T 1 0]) where:
%                    K = Plant gain
%                    T = Mechanical time constant
%   config - Configuration structure with fields:
%            .maxIterations  - Maximum number of iterations
%            .populationSize - Size of DE population
%            .maxPIDValue   - Maximum initial value for PID constants
%            .F            - DE differential mutation factor [0-2]
%            .CR           - DE crossover constant [0-1]
%
% Outputs:
%   Solution - Structure containing:
%              .bestmem    - Optimized [Kp Ki Kd] parameters
%              .error      - Final cost value
%              .population - Final population with costs
%              .CONV       - Convergence history
%              .M         - Closed-loop transfer function
%
% Author: Fernando Martin Monar (Original)
% Modified: [Current Date]

%% Initialize Parameters
maxIter = config.maxIterations;
popSize = config.populationSize;
maxPID = config.maxPIDValue;
F = config.F;
CR = config.CR;
nParams = 3;  % Number of parameters [Kp Ki Kd]

%% Initialize Tracking Variables
globalError = zeros(maxIter, 1);
minError = zeros(maxIter, 1);
maxError = zeros(maxIter, 1);

%% Generate Initial Population
% Format: [cost Kp Ki Kd]
population = [zeros(popSize, 1), maxPID * rand(popSize, nParams)];

%% Evaluate Initial Population
for i = 1:popSize
    population(i,1) = cost(population(i,2:(nParams+1)));
end

%% Main DE Loop
for iter = 1:maxIter
    % Create trial population
    for i = 1:popSize
        % Generate unique random indices for mutation
        a = randi(popSize);
        while a == i
            a = randi(popSize);
        end
        
        b = randi(popSize);
        while b == i || b == a
            b = randi(popSize);
        end
        
        c = randi(popSize);
        while c == i || c == a || c == b
            c = randi(popSize);
        end
        
        % Generate trial vector
        trial = zeros(1, nParams);
        for j = 1:nParams
            if rand < CR
                trial(j) = population(c,j+1) + F * (population(a,j+1) - population(b,j+1));
            else
                trial(j) = population(i,j+1);
            end
            % Ensure non-negative PID parameters
            trial(j) = max(0, trial(j));
        end
        
        % Evaluate trial vector
        trialCost = cost(trial);
        
        % Selection
        if trialCost < population(i,1)
            population(i,1) = trialCost;
            population(i,2:end) = trial;
        end
    end
    
    % Optional: Discard worst solutions (diversity maintenance)
    if popSize > 4
        [~, idx] = sort(population(:,1));
        population = population(idx,:);
        
        nDiscard = max(1, floor(popSize/20));
        for i = 1:nDiscard
            randIdx = randi(floor(popSize/2));
            population(end-i+1,:) = population(randIdx,:);
        end
    end
    
    % Update tracking
    globalError(iter) = sum(population(:,1));
    minError(iter) = min(population(:,1));
    maxError(iter) = max(population(:,1));
    
    % Get best solution
    [bestError, bestIdx] = min(population(:,1));
    bestParams = population(bestIdx,2:end);
    
    % Display progress every 4 iterations
    if mod(iter, 5) == 0
        fprintf('Iteration: %d\n', iter);
        fprintf('Best Error: %.4f | Worst Error: %.4f | Global Error: %.4f\n', ...
            bestError, max(population(:,1)), globalError(iter));
        fprintf('PID Parameters - Kp: %.4f | Ki: %.4f | Kd: %.4f\n\n', ...
            bestParams(1), bestParams(2), bestParams(3));
    end
end

%% Prepare Output
Solution = struct();
Solution.bestmem = bestParams;
Solution.error = bestError;
Solution.population = population;
Solution.CONV = struct('MIN', minError, ...
                     'MAX', maxError, ...
                     'GLOBAL', globalError);

% Create PID controller and closed-loop system
pidController = pid(bestParams(1), bestParams(2), bestParams(3));
Solution.M = feedback(pidController * plant, 1);

%% Plot Results
figure("Name","PID Optimization for position control "+sys_name);
y = step(Solution.M, 0:0.05:5);
plot(0:0.05:5, y, '-r', 'DisplayName', 'Estimated tf')
grid on
xlabel('Time (s)', 'FontSize', 12)
ylabel('Position', 'FontSize', 12)
title("Optimized Closed-loop Step Response "+sys_name)
legend('Location', 'best')

% Convergence history
figure('Name', "Convergence History for PID optimization "+sys_name);
plot(1:maxIter, Solution.CONV.MIN, 'b-', 'LineWidth', 2);
hold on;
plot(1:maxIter, Solution.CONV.MAX, 'r--', 'LineWidth', 2);
plot(1:maxIter, Solution.CONV.GLOBAL, 'g:', 'LineWidth', 2);
grid on;
xlabel('Iteration', 'FontSize', 12);
ylabel('Cost', 'FontSize', 12);
title("Convergence History "+sys_name);
legend('Best', 'Worst', 'Global', 'Location', 'NorthEast');
hold off;
end