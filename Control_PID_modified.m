function Solution = Control_PID_modified(plant, config, sys_name)
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
% Modified: Sergio Jácobo Zavaleta

%% Initialize DE Parameters
DE = struct('maxIter', config.maxIterations, ...
    'popSize', config.populationSize, ...
    'maxPID', config.maxPIDValue, ...
    'F', config.F, ...
    'CR', config.CR, ...
    'dim', 3 ); % Number of parameters [Kp Ki Kd]

%% Initialize Tracking Variables
tracking = struct('globalError', zeros(DE.maxIter, 1), ...
    'minError', zeros(DE.maxIter, 1), ...
    'maxError', zeros(DE.maxIter, 1));

%% Generate Initial Population
population = initializePopulation(DE.popSize, DE.dim, DE.maxPID);

%% Evaluate Initial Population
for i = 1:DE.popSize
    population(i,1) = cost(population(i,2:(DE.dim+1)));
end

%% Main DE Loop
for iter = 1:DE.maxIter
    % Create trial population
    trialPopulation = generateTrialPopulation(population, DE);
    
    % Selection
    population = selection(population, trialPopulation);
    
    % Optional: Discard worst solutions (diversity maintenance)
    % When the population size is above 4 individuals
    if DE.popSize > 4
        population = discardWorstSolutions(population);
    end
    
    % Track progress
    [tracking, bestSolution] = updateProgress(tracking, population, iter);
    
    % Display progress every 4 iterations
    if mod(iter, 5) == 0
        displayProgress(iter, bestSolution, population);
    end
end

%% Prepare Output
Solution = generateSolution(bestSolution, tracking, population, plant);

%% Plot Results
plotResults(Solution,sys_name);
end

%% Helper Functions
function population = initializePopulation(popSize, dim, maxValue)
    % Initialize population with random PID parameters
    % Format: [cost Kp Ki Kd]
    population = [zeros(popSize, 1), maxValue * rand(popSize, dim)];
end

function trialPopulation = generateTrialPopulation(population, DE)
    % Generate trial solutions using DE mutation and crossover
    trialPopulation = zeros(size(population));
    
    for i = 1:DE.popSize
        % Select random indices for mutation
        indices = generateRandomIndices(i, DE.popSize);
        
        % Generate trial vector
        trial = generateTrialVector(population, indices, DE);
        
        % Evaluate trial solution
        trialPopulation(i,1) = cost(trial);
        trialPopulation(i,2:end) = trial;
    end
end

function indices = generateRandomIndices(targetIdx, popSize)
    % Generate three unique random indices different from current
    % indices = randperm(popSize);
    % indices(indices == targetIdx) = [];

    indices = zeros(1,3);
    indices(1) = random('unid',popSize);
    while(indices(1)==targetIdx || indices(1)==0)
        indices(1) = random('unid',popSize);
    end
    indices(2) = random('unid',popSize);
    while(indices(2)==targetIdx || indices(2)==indices(1) || indices(2)==0)
        indices(2) = random('unid',popSize);
    end
    indices(3) = random('unid',popSize);
    while(indices(3)==targetIdx || indices(3)==indices(1) || indices(3)==indices(2) || indices(3)==0)
        indices(3) = random('unid',popSize);
    end
end

function trial = generateTrialVector(population, indices, DE)
    % Generate trial vector using DE mutation and crossover
    a = indices(1); b = indices(2); c = indices(3);
    
    trial = zeros(1, DE.dim);
    for j = 1:DE.dim
        if rand < DE.CR
            trial(j) = population(c,j+1) + DE.F * (population(a,j+1) - population(b,j+1));% donor vector
        else
            trial(j) = population(c,j+1);% target vector
        end
        trial(j) = max(0, trial(j)); % Ensure non-negative PID parameters
    end
end

function population = selection(currentPop, trialPop)
    % Select best solutions between current and trial populations
    mask = trialPop(:,1) < currentPop(:,1);
    population = currentPop;
    population(mask,:) = trialPop(mask,:);
end

function population = discardWorstSolutions(population)
    % Sort population and replace worst solutions
    [~, idx] = sort(population(:,1));
    population = population(idx,:);
    
    % DISCARDING - The 5% of worst individuals are discarded
    nWorst = floor(length(population)* 0.05);
    for i = 1:nWorst
        randIdx = randi(floor(length(population)/2));
        population(end-i+1,:) = population(randIdx,:);
    end
end

function [tracking, bestSolution] = updateProgress(tracking, population, iter)
    % Update tracking metrics
    tracking.globalError(iter) = sum(population(:,1));
    tracking.minError(iter) = min(population(:,1));
    tracking.maxError(iter) = max(population(:,1));
    
    [~, bestIdx] = min(population(:,1));
    bestSolution.parameters = population(bestIdx,2:end);
    bestSolution.error = population(bestIdx,1);
end

function displayProgress(iter, bestSolution, population)
    % Display optimization progress
    fprintf('Iteration: %d\n', iter);
    fprintf('Best Error: %.4f | Worst Error: %.4f | Global Error: %.4f\n', ...
        bestSolution.error, max(population(:,1)), sum(population(:,1)));
    fprintf('PID Parameters - Kp: %.4f | Ki: %.4f | Kd: %.4f\n\n', ...
        bestSolution.parameters(1), bestSolution.parameters(2), bestSolution.parameters(3));
end

function Solution = generateSolution(bestSolution, tracking, population, plant)
    % Create final solution structure
    Solution = struct();
    Solution.bestmem = bestSolution.parameters;
    Solution.error = bestSolution.error;
    Solution.population = population;
    Solution.CONV = struct('MIN', tracking.minError, ...
                         'MAX', tracking.maxError, ...
                         'GLOBAL', tracking.globalError);
    
    % Create PID controller and closed-loop system
    pidController = pid(Solution.bestmem(1), Solution.bestmem(2), Solution.bestmem(3));
    Solution.M = feedback(pidController * plant, 1);
end

function plotResults(Solution,sys_name)
    % Plot closed-loop step response
    figure("Name","PID Optimization for position control "+sys_name);
    y = step(Solution.M, 0:0.05:5);
    plot(0:0.05:5, y, '-r', 'DisplayName', 'Estimated tf')
    grid on
    xlabel('Time (s)')
    ylabel('Position')
    legend('Location', 'best')
    title('Optimized Closed-loop Step Response '+sys_name)

    % Convergence history
    figure('Name', "Convergence History for PID optimization "+sys_name);
    plot(1:length(Solution.population), Solution.CONV.MIN, 'b-', 'LineWidth', 2);
    hold on;
    plot(1:length(Solution.population), Solution.CONV.MAX, 'r--', 'LineWidth', 2);
    plot(1:length(Solution.population), Solution.CONV.GLOBAL, 'g:', 'LineWidth', 2);
    grid on;
    xlabel('Iteration', 'FontSize', 12);
    ylabel('Cost', 'FontSize', 12);
    title("Convergence History "+sys_name);
    legend('Best', 'Worst', 'Global', 'Location', 'NorthEast');
    hold off;
end