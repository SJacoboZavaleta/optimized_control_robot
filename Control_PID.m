function Solution = Control_PID(plant, config, sys_name)
%--------------------------------------------------------------------------
%   Main Program: Control_PID.
%   Author: Fernando Martin Monar.
%   Date: April, 2013
%--------------------------------------------------------------------------
% -> Description: A partir de la funcion de transferencia en bucle abierto
% de la planta, el programa estima los parámetros de un regulador PID para
% optimizar la respuesta a entrada escalón del sistema utilizando el
% algoritmo Differential Evolution
%--------------------------------------------------------------------------
% -> Usage:
%         []=Control_PID
%--------------------------------------------------------------------------
% -> Parámetros a inicializar:
%   Funcion de transferencia del proceso a controlar, en bucle abierto:
%       K= Ganancia del motor
%       T= Constante de tiempo mecanico
%       plant=tf(K,[T 1 0]);
%------------------------------
%   Variables del DE:
%       iter_max= Número de iteraciones
%       NP= Tamaño de la población
%       MAX_K_VAR= Máximo valor inicial para las constantes del PID
%       F=... Factor de amplitud de diferencias del DE, [0-2]  
%       CR=... Constante de cruce, [0-1]
%--------------------------------------------------------------------------
% -> Output: Solution=
%       Solution.bestmem: [Kp Ki Kd]
%       Solution.error: valor de coste
%       Solution.population: población, el primer elemento de cada fila es
%       el coste asociado del elemento de esa fila
%       Solution.CONV: evolución de la funcion de error con las
%       iteraciones.
%       Solution.M: función de transferencia del sistema con el PID
%       calculado, en bucle cerrado
%       Solution.Pm: margen de fase
%       Solution.Wpm: frecuencia de margen de fase
% -> See also: cost
%--------------------------------------------------------------------------

%------------------------------
% Initialization of DE variables
iter_max=config.maxIterations;%50;
NP=config.populationSize;  %Number of particles
MAX_K_VAR=config.maxPIDValue;%5; % Maximum value of the PID constants.
F=config.F;          %differential mutation factor (0-2)  
CR=config.CR;        %crossover constant (0-1)
%------------------------------
D=3;    %Number of chromosomes (Kp,Kd,Ki)
%------------------------------
% Initialization of some auxiliar variables
trial=zeros(1,D);
pob_aux=zeros(NP,D+1);
error=10000;
error_max=20000;
GLOBAL=zeros(iter_max,1);
MIN=zeros(iter_max,1);
MAX=zeros(iter_max,1);

% The PID Controller is defined by the following Expression:
%       PID(s)=Kp + Ki/s + Kd*s
% The initial population is generated.
population=inicio_pob(NP,D,MAX_K_VAR);

% Cost function evaluation for the initial population.
for i=1:NP        
    population(i,1)=cost(population(i,2:(D+1)));
end

%The DE algorithm estimates the best solution
count=1;
%imp_count=1;
while(count<=iter_max)% && error_max > (error + 0.000001))
    for i=1:NP
        %------------------------------------------------------------------
        %MUTATION AND CROSSOVER
        %Thre different vectors and different from runnning index i
        a=random('unid',NP);
        while((a==i)||(a==0))
            a=random('unid',NP);
        end
        b=random('unid',NP);
        while((b==i)||(b==a)||(b==0))
            b=random('unid',NP);
        end
        c=random('unid',NP);
        while((c==i)||(c==a)||(c==b)||(c==0))
            c=random('unid',NP);
        end        
        for k=2:(D+1)
            cross_rand=random('unid',100);
            if(cross_rand < (100*CR))
                trial(1,(k-1))=population(c,k)+ F*(population(a,k)-population(b,k));
            else trial(1,(k-1))=population(i,k);
            end
        end
        
        % The Ks cannot be negative numbers
        if trial(1,1)<0
            trial(1,1)=0;
        end
        if trial(1,2)<0
            trial(1,2)=0;
        end
        if trial(1,3)<0
            trial(1,3)=0;
        end
        
        %------------------------------------------------------------------
        % Cost function evaluation according to each hypothesis.
        % error_trial=cost(plant,population(i,2:(D+1)));
        error_trial=cost(trial);%cost(plant,trial);
        %------------------------------------------------------------------
        % SELECTION: The best individuals are chosen for the next
        % generation (between candidates and current
        % population).Thresholding band to deal with noisy measurements is
        % also implemented.
        if(error_trial<population(i,1)*1.00) %Thresholding band: Thau= 0.00
            for j=2:(D+1)
                pob_aux(i,j)=trial(1,j-1);
            end
            pob_aux(i,1)=error_trial;
        else
            for j=1:(D+1)
                pob_aux(i,j)=population(i,j);
            end
        end
    end
    %----------------------------------------------------------------------
    population=pob_aux;    %Population for the next generation.
    %----------------------------------------------------------------------
    %DISCARDING - The 5% of worst individuals are discarded
    pob_orden=sortrows(population,1);
    if NP<10
        ndisc=1;
    else
        ndisc=int8(NP/20);
    end
    if NP>4 %discarding is is nonsense with less than 5 individuals
        for i=1:ndisc
            disc=random('unid',double(int8(NP/2)));
            while disc==0
                disc=random('unid',double(int8(NP/2)));
            end
            pob_orden(NP+1-i,:)=pob_orden(disc,:);
        end
    end
    population=pob_orden;
    %----------------------------------------------------------------------
    %The best, worst and global error are printed     
    bestmem=population(1,2:(D+1));
    error=population(1,1);
    error_max=max(population(:,1));
    error_global=sum(population(:,1)); 
    if mod(count, 5) == 0
        %fprintf(1,'\n It: %d Best %f Worst: %f Global: %f \n Parameters: Kp: %f Ki: %f Kd: %f \n',count,error,error_max,error_global,bestmem(1),bestmem(2),bestmem(3));
        %imp_count=0;

        fprintf('Iteration: %d\n', count);
        fprintf('Best Error: %.4f | Worst Error: %.4f | Global Error: %.4f\n', ...
        error, error_max, error_global);
        fprintf('PID Parameters - Kp: %.4f | Ki: %.4f | Kd: %.4f\n\n', ...
        bestmem(1), bestmem(2), bestmem(3));
    end
    
    GLOBAL(count)=error_global;
    MIN(count)=error;
    MAX(count)=error_max;
    
    %imp_count=imp_count+1;
    count=count+1;
end

CONV.MIN=MIN;
CONV.MAX=MAX;
CONV.GLOBAL=GLOBAL;

Solution.bestmem=bestmem;
Solution.error=error;
Solution.population=population;
Solution.CONV=CONV;

% Crear el controlador PID
PID = pid(bestmem(1), bestmem(2), bestmem(3));
%C = tf([bestmem(3) bestmem(1) bestmem(2)], [1 0]);  % Forma: Kd*s + Kp + Ki/s

% Crear el sistema en lazo cerrado
Solution.M = feedback(PID*plant, 1);
% Plot of the response when the input is a step, useful when designing with
% the error of the step response

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
plot(1:iter_max, CONV.MIN, 'b-', 'LineWidth', 2);
hold on;
plot(1:iter_max, CONV.MAX, 'r--', 'LineWidth', 2);
plot(1:iter_max, CONV.GLOBAL, 'g:', 'LineWidth', 2);
grid on;
xlabel('Iteration', 'FontSize', 12);
ylabel('Cost', 'FontSize', 12);
title("Convergence History "+sys_name);
legend('Best', 'Worst', 'Global', 'Location', 'NorthEast');
hold off;

%--------------------------------------------------------------------------
%   Function: Population initialization.
%   Author: Fernando Martin Monar.
%   Date: October, 2010
%--------------------------------------------------------------------------
% Description:   
%   The initial population of the DE algorithm is generated
%--------------------------------------------------------------------------
% Usage:
%   pop=inicio_pob(NP,D,D_exp)
% ->where NP is the number of elements and D is the number of genes. 
% Output:  pop (size NP*(D+1)). Each row has the cost function value in the
% first column and the values of the PID in the other ones.
%--------------------------------------------------------------------------
% Parameters: 
%   NP          Number of particles
%   D=3         Number of chromosomes (Kp,Kd,Ki)
%   D_exp;      Number of chormosomes (of D) that correspond to exponents.
%   MAX_K_VAR;  Maximum value of the PID constants.
%--------------------------------------------------------------------------
function pop=inicio_pob(NP,D,MAX_K_VAR)
poppre=zeros(NP,D);
cost=zeros(NP,1);
for i=1:NP
   poppre(i,:) = MAX_K_VAR*rand(1,D);
end
pop=[cost poppre];