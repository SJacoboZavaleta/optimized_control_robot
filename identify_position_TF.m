function G = identify_position_TF(Ts, pos, time, t1, t2)
    % Validación de entradas (añadir)
    if nargin ~= 5
        error('Se requieren 5 argumentos: Ts, pos, time, t1, t2');
    end
    if length(pos) ~= length(time)
        error('Los vectores pos y time deben tener la misma longitud');
    end
    
    % Elimina el primer punto
    time(1) = [];
    pos(1) = [];
    
    % Convierte tiempos a índices
    n1 = round(t1 / Ts) + 1;
    n2 = round(t2 / Ts) + 1;
    
    % Validación de índices (añadir)
    if n1 > length(pos) || n2 > length(pos) || n1 < 1 || n2 < 1
        error('Índices t1 o t2 fuera de rango');
    end
    
    % Cálculo de ganancia K
    K = (pos(n2) - pos(n1)) / (time(n2) - time(n1));
    
    % Punto medio y constante de tiempo
    n_m = round((n1 + n2) / 2);
    T = time(n_m) - (pos(n_m) - pos(1)) / K;
    
    % Función de transferencia
    G = tf(K, [T, 1, 0]);
    
    % Visualización
    figure('Name', 'Motor Transfer Function Identification')
    y = step(G, time);
    plot(time, pos, '.b', 'DisplayName', 'Real')
    hold on
    plot(time, y, '.r', 'DisplayName', 'Estimated')
    grid on
    xlabel('Time (s)')
    ylabel('Position')
    legend('Location', 'best')
    title('Open-loop Step Response')
    G
    % Cálculo y muestra de error
    error = norm(pos - y) / mean(pos) * 100;
    fprintf('Estimated transfer function:\n')
    fprintf('Estimation error: %.2f%%\n', error)
end