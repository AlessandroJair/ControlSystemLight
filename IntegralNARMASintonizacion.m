clear; clc; close all;

% ----------------------------------------------------------
% 1. Configuración Inicial
% ----------------------------------------------------------
% Cargar parámetros de normalización (necesarios si el SLX usa el ANFIS)
if isfile('data/norm_params_azul.mat')
    load('data/norm_params_azul.mat');
    % Enviar al workspace base para que Simulink los vea
    assignin('base', 'min_u', min_u); assignin('base', 'max_u', max_u);
    assignin('base', 'min_bl', min_bl); assignin('base', 'max_bl', max_bl);
    assignin('base', 'lags_u', lags_u); assignin('base', 'lags_y', lags_y);
else
    warning('No se cargaron parámetros de normalización (norm_params_azul.mat no encontrado).');
end

% ----------------------------------------------------------
% 2. Límites de las variables a optimizar
% Variable 1: Ki (Ganancia Integral)
% Variable 2: Lambda (Factor de ponderación o aprendizaje)
% ----------------------------------------------------------
lb = [0.001  0.01];   % Límites inferiores [Ki, Lambda]
ub = [10.0   20.0];     % Límites superiores [Ki, Lambda]

% ----------------------------------------------------------
% 3. Opciones del GA
% ----------------------------------------------------------
options = optimoptions('ga', ...
    'PopulationSize', 20, ...
    'MaxGenerations', 30, ...
    'Display', 'iter', ...
    'UseParallel', false, ... % Cambiar a true si tienes Parallel Toolbox
    'PlotFcn', {@gaplotbestf});

fprintf('Iniciando optimización de Ki y Lambda...\n');

% ----------------------------------------------------------
% 4. Ejecutar GA
% ----------------------------------------------------------
% Nota: Pasamos una función anónima para fijar el nombre del archivo SLX
FunCosto = @(x) cost_NARMA_PID(x); 

[bestVars, bestJ] = ga(FunCosto, 2, [], [], [], [], lb, ub, [], options);

% ----------------------------------------------------------
% 5. Resultados
% ----------------------------------------------------------
bestKi     = bestVars(1);
bestLambda = bestVars(2);

disp("=================================");
disp("   OPTIMIZACIÓN COMPLETADA");
disp("=================================");
fprintf("Mejor Ki     = %.5f\n", bestKi);
fprintf("Mejor Lambda = %.5f\n", bestLambda);
fprintf("Costo Final  = %.5f\n", bestJ);

% Guardar resultados
if ~exist('controladores', 'dir'), mkdir('controladores'); end
save('controladores/NARMA_Optimizado.mat', 'bestKi', 'bestLambda', 'bestJ');


%% =========================================================
%  FUNCIÓN DE COSTO
% =========================================================
function J = cost_NARMA_PID(K)
    
    % 1. Extraer variables
    Ki_val     = K(1);
    Lambda_val = K(2);

    % 2. Asignar variables al Workspace Base (donde Simulink las busca)
    assignin('base', 'Ki', Ki_val);
    assignin('base', 'Lambda', Lambda_val);

    % 3. Ejecutar Simulink
    simOut = sim('NARMA.slx', ...
                 'SrcWorkspace', 'base', ...
                 'FastRestart', 'on'); % 'off' es más seguro si cambian params estructurales
    % 4. Extraer datos
    % Asumimos que el bloque "To Workspace" se llama 'Data_NARMA'
    % y está configurado como "Array" (Matriz).
    % Columnas: [1:Tiempo, 2:Referencia, 3:Salida]

    datos = simOut.Data_NARMA;
        
    t = datos(:, 1);
    r = datos(:, 2);
    y = datos(:, 3);
        
    % 5. Cálculo de Costo (ITAE + Overshoot)
    e = r - y;
        
    % ITAE: Integral del tiempo * error absoluto
    ITAE = trapz(t, t .* abs(e));

    J = ITAE;
end