clear; clc; close all; 
% rng('shuffle'); 

% Desactivar advertencias de rango fuzzy
warning('off', 'fuzzy:general:warnEvalfisInputOutOfRange');

%% --- 1) Configuración y Carga de Datos ---
carpeta_data = 'data';
carpeta_out  = 'controladores';

if ~exist(carpeta_out, 'dir'), mkdir(carpeta_out); end

% A. Cargar Parámetros de Normalización y Lags
file_norm = fullfile(carpeta_data, 'norm_params_fr.mat');
if ~isfile(file_norm), error('Falta %s', file_norm); end
load(file_norm); % Carga: min_u, max_u, min_bl, max_bl, lags_u, lags_y

% B. Cargar Modelo ANFIS (NARX)
file_fis = fullfile(carpeta_data, 'modelo_fr_dinamico.fis');
if ~isfile(file_fis), error('Falta %s', file_fis); end
fis = readfis(file_fis);

% C. Cargar Función de Transferencia (Para K y T del modelo aproximado)
file_ft = fullfile(carpeta_data, 'modelo_ft_fr.mat');
if ~isfile(file_ft), error('Falta %s', file_ft); end
load(file_ft, 'sys_tf'); 

fprintf('Datos cargados.\n -> Lags Output: %d\n -> Lags Input: %d\n', lags_y, lags_u);

%% --- 2) Extracción Automática de K y T (Modelo Lineal para SMC) ---
% G(s) = K / (T*s + 1)
K_model = dcgain(sys_tf);           
polos   = pole(sys_tf);             
polo_dom = min(abs(real(polos)));   
T_model = 1 / polo_dom;             

fprintf('Parametros SMC estimados: K=%.4f, T=%.4f\n', K_model, T_model);

%% --- 3) Configuración del GA ---
disp('Configurando Algoritmo Genético...');

% Variables a optimizar: [ks, lambda, phi]
% ks:     Ganancia de conmutación
% lambda: Ancho de banda (velocidad)
% phi:    Capa límite (suavizado)
lb = [0.1,  0.01,  0.1]; 
ub = [50,   5.0,   5.0];

% Tiempos
ts = 0.25;       
t_final = 30;   

% Referencia: Escalón al 60%
ref_target = (max_bl - min_bl) * 0.6 + min_bl;

% Función de costo (Wrapper)
ObjFcn = @(v) fitness_SMC_ITAE(v, fis, ts, t_final, ref_target, ...
    K_model, T_model, min_u, max_u, min_bl, max_bl, lags_u, lags_y);

opts = optimoptions('ga', ...
    'PopulationSize', 40, ...
    'MaxGenerations', 30, ...
    'Display', 'iter', ...
    'UseParallel', false, ... % CAMBIO: false para evitar error de pool
    'PlotFcn', @gaplotbestf);

disp('Iniciando optimización (Criterio: SOLO ITAE)...');
[best_vars, final_cost] = ga(ObjFcn, 3, [], [], [], [], lb, ub, [], opts);

ks_opt     = best_vars(1);
lambda_opt = best_vars(2);
phi_opt    = best_vars(3);

fprintf('\n--- RESULTADOS ÓPTIMOS ---\n');
fprintf('Ks: %.4f | Lambda: %.4f | Phi: %.4f\n', ks_opt, lambda_opt, phi_opt);
fprintf('ITAE Final: %.4f\n', final_cost);

% Guardar controlador
save(fullfile(carpeta_out, 'SMC_FR_Optimizado.mat'), ...
    'ks_opt', 'lambda_opt', 'phi_opt', 'K_model', 'T_model', 'ref_target');

%% --- 4) Simulación Final y Gráficas ---
[t, y_real, u, u_eq_hist, s_hist] = simulate_SMC_NARX(ks_opt, lambda_opt, phi_opt, ...
    fis, ts, t_final, ref_target, K_model, T_model, ...
    min_u, max_u, min_bl, max_bl, lags_u, lags_y);

figure('Name', 'SMC Optimizado ITAE', 'Color', 'w');

subplot(3,1,1);
yline(ref_target, 'k--', 'LineWidth', 1.5); hold on;
plot(t, y_real, 'b', 'LineWidth', 1.5);
grid on; ylabel('Luminosidad [W/m^2]');
title(['Respuesta Temporal (ITAE=' num2str(final_cost,'%.2f') ')']);
legend('Referencia', 'Salida SMC');

subplot(3,1,2);
plot(t, u, 'r', 'LineWidth', 1.2); hold on;
plot(t, u_eq_hist, 'g:', 'LineWidth', 1);
grid on; ylabel('PWM (u)');
legend('Control Total', 'Control Eq');
title('Esfuerzo de Control');

subplot(3,1,3);
plot(t, s_hist, 'm', 'LineWidth', 1.2);
yline(phi_opt, 'k:', 'LineWidth', 1.2); 
yline(-phi_opt, 'k:', 'LineWidth', 1.2);
grid on; ylabel('Superficie S');
title('Superficie de Deslizamiento');


%% ========================================================================
%                  FUNCIONES LOCALES
% =========================================================================

function J = fitness_SMC_ITAE(vars, fis, ts, t_final, ref_val, Km, Tm, ...
                              umin, umax, ymin, ymax, lags_u, lags_y)

    ks  = vars(1);
    lam = vars(2);
    phi = vars(3);

    [t, y, ~, ~, ~, is_unstable] = simulate_SMC_NARX( ...
        ks, lam, phi, fis, ts, t_final, ref_val, ...
        Km, Tm, umin, umax, ymin, ymax, lags_u, lags_y);

    if is_unstable
        J = 1e9;
        return;
    end

    %% --- Error ---
    e = ref_val - y;

    %% --- ITAE ---
    J_itae = sum(t .* abs(e)) * ts;

    %% --- Overshoot ---
    overshoot = max(0, y - ref_val);
    J_os = sum(t .* overshoot.^2) * ts;

    %% --- Tiempo de establecimiento ---
    beta = 0.02;  % 2%
    tol = beta * abs(ref_val);

    idx_settle = find(abs(e) > tol, 1, 'last');

    if isempty(idx_settle)
        Ts = 0;  % se asentó muy rápido
    else
        Ts = t(idx_settle);
    end

    J_ts = Ts^2;  % penalización cuadrática

    %% --- Costo total ---
    alpha = 500;   % overshoot
    gamma = 10;    % tiempo de establecimiento

    J = J_itae + alpha * J_os + gamma * J_ts;
end

function [t, y_hist, u_hist, ueq_hist, s_hist, is_unstable] = simulate_SMC_NARX(...
    ks, lambda, phi, fis, ts, t_final, ref_val, K_model, T_model, ...
    min_u, max_u, min_bl, max_bl, lags_u, lags_y)

    t = 0:ts:t_final;
    N = length(t);
    
    y_hist = zeros(1,N);
    u_hist = zeros(1,N);
    ueq_hist = zeros(1,N);
    s_hist = zeros(1,N);
    
    % Condiciones iniciales (Todo en mínimos)
    y_hist(:) = min_bl;
    u_hist(:) = min_u;
    
    y_prev = min_bl; 
    u_prev = min_u;
    e_integral = 0;
    
    is_unstable = false;
    
    for k = 1:N
        % --- 1. Calcular Error ---
        % Usamos la salida anterior para calcular el error actual
        if k > 1, y_prev = y_hist(k-1); else, y_prev = min_bl; end
        
        e = ref_val - y_prev;
        
        % --- 2. Anti-Windup (Integral de Superficie) ---
        saturated = (u_prev >= max_u && e > 0) || (u_prev <= min_u && e < 0);
        
        if ~saturated
            e_integral = e_integral + e * ts;
        end
        
        % --- 3. Superficie de Deslizamiento ---
        s = e + lambda * e_integral;
        s_hist(k) = s;
        
        % --- 4. Control Equivalente (Dinámica Inversa Aprox) ---
        u_eq = (y_prev + T_model * lambda * e) / K_model;
        ueq_hist(k) = u_eq;
        
        % --- 5. Control de Conmutación (Switching) ---
        u_sw = ks * tanh(s / phi);

        % --- 6. Control Total y Saturación ---
        u_total = u_eq + u_sw;
        u_curr = max(min_u, min(max_u, u_total));
        
        u_hist(k) = u_curr;
        u_prev = u_curr; % Actualizar para siguiente ciclo
        
        % --- 7. EVALUACIÓN ANFIS (PLANTA NARX) ---
        % Construcción del vector input IDÉNTICA al PID
        input_vector = [];
        
        % A. Retardos de SALIDA (y)
        for d = 1:lags_y
            idx = k - d;
            if idx < 1, val = min_bl;
            else,       val = y_hist(idx); end
            
            % Normalizar y Clampear
            val_norm = (val - min_bl)/(max_bl - min_bl);
            val_norm = max(0, min(1, val_norm)); 
            
            input_vector = [input_vector, val_norm]; %#ok<AGROW>
        end
        
        % B. Retardos de ENTRADA (u)
        % Incluye el actual (d=0)
        for d = 0:lags_u
            idx = k - d;
            if idx < 1, val = min_u;
            else,       val = u_hist(idx); end
            
            val_norm = (val - min_u)/(max_u - min_u);
            val_norm = max(0, min(1, val_norm)); 
            
            input_vector = [input_vector, val_norm]; %#ok<AGROW>
        end
        
        % C. Predecir siguiente salida
        y_next_norm = evalfis(fis, input_vector);
        y_next = y_next_norm * (max_bl - min_bl) + min_bl;
        
        % Chequeo de estabilidad
        if isnan(y_next) || isinf(y_next) || abs(y_next) > (max_bl*3)
            is_unstable = true;
            return;
        end
        
        y_hist(k) = y_next;
    end
end