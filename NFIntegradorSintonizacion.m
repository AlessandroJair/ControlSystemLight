clear; clc; close all; rng('shuffle');

% Desactivar advertencias
warning('off', 'fuzzy:general:warnEvalfisInputOutOfRange');

%% --- 1. Configuración y Carga ---
carpeta_data = 'data';
carpeta_ctrl = 'controladores';

% A. Cargar Parámetros y Modelos
if ~isfile(fullfile(carpeta_data, 'norm_params_fr.mat')), error('Falta norm_params'); end
load(fullfile(carpeta_data, 'norm_params_fr.mat')); 

% B. Planta (ANFIS NARX)
fis_planta = readfis(fullfile(carpeta_data, 'modelo_fr_dinamico.fis'));

% C. Controlador ANFIS Base (3 Inputs)
file_nf3 = fullfile(carpeta_ctrl, 'NF_FR.fis');
if ~isfile(file_nf3), error('No se encuentra NF_FR.fis'); end
nf_3in = readfis(file_nf3);

fprintf('Modelos cargados. Iniciando optimización de Integración Condicional...\n');

%% --- 2. Configuración del GA ---
% Variables a optimizar: [Ki, Umbral_Activacion]
% Ki: Ganancia del integrador
% Umbral: Valor de ERROR (W/m2) debajo del cual se enciende el integrador.

lb = [0.01,  0.5];   % Min: Ki bajo, Umbral pequeño (0.5 W/m2)
ub = [10.0,  15.0];  % Max: Ki fuerte, Umbral grande (15 W/m2)

opts = optimoptions('ga', ...
    'PopulationSize', 30, ...
    'MaxGenerations', 30, ...
    'Display', 'iter', ...
    'UseParallel', false, ...
    'PlotFcn', @gaplotbestf);

% Escenario
ts = 0.25;
t_total = 30;
t = 0:ts:t_total;
N = length(t);

% Referencia Escalera
ref_vec = zeros(1, N);
ref_vec(t>=0 & t<10)  = min_bl + (max_bl - min_bl)*0.3; 
ref_vec(t>=10 & t<20) = min_bl + (max_bl - min_bl)*0.7; 
ref_vec(t>=20)        = min_bl + (max_bl - min_bl)*0.4; 

% Función de Costo
ObjFcn = @(vars) fitness_Hybrid_Cond(vars, nf_3in, fis_planta, t, ref_vec, ts, ...
    min_u, max_u, min_bl, max_bl, lags_u, lags_y);

%% --- 3. Ejecutar Optimización ---
fprintf('Optimizando Ki y Zona de Activación...\n');
[best_vars, best_cost] = ga(ObjFcn, 2, [], [], [], [], lb, ub, [], opts);

Ki_opt = best_vars(1);
Umbral_opt = best_vars(2);

fprintf('\n=== RESULTADOS ÓPTIMOS ===\n');
fprintf('Ki (Ganancia):      %.4f\n', Ki_opt);
fprintf('Umbral Activación:  %.4f (W/m^2)\n', Umbral_opt);
fprintf('Costo Final:        %.4f\n', best_cost);

save(fullfile(carpeta_ctrl, 'Params_Hibrido_Condicional.mat'), 'Ki_opt', 'Umbral_opt');

%% --- 4. Simulación Final ---
[y_hib, u_hib, u_int_hist] = simulate_Hybrid_Cond(Ki_opt, Umbral_opt, nf_3in, fis_planta, t, ref_vec, ts, ...
                             min_u, max_u, min_bl, max_bl, lags_u, lags_y);

%% --- 5. Gráficas ---
figure('Name', 'Integración Condicional', 'Color', 'w');

subplot(3,1,1);
plot(t, ref_vec, 'k--', 'LineWidth', 1.5); hold on;
plot(t, y_hib, 'b', 'LineWidth', 1.5);
title(['Salida (Error Estacionario corregido cuando Error < ' num2str(Umbral_opt,'%.1f') ')']); 
ylabel('W/m^2'); grid on; legend('Referencia', 'Salida Híbrida');

subplot(3,1,2);
plot(t, u_hib, 'r', 'LineWidth', 1.5);
title('Esfuerzo de Control Total'); ylabel('PWM'); grid on;

subplot(3,1,3);
plot(t, u_int_hist, 'm', 'LineWidth', 1.5);
yline(0, 'k-');
title('Acción Integral (Solo activa en zona fina)'); 
ylabel('Aporte PWM'); xlabel('Tiempo (s)'); grid on;


%% =========================================================================
%%              FUNCIONES AUXILIARES
%% =========================================================================

function J = fitness_Hybrid_Cond(vars, nf_ctrl, nf_plant, t, ref, ts, ...
                            min_u, max_u, min_bl, max_bl, lags_u, lags_y)
    
    Ki  = vars(1);
    Umbral = vars(2);
    
    [y_hist, ~, ~] = simulate_Hybrid_Cond(Ki, Umbral, nf_ctrl, nf_plant, t, ref, ts, ...
                                     min_u, max_u, min_bl, max_bl, lags_u, lags_y);
    
    if any(isnan(y_hist)) || any(isinf(y_hist))
        J = 1e10; return;
    end
    
    e = ref - y_hist;
    
    % 1. ITAE (Principal)
    J_ITAE = sum(t .* abs(e)) * ts;
    
    % 2. Overshoot
    % Penalizamos fuerte si la integral causa picos al activarse
    val_max = max(y_hist);
    ref_max = max(ref);
    overshoot = max(0, val_max - ref_max);
    
    % 3. Error Final
    err_final = mean(abs(e(end-10:end)));
    
    J = J_ITAE + (100 * overshoot) + (1000 * err_final);
end

function [y_hist, u_hist, u_int_hist] = simulate_Hybrid_Cond(Ki, Umbral, nf_ctrl, nf_plant, t, ref_vec, ts, ...
                                          min_u, max_u, min_bl, max_bl, lags_u, lags_y)
    
    N = length(t);
    y_hist = zeros(1, N); u_hist = zeros(1, N); u_int_hist = zeros(1, N);
    
    y_curr = min_bl; 
    u_prev = min_u; 
    
    buff_y = min_bl * ones(1, lags_y);
    buff_u = min_u * ones(1, lags_u + 1);
    
    integral_accum = 0;
    
    for k = 1:N
        ref = ref_vec(k);
        
        % --- 1. RAMA ANFIS (Principal) ---
        u_prev_n = (u_prev - min_u)/(max_u - min_u);
        y_curr_n = (y_curr - min_bl)/(max_bl - min_bl);
        ref_n    = (ref - min_bl)/(max_bl - min_bl);
        
        inputs_nf = max(0, min(1, [u_prev_n, y_curr_n, ref_n]));
        u_anfis_n = evalfis(nf_ctrl, inputs_nf);
        u_anfis = u_anfis_n * (max_u - min_u) + min_u;
        
        % --- 2. RAMA INTEGRAL CONDICIONAL ---
        error = ref - y_curr;
        
        % LÓGICA DEL USUARIO:
        % Si el error es pequeño (dentro del umbral), ACTIVAR integral.
        % Si el error es grande (transitorio), APAGAR integral (Reset a 0).
        
        if abs(error) <= Umbral
            % Zona de activación
            integral_accum = integral_accum + (Ki * error * ts);
            
            % Opcional: Pequeña saturación de seguridad (ej. +/- 30 PWM)
            % Para que si el error persiste mucho no se sature todo el sistema
            integral_accum = max(-30, min(30, integral_accum));
        else
            % Fuera de zona (Transitorio grande) -> Desactivar
            integral_accum = 0; 
        end
        
        % --- 3. SUMA TOTAL ---
        u_total = u_anfis + integral_accum;
        u_curr = max(min_u, min(max_u, u_total));
        
        % Guardar
        u_hist(k) = u_curr;
        y_hist(k) = y_curr;
        u_int_hist(k) = integral_accum;
        
        % --- 4. SIMULAR PLANTA ---
        buff_u = [u_curr, buff_u(1:end-1)];
        buff_y = [y_curr, buff_y(1:end-1)];
        
        input_plant = [];
        for d = 1:lags_y
            val_n = max(0, min(1, (buff_y(d)-min_bl)/(max_bl-min_bl)));
            input_plant = [input_plant, val_n]; %#ok<AGROW>
        end
        for d = 1:(lags_u + 1)
            val_n = max(0, min(1, (buff_u(d)-min_u)/(max_u-min_u)));
            input_plant = [input_plant, val_n]; %#ok<AGROW>
        end
        
        y_next_n = evalfis(nf_plant, input_plant);
        y_next = y_next_n * (max_bl - min_bl) + min_bl;
        
        if isnan(y_next), break; end
        
        y_curr = y_next;
        u_prev = u_curr;
    end
end