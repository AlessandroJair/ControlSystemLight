clear; clc; close all; rng('shuffle');
warning('off', 'fuzzy:general:warnEvalfisInputOutOfRange');

%% --- 1. Configuración y Carga ---
carpeta_data = 'data';
carpeta_out  = 'controladores';
if ~exist(carpeta_out, 'dir'), mkdir(carpeta_out); end

% Cargar Archivos
if ~isfile(fullfile(carpeta_data, 'norm_params_fr.mat')), error('Falta norm_params'); end
load(fullfile(carpeta_data, 'norm_params_fr.mat')); 

if ~isfile(fullfile(carpeta_data, 'modelo_fr_dinamico.fis')), error('Falta modelo FIS'); end
fis = readfis(fullfile(carpeta_data, 'modelo_fr_dinamico.fis'));

fprintf('Modelos cargados. Iniciando optimización PID...\n');

%% --- 2. Escenario de Simulación ---
ts = 0.25;
t_total = 30;
t = 0:ts:t_total;
N = length(t);

% Referencia Dinámica
ref_vec = zeros(1, N);
ref_vec(t>=0  & t<15) = min_bl + (max_bl - min_bl) * 0.25; % Zona Baja
ref_vec(t>=15 & t<35) = min_bl + (max_bl - min_bl) * 0.75; % Zona Alta (Saturación)
ref_vec(t>=35)        = min_bl + (max_bl - min_bl) * 0.40; % Bajada

%% --- 3. Configuración del GA ---
% Variables: [Kp, Ki, Kd]
lb = [0.1,  0.01,  0.0]; 
ub = [15.0, 10.0,  2.0]; 

opts = optimoptions('ga', ...
    'PopulationSize', 50, ...
    'MaxGenerations', 50, ...
    'Display', 'iter', ...
    'UseParallel', false, ...
    'PlotFcn', @gaplotbestf);

ObjFcn = @(vars) fitness_PID(vars, fis, t, ref_vec, ts, ...
    min_u, max_u, min_bl, max_bl, lags_u, lags_y);

%% --- 4. Ejecutar Optimización ---
fprintf('Sintonizando ganancias PID...\n');
[best_vars, best_cost] = ga(ObjFcn, 3, [], [], [], [], lb, ub, [], opts);

Kp_opt = best_vars(1);
Ki_opt = best_vars(2);
Kd_opt = best_vars(3);

fprintf('\n=== RESULTADOS PID ===\n');
fprintf('Kp: %.4f\n', Kp_opt);
fprintf('Ki: %.4f\n', Ki_opt);
fprintf('Kd: %.4f\n', Kd_opt);
fprintf('Costo: %.4f\n', best_cost);

save(fullfile(carpeta_out, 'PID_Rojo_Optimizado.mat'), 'Kp_opt', 'Ki_opt', 'Kd_opt');

%% --- 5. Simulación Final y Métricas ---
[y_out, u_out, e_out] = simulate_PID(best_vars, fis, t, ref_vec, ts, ...
                        min_u, max_u, min_bl, max_bl, lags_u, lags_y);

% Cálculo de Métricas (Escalón principal t=15 a 35)
t_start = 15; t_end = 35;
idx_w = t >= t_start & t < t_end;
[Ess, OS, Ts_val] = calc_metrics(t(idx_w), y_out(idx_w), ref_vec(idx_w), t_start);
ITAE_val = sum(t .* abs(e_out)) * ts;

fprintf('\n--- MÉTRICAS DE DESEMPEÑO ---\n');
fprintf('ITAE Global:       %.2f\n', ITAE_val);
fprintf('Error Estacionario: %.4f\n', Ess);
fprintf('Sobreimpulso:       %.2f %%\n', OS);
fprintf('Tiempo Establecim.: %.2f s\n', Ts_val);

%% --- 6. Gráficas ---
figure('Name', 'PID Optimizado', 'Color', 'w');

subplot(3,1,1);
plot(t, ref_vec, 'k--', 'LineWidth', 1.5); hold on;
plot(t, y_out, 'b', 'LineWidth', 1.5);
title('Respuesta de Salida'); ylabel('W/m^2'); legend('Referencia', 'PID'); grid on;

subplot(3,1,2);
plot(t, u_out, 'r', 'LineWidth', 1.5);
title('Esfuerzo de Control'); ylabel('PWM'); grid on;

subplot(3,1,3);
plot(t, e_out, 'm', 'LineWidth', 1); yline(0, 'k-');
title('Error de Seguimiento'); ylabel('Error'); xlabel('Tiempo (s)'); grid on;


%% =========================================================================
%%              FUNCIONES AUXILIARES
%% =========================================================================

function J = fitness_PID(vars, fis, t, ref, ts, min_u, max_u, min_bl, max_bl, lags_u, lags_y)
    
    [y_hist, u_hist, e_hist] = simulate_PID(vars, fis, t, ref, ts, ...
                               min_u, max_u, min_bl, max_bl, lags_u, lags_y);
    
    if any(isnan(y_hist)) || any(isinf(y_hist))
        J = 1e10; return;
    end
    
    % 1. ITAE
    J_ITAE = sum(t .* abs(e_hist)) * ts;
    
    % 2. Penalización Overshoot
    overshoot_sum = sum(max(0, y_hist - ref));
    
    % 3. Penalización Suavidad (Chattering en el control)
    % Importante para que la derivada (Kd) no meta ruido excesivo
    du = sum(abs(diff(u_hist)));
    
    J = J_ITAE + (100 * overshoot_sum) + (0.5 * du);
end

function [y_hist, u_hist, e_hist] = simulate_PID(vars, fis, t, ref_vec, ts, ...
                                      min_u, max_u, min_bl, max_bl, lags_u, lags_y)
    Kp = vars(1);
    Ki = vars(2);
    Kd = vars(3);
    
    N = length(t);
    y_hist = zeros(1, N); u_hist = zeros(1, N); e_hist = zeros(1, N);
    
    y_curr = min_bl; 
    u_prev = min_u;
    
    % Buffers ANFIS
    buff_y = min_bl * ones(1, lags_y);
    buff_u = min_u * ones(1, lags_u + 1);
    
    integral = 0;
    prev_error = 0;
    
    for k = 1:N
        ref = ref_vec(k);
        error = ref - y_curr;
        
        % --- PID CON ANTI-WINDUP (Clamping) ---
        
        % 1. Término Proporcional
        P = Kp * error;
        
        % 2. Término Integral (Condicional)
        % Solo integramos si no estamos saturados
        % (Predecimos si la acción futura saturará)
        possible_u = u_prev + (Ki * error * ts); % Estimación simple
        
        if (u_prev >= max_u && error > 0) || (u_prev <= min_u && error < 0)
            % Saturado y el error quiere empujar más -> NO INTEGRAR
        else
            integral = integral + (error * ts);
        end
        I = Ki * integral;
        
        % 3. Término Derivativo
        % Derivada sobre la medición (y) es mejor para evitar picos en escalones,
        % pero usaremos sobre el error (estándar) con filtrado si fuera real.
        % Aquí: Derivada simple.
        derivative = (error - prev_error) / ts;
        D = Kd * derivative;
        
        % 4. Salida PID
        u_calc = P + I + D;
        
        % Saturación Física
        u_curr = max(min_u, min(max_u, u_calc));
        
        % Guardar
        y_hist(k) = y_curr;
        u_hist(k) = u_curr;
        e_hist(k) = error;
        prev_error = error;
        
        % --- Simulación Planta (ANFIS) ---
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
        
        y_next_n = evalfis(fis, input_plant);
        y_next = y_next_n * (max_bl - min_bl) + min_bl;
        
        if isnan(y_next), break; end
        y_curr = y_next;
        u_prev = u_curr;
    end
end

function [Ess, OS, Ts] = calc_metrics(t, y, ref, t_start)
    n = length(y);
    idx_steady = round(n*0.9):n;
    y_final = mean(y(idx_steady));
    r_final = mean(ref(idx_steady));
    Ess = abs(r_final - y_final);
    
    step_size = abs(r_final - y(1));
    [y_max, ~] = max(y);
    if y_max > r_final && step_size > 0.1
        OS = ((y_max - r_final) / step_size) * 100;
    else
        OS = 0;
    end
    
    tol = 0.02 * step_size; 
    err_abs = abs(y - r_final);
    idx_in = find(err_abs > tol, 1, 'last');
    
    if isempty(idx_in), Ts = 0; else
        if idx_in < length(t), Ts = t(idx_in+1) - t_start; else, Ts = t(end) - t_start; end
    end
    if Ts < 0, Ts=0; end
end