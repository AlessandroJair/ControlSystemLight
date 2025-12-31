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

fprintf('Modelos cargados. Iniciando optimización BELBIC (Multi-Step)...\n');

%% --- 2. Escenario de Simulación (Multi-Step) ---
ts = 0.1;
t_total = 50; % Tiempo aumentado para permitir múltiples cambios
t = 0:ts:t_total;
N = length(t);

% Referencia: Escalera Dinámica
ref_vec = zeros(1, N);

% 1. Arranque suave (20%) -> Prueba sensibilidad en zona baja
ref_vec(t>=0 & t<15)  = min_bl + (max_bl - min_bl) * 0.20; 

% 2. Subida fuerte (80%) -> Prueba saturación y evita windup en pesos
ref_vec(t>=15 & t<35) = min_bl + (max_bl - min_bl) * 0.80; 

% 3. Bajada media (50%) -> Prueba capacidad de inhibición (frenado)
ref_vec(t>=35)        = min_bl + (max_bl - min_bl) * 0.50; 

%% --- 3. Parámetros del GA ---
% Variables: [Alpha, Beta, K_sensorial, K_recompensa]
lb = [1e-6,  1e-6,  0.01,  0.01];
ub = [0.1,   0.1,   10.0,  10.0];

opts = optimoptions('ga', ...
    'PopulationSize', 50, ...
    'MaxGenerations', 50, ...
    'Display', 'iter', ...
    'UseParallel', false, ...
    'PlotFcn', @gaplotbestf);

% Función de Costo
ObjFcn = @(vars) fitness_BELBIC_Multi(vars, fis, t, ref_vec, ts, ...
    min_u, max_u, min_bl, max_bl, lags_u, lags_y);

%% --- 4. Ejecutar Optimización ---
fprintf('Optimizando parámetros para toda la secuencia...\n');
[best_vars, best_cost] = ga(ObjFcn, 4, [], [], [], [], lb, ub, [], opts);

alpha_opt = best_vars(1);
beta_opt  = best_vars(2);
Ks_opt    = best_vars(3);
Kr_opt    = best_vars(4);

fprintf('\n=== RESULTADOS BELBIC MULTI-STEP ===\n');
fprintf('Alpha (Amygdala): %.6f\n', alpha_opt);
fprintf('Beta  (Orbito):   %.6f\n', beta_opt);
fprintf('K_sensor: %.4f | K_reward: %.4f\n', Ks_opt, Kr_opt);
fprintf('Costo Final: %.4f\n', best_cost);

save(fullfile(carpeta_out, 'BELBIC_FR_Optimizado.mat'), ...
    'alpha_opt', 'beta_opt', 'Ks_opt', 'Kr_opt');

%% --- 5. Simulación Final y Gráficas ---
[y_out, u_out, nodes_out] = simulate_BELBIC(best_vars, fis, t, ref_vec, ts, ...
                            min_u, max_u, min_bl, max_bl, lags_u, lags_y);

amygdala = nodes_out(:,1);
orbito   = nodes_out(:,2);

figure('Name', 'BELBIC Optimizado Multi-Step', 'Color', 'w');

subplot(3,1,1);
plot(t, ref_vec, 'k--', 'LineWidth', 1.5); hold on;
plot(t, y_out, 'b', 'LineWidth', 1.5);
title(['Respuesta Multi-Step (Costo=' num2str(best_cost,'%.1f') ')']); 
ylabel('W/m^2'); legend('Referencia', 'BELBIC'); grid on;

subplot(3,1,2);
plot(t, u_out, 'r', 'LineWidth', 1.5);
title('Esfuerzo de Control (PWM)'); ylabel('u'); grid on;

subplot(3,1,3);
plot(t, amygdala, 'g', 'LineWidth', 1); hold on;
plot(t, orbito, 'm', 'LineWidth', 1);
title('Dinámica Emocional (Pesos)'); 
ylabel('Valor'); xlabel('Tiempo (s)'); 
legend('Amígdala (V)', 'Corteza (W)'); grid on;


%% =========================================================================
%%              FUNCIONES AUXILIARES
%% =========================================================================

function J = fitness_BELBIC_Multi(vars, fis, t, ref, ts, min_u, max_u, min_bl, max_bl, lags_u, lags_y)
    
    [y_hist, ~, ~] = simulate_BELBIC(vars, fis, t, ref, ts, ...
                                     min_u, max_u, min_bl, max_bl, lags_u, lags_y);
    
    if any(isnan(y_hist)) || any(isinf(y_hist))
        J = 1e10; return;
    end
    
    e = ref - y_hist;
    
    % 1. ITAE (Error acumulado ponderado por tiempo)
    % Penaliza más los errores tardíos en cada escalón
    J_ITAE = sum(t .* abs(e)) * ts;
    
    % 2. Penalización de Overshoot Global
    % Buscamos picos que superen la referencia en cualquier momento
    overshoot_acum = sum(max(0, y_hist - ref));
    
    % 3. Penalización de Oscilación (Smoothness)
    % Importante para que los pesos V y W no se vuelvan locos
    diff_y = diff(y_hist);
    J_Osc = sum(abs(diff_y));
    
    % Pesos de la función de costo
    J = J_ITAE + (100 * overshoot_acum) + (10 * J_Osc);
end

function [y_hist, u_hist, nodes_hist] = simulate_BELBIC(vars, fis, t, ref_vec, ts, ...
                                          min_u, max_u, min_bl, max_bl, lags_u, lags_y)
    % Desempaquetar
    alpha = vars(1); 
    beta  = vars(2); 
    Ks    = vars(3); 
    Kr    = vars(4); 
    
    N = length(t);
    y_hist = zeros(1, N); u_hist = zeros(1, N);
    nodes_hist = zeros(N, 2); 
    
    y_curr = min_bl; 
    u_prev = min_u;
    
    buff_y = min_bl * ones(1, lags_y);
    buff_u = min_u * ones(1, lags_u + 1);
    
    V = 0; % Amigdala
    W = 0; % Corteza
    
    for k = 1:N
        ref = ref_vec(k);
        e = ref - y_curr;
        
        % --- BELBIC ---
        % Entrada Sensorial: Combinación de error y referencia a veces ayuda, 
        % pero Ks * e es la forma canónica para control de seguimiento.
        SI = Ks * e; 
        
        % Recompensa (Estrés): Queremos e=0.
        REW = Kr * abs(e); 
        
        % Nodos
        A_node = V * SI;
        O_node = W * SI;
        
        % Salida del modelo (Acción incremental)
        MO = A_node - O_node;
        
        % Aprendizaje (Regla estándar BELBIC)
        % max(0, ...) en la amígdala asegura que el peso V no decrezca (biomimético),
        % aunque en control a veces se permite que decrezca. Usaremos la versión estándar.
        dV = alpha * SI * (REW - A_node); 
        dW = beta * SI * (MO - REW);
        
        V = V + dV;
        W = W + dW;
        
        % Control
        u_curr = u_prev + MO;
        u_curr = max(min_u, min(max_u, u_curr));
        
        % --- Guardar ---
        u_hist(k) = u_curr;
        y_hist(k) = y_curr;
        nodes_hist(k,:) = [A_node, O_node];
        
        % --- ANFIS ---
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