clear; clc; close all; rng('shuffle');

% Desactivar advertencias específicas de lógica difusa
warning('off', 'fuzzy:general:warnEvalfisInputOutOfRange');

%% --- 1) Configuración y Carga de Datos ---
nombre_carpeta_data = 'data';
nombre_carpeta_out  = 'controladores';

if ~exist(nombre_carpeta_out, 'dir')
    mkdir(nombre_carpeta_out);
end

% A. Cargar Parámetros
archivo_params = fullfile(nombre_carpeta_data, 'norm_params_azul.mat');
if ~isfile(archivo_params), error('Falta %s', archivo_params); end
load(archivo_params); 

% B. Cargar Modelo FIS
archivo_fis = fullfile(nombre_carpeta_data, 'modelo_azul_dinamico.fis');
if ~isfile(archivo_fis), error('Falta FIS'); end
fis = readfis(archivo_fis);

% C. Cargar FT
archivo_ft = fullfile(nombre_carpeta_data, 'modelo_ft_azul.mat');
if ~isfile(archivo_ft), error('Falta FT'); end
load(archivo_ft, 'sys_tf');

fprintf('Modelos cargados.\n');

%% --- 2) Modelo Lineal (K y T) ---
K_model = dcgain(sys_tf);           
polos   = pole(sys_tf);             
polo_dom = min(abs(real(polos)));   
T_model = 1 / polo_dom;             

fprintf(' -> SMC Params: K=%.4f, T=%.4f\n', K_model, T_model);

%% --- 3) Parámetros del GA ---
popSize = 50;
numGen  = 50; 
crossoverProb = 0.8;
mutationStd = [0.5, 5.0, 0.2]; 
elitism = 2;

% LÍMITES
bounds = [0.0    17.0;   % Lambda
          0.1    10.0;  % Ks 
          0.1    3.0];   % Phi 

%% --- 4) Escenario ---
Tend = 15;       
dt   = 0.25; 
t    = 0:dt:Tend;

% Referencia alta (40%)
ref_val = (max_bl - min_bl) * 0.4 + min_bl; 
r_vec = ref_val * ones(size(t));

fprintf('Optimizando SMC (ITAE + Offset + OVERSHOOT)...\n');

%% --- 5) Bucle GA ---
pop = zeros(popSize, 3);
for i=1:3
    pop(:,i) = bounds(i,1) + rand(popSize,1)*(bounds(i,2)-bounds(i,1));
end

fit = zeros(popSize, 1);
for i=1:popSize
    fit(i) = fitness_SMC_Corrected(fis, pop(i,:), t, r_vec, ...
             min_u, max_u, min_bl, max_bl, lags_u, lags_y, K_model, T_model);
end

bestHistory = zeros(numGen, 3);

for gen = 1:numGen
    [fit, idx] = sort(fit);
    pop = pop(idx,:);
    
    bestHistory(gen,:) = pop(1,:);
    
    if mod(gen,5)==0 || gen==1
        fprintf('Gen %d/%d | Costo=%.2f | L=%.3f Ks=%.3f Phi=%.3f\n', ...
            gen, numGen, fit(1), pop(1,1), pop(1,2), pop(1,3));
    end
    
    newPop = pop(1:elitism,:);
    while size(newPop,1) < popSize
        p1 = tournamentSelection(pop, fit, 3);
        p2 = tournamentSelection(pop, fit, 3);
        
        if rand < crossoverProb
            alpha = rand;
            child = alpha*p1 + (1-alpha)*p2;
        else
            child = p1;
        end
        
        for k = 1:3
            if rand < 0.3
                child(k) = child(k) + mutationStd(k)*randn;
            end
            child(k) = max(bounds(k,1), min(bounds(k,2), child(k)));
        end
        newPop = [newPop; child];
    end
    pop = newPop;
    
    for i=1:popSize
        fit(i) = fitness_SMC_Corrected(fis, pop(i,:), t, r_vec, ...
                 min_u, max_u, min_bl, max_bl, lags_u, lags_y, K_model, T_model);
    end
end

%% --- 6) Resultados ---
[fit, idx] = sort(fit);
bestVars = pop(idx(1),:);
bestJ = fit(1);

fprintf('\n--- RESULTADOS FINALES ---\n');
fprintf('Mejor Costo: %.4f\n', bestJ);
fprintf('Lambda: %.4f | Ks: %.4f | Phi: %.4f\n', bestVars(1), bestVars(2), bestVars(3));

save(fullfile(nombre_carpeta_out, 'SMC_Azul_Final.mat'), 'bestVars', 'bestJ', 'r_vec', 't');

%% --- 7) Gráficos ---
[y_out, e_out, u_out, u_eq, s_out] = simulate_SMC_NARX(bestVars, fis, t, r_vec, ...
                        min_u, max_u, min_bl, max_bl, lags_u, lags_y, K_model, T_model);

figure('Name', 'SMC Optimizado (Sin Overshoot)', 'Color', 'w');

subplot(3,1,1);
plot(t, r_vec, 'k--', 'LineWidth', 1.5); hold on;
plot(t, y_out, 'b', 'LineWidth', 2);
title('Respuesta Temporal'); ylabel('W/m^2'); grid on; legend('Ref', 'Salida');

subplot(3,1,2);
plot(t, u_out, 'r', 'LineWidth', 1.5); hold on;
plot(t, u_eq, 'g:', 'LineWidth', 1);
title('Control'); ylabel('PWM'); grid on; legend('Total', 'Eq');

subplot(3,1,3);
plot(t, s_out, 'm', 'LineWidth', 1.5);
yline(bestVars(3), 'k:'); yline(-bestVars(3), 'k:');
title('Superficie S'); ylabel('S'); xlabel('Tiempo (s)'); grid on;


%% --- FUNCIONES AUXILIARES ---

function J = fitness_SMC_Corrected(fis, Vars, t, r, min_u, max_u, min_bl, max_bl, lags_u, lags_y, Km, Tm)
    [y_hist, e_hist, ~, ~, ~, is_unstable] = simulate_SMC_NARX(Vars, fis, t, r, ...
                                       min_u, max_u, min_bl, max_bl, lags_u, lags_y, Km, Tm);
    
    if is_unstable
        J = 1e10; 
        return;
    end
    
    % 1. ITAE Estándar
    J_ITAE = trapz(t, t .* abs(e_hist));
    
    % 2. Penalización por ERROR FINAL (Offset)
    error_final = mean(abs(e_hist(end-10:end)));
    W_offset = 100; 
    
    % 3. Penalización por SOBREIMPULSO (Overshoot) --- NUEVO ---
    val_max = max(y_hist);
    ref_final = r(end);
    
    if val_max > ref_final
        overshoot = val_max - ref_final;
    else
        overshoot = 0;
    end
    
    W_overshoot = 100; % Peso alto para prohibir el sobreimpulso
    
    % Costo Total
    J = J_ITAE + (W_offset * error_final) + (W_overshoot * overshoot^2);
end

function [y_hist, e_hist, u_hist, ueq_hist, s_hist, is_unstable] = simulate_SMC_NARX(...
    Vars, fis, t, r, min_u, max_u, min_bl, max_bl, lags_u, lags_y, Km, Tm)

    lambda = Vars(1);
    ks     = Vars(2);
    phi    = Vars(3);

    dt = t(2)-t(1);
    N = length(t);
    
    y_hist = zeros(1, N); u_hist = zeros(1, N);
    e_hist = zeros(1, N); ueq_hist = zeros(1, N); s_hist = zeros(1, N);
    
    y_hist(:) = min_bl; u_hist(:) = min_u;
    
    e_integral = 0;
    is_unstable = false;
    
    for k = 1:N
        % Medida actual (con retardo de un paso)
        if k > 1, y_curr = y_hist(k-1); else, y_curr = min_bl; end
        
        % 1. Error
        e = r(k) - y_curr;
        
        % 2. Anti-Windup ESTRICTO
        u_prev_step = u_hist(max(1,k-1));
        
        is_saturated = (u_prev_step >= max_u && e > 0) || ...
                       (u_prev_step <= min_u && e < 0);
        
        if ~is_saturated
            e_integral = e_integral + e * dt;
        end
        
        % 3. Superficie
        s = e + lambda * e_integral;
        
        % 4. Control Equivalente
        u_eq = (y_curr + Tm * lambda * e) / Km;
        
        % 5. Control Switching (Tanh)
        u_sw = ks * tanh(s / phi);
        
        % 6. Total
        u_total = u_eq + u_sw;
        u_curr = max(min_u, min(max_u, u_total));
        
        % Guardar
        u_hist(k) = u_curr;
        e_hist(k) = e;
        ueq_hist(k) = u_eq;
        s_hist(k) = s;
        
        % 7. ANFIS (NARX) con CLAMPING
        input_vector = [];
        for d = 1:lags_y
            idx = k - d;
            if idx < 1, val = min_bl; else, val = y_hist(idx); end
            val_norm = max(0, min(1, (val - min_bl)/(max_bl - min_bl))); 
            input_vector = [input_vector, val_norm]; %#ok<AGROW>
        end
        for d = 0:lags_u
            idx = k - d;
            if idx < 1, val = min_u; else, val = u_hist(idx); end
            val_norm = max(0, min(1, (val - min_u)/(max_u - min_u))); 
            input_vector = [input_vector, val_norm]; %#ok<AGROW>
        end
        
        y_next_norm = evalfis(fis, input_vector);
        y_next = y_next_norm * (max_bl - min_bl) + min_bl;
        
        if isnan(y_next) || isinf(y_next) || abs(y_next) > (max_bl*3)
            is_unstable = true; return;
        end
        y_hist(k) = y_next; 
    end
end

function parent = tournamentSelection(pop, fit, tam)
    n = size(pop,1);
    inds = randi(n, tam, 1);
    [~, localIdx] = min(fit(inds));
    parent = pop(inds(localIdx), :);
end