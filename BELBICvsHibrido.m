clear; clc; close all;
warning('off', 'fuzzy:general:warnEvalfisInputOutOfRange');

%% --- 1. Carga de Modelos y Parámetros ---
carpeta_data = 'data';
carpeta_ctrl = 'controladores';
carpeta_sim  = 'data_sim';

% Parametros generales
if ~isfile(fullfile(carpeta_data, 'norm_params_rojo.mat')), error('Falta norm_params'); end
load(fullfile(carpeta_data, 'norm_params_rojo.mat')); 

fis_planta = readfis(fullfile(carpeta_data, 'modelo_rojo_dinamico.fis'));

% A. Cargar HÍBRIDO (Neuro-Fuzzy + Integrador)
if ~isfile(fullfile(carpeta_ctrl, 'NF_Rojo.fis')), error('Falta NF_Rojo.fis'); end
nf_3in = readfis(fullfile(carpeta_ctrl, 'NF_Rojo.fis'));

if ~isfile(fullfile(carpeta_ctrl, 'Params_Hibrido_Condicional.mat'))
    Ki_hib = 2.0; Umb_hib = 5.0; warning('Usando valores Híbrido defecto');
else
    load(fullfile(carpeta_ctrl, 'Params_Hibrido_Condicional.mat')); % Carga Ki_opt, Umbral_opt
    Ki_hib = Ki_opt; Umb_hib = Umbral_opt;
end

% B. Cargar BELBIC
if ~isfile(fullfile(carpeta_ctrl, 'BELBIC_Rojo_Optimizado.mat'))
    error('Falta BELBIC_Optimizado.mat. Ejecuta la optimización primero.');
end
load(fullfile(carpeta_ctrl, 'BELBIC_Rojo_Optimizado.mat')); % Carga alpha_opt, beta_opt, Ks_opt, Kr_opt

fprintf('Controladores cargados. Iniciando simulación...\n');

%% --- 2. Escenario de Prueba (Multi-Step) ---
ts = 0.25;
t_total = 60;
t = 0:ts:t_total;
N = length(t);

ref = zeros(1, N);
ref(t>=0  & t<15) = min_bl + (max_bl - min_bl) * 0.3; % 30%
ref(t>=15 & t<30) = min_bl + (max_bl - min_bl) * 0.8; % 80% (Escalón de Análisis)
ref(t>=30 & t<45) = min_bl + (max_bl - min_bl) * 0.5; % 50%
ref(t>=45)        = min_bl + (max_bl - min_bl) * 0.2; % 20%

%% --- 3. Simulación Híbrido ---
yH = zeros(1,N); uH = zeros(1,N);
y_curr = min_bl; u_prev = min_u;
buff_y = min_bl * ones(1, lags_y); buff_u = min_u * ones(1, lags_u + 1);
integral_H = 0;

for k = 1:N
    r = ref(k);
    err = r - y_curr;
    
    % Integrador Condicional
    if abs(err) <= Umb_hib
        integral_H = integral_H + (Ki_hib * err * ts);
        integral_H = max(-30, min(30, integral_H));
    else
        integral_H = 0;
    end
    
    % ANFIS
    u_n = (u_prev - min_u)/(max_u - min_u);
    y_n = (y_curr - min_bl)/(max_bl - min_bl);
    r_n = (r - min_bl)/(max_bl - min_bl);
    ins = max(0, min(1, [u_n, y_n, r_n]));
    u_anfis = evalfis(nf_3in, ins) * (max_u - min_u) + min_u;
    
    u_curr = max(min_u, min(max_u, u_anfis + integral_H));
    
    % Planta
    uH(k) = u_curr; yH(k) = y_curr;
    buff_u = [u_curr, buff_u(1:end-1)]; buff_y = [y_curr, buff_y(1:end-1)];
    input_p = build_input(buff_u, buff_y, lags_u, lags_y, min_u, max_u, min_bl, max_bl);
    y_curr = evalfis(fis_planta, input_p) * (max_bl - min_bl) + min_bl;
    u_prev = u_curr;
end

%% --- 4. Simulación BELBIC ---
yB = zeros(1,N); uB = zeros(1,N);
y_curr = min_bl; u_prev = min_u;
buff_y = min_bl * ones(1, lags_y); buff_u = min_u * ones(1, lags_u + 1);
V = 0; W = 0; 

for k = 1:N
    r = ref(k);
    e = r - y_curr;
    
    % BELBIC
    SI = Ks_opt * e; 
    REW = Kr_opt * abs(e);
    
    A_node = V * SI;
    O_node = W * SI;
    MO = A_node - O_node;
    
    dV = alpha_opt * SI * (REW - A_node);
    dW = beta_opt * SI * (MO - REW);
    
    V = V + dV; W = W + dW;
    
    u_curr = max(min_u, min(max_u, u_prev + MO));
    
    % Planta
    uB(k) = u_curr; yB(k) = y_curr;
    buff_u = [u_curr, buff_u(1:end-1)]; buff_y = [y_curr, buff_y(1:end-1)];
    input_p = build_input(buff_u, buff_y, lags_u, lags_y, min_u, max_u, min_bl, max_bl);
    y_curr = evalfis(fis_planta, input_p) * (max_bl - min_bl) + min_bl;
    u_prev = u_curr;
end

%% --- 5. Cálculo de Métricas (Análisis del Escalón Principal) ---
% Analizamos el tramo de t=15 a t=30 (Subida grande)
t_start = 15; 
t_end   = 30;
idx_w   = t >= t_start & t < t_end;

% Extraer datos ventana
t_win   = t(idx_w);
ref_win = ref(idx_w);
yH_win  = yH(idx_w);
yB_win  = yB(idx_w);

% Calcular Métricas Específicas
[Ess_H, OS_H, Ts_H] = calc_metrics(t_win, yH_win, ref_win, t_start);
[Ess_B, OS_B, Ts_B] = calc_metrics(t_win, yB_win, ref_win, t_start);

% Métricas Globales
ITAE_H = sum(t .* abs(ref - yH)) * ts;
ITAE_B = sum(t .* abs(ref - yB)) * ts;

fprintf('\n======================================================\n');
fprintf('         COMPARATIVA FINAL: HÍBRIDO vs BELBIC\n');
fprintf('======================================================\n');
fprintf('Métrica                | HÍBRIDO (NF+Int) | BELBIC (Bio)\n');
fprintf('-----------------------|------------------|-------------\n');
fprintf('ITAE (Global)          | %10.2f       | %10.2f\n', ITAE_H, ITAE_B);
fprintf('-----------------------|------------------|-------------\n');
fprintf('Error Estacionario     | %10.4f       | %10.4f\n', Ess_H, Ess_B);
fprintf('Sobreimpulso (%%)       | %9.2f %%       | %9.2f %%\n', OS_H, OS_B);
fprintf('Tiempo Establecim. (s) | %10.4f       | %10.4f\n', Ts_H, Ts_B);
fprintf('======================================================\n');

%% --- 6. Gráficas ---
figure('Name', 'Batalla Final: Híbrido vs BELBIC', 'Color', 'w', 'Position', [100 100 1000 600]);

subplot(3,1,1);
plot(t, ref, 'k--', 'LineWidth', 1.5); hold on;
plot(t, yH, 'g', 'LineWidth', 1.5);
plot(t, yB, 'm', 'LineWidth', 1.5);
title('Respuesta de Salida Global'); ylabel('W/m^2'); 
legend('Referencia', 'Híbrido', 'BELBIC'); grid on;

subplot(3,1,2);
plot(t, uH, 'g', 'LineWidth', 1); hold on;
plot(t, uB, 'm', 'LineWidth', 1);
title('Esfuerzo de Control'); ylabel('PWM'); grid on;

subplot(3,1,3); % ZOOM AL ESCALÓN
plot(t_win, ref_win, 'k--', 'LineWidth', 1.5); hold on;
plot(t_win, yH_win, 'g', 'LineWidth', 2);
plot(t_win, yB_win, 'm', 'LineWidth', 2);
xlim([15 25]); % Zoom en la transición
title('Zoom: Análisis Transitorio (Overshoot y Settling)'); ylabel('W/m^2'); grid on;

%% --- FUNCIONES AUXILIARES ---
function v = build_input(bu, by, lu, ly, minu, maxu, miny, maxy)
    v = [];
    for d=1:ly, v=[v, (by(d)-miny)/(maxy-miny)]; end
    for d=1:lu+1, v=[v, (bu(d)-minu)/(maxu-minu)]; end
    v = max(0, min(1, v));
end

function [Ess, OS, Ts] = calc_metrics(t, y, ref, t_start)
    % 1. Error Estacionario (Promedio últimos 10% muestras)
    n = length(y);
    idx_steady = round(n*0.9):n;
    y_final = mean(y(idx_steady));
    r_final = mean(ref(idx_steady));
    Ess = abs(r_final - y_final);
    
    % 2. Sobreimpulso
    y_initial = y(1);
    step_size = abs(r_final - y_initial);
    [y_max, ~] = max(y);
    
    if y_max > r_final && step_size > 0.1
        OS = ((y_max - r_final) / step_size) * 100;
    else
        OS = 0;
    end
    
    % 3. Tiempo de Establecimiento (2%)
    tol = 0.02 * step_size;
    err_abs = abs(y - r_final);
    idx_in = find(err_abs > tol, 1, 'last');
    
    if isempty(idx_in)
        Ts = 0; 
    else
        if idx_in < length(t)
            Ts = t(idx_in+1) - t_start;
        else
            Ts = t(end) - t_start; 
        end
    end
    if Ts < 0, Ts=0; end
end