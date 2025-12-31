clear; clc; close all;

% Desactivar advertencias
warning('off', 'fuzzy:general:warnEvalfisInputOutOfRange');

%% --- 1. Configuración y Carga ---
carpeta_data = 'data';
carpeta_ctrl = 'controladores';
carpeta_sim  = 'data_sim';

% A. Cargar Parámetros Generales
if ~isfile(fullfile(carpeta_data, 'norm_params_rojo.mat')), error('Falta norm_params'); end
load(fullfile(carpeta_data, 'norm_params_rojo.mat')); 

% B. Cargar Planta
fis_planta = readfis(fullfile(carpeta_data, 'modelo_rojo_dinamico.fis'));

% C. Cargar Controladores ANFIS
nf_3in = readfis(fullfile(carpeta_ctrl, 'NF_Rojo.fis'));            % Para el Híbrido
nf_4in = readfis(fullfile(carpeta_ctrl, 'NF_Rojo_Integrador.fis')); % Para el NF Puro

% D. Cargar Parámetros del Híbrido (Ki y Umbral)
file_params_hib = fullfile(carpeta_ctrl, 'Params_Hibrido_Condicional.mat');
if isfile(file_params_hib)
    load(file_params_hib); % Carga Ki_opt y Umbral_opt
    fprintf('Parámetros Híbridos cargados: Ki=%.2f, Umbral=%.2f\n', Ki_opt, Umbral_opt);
else
    warning('No se encontraron params híbridos. Usando valores por defecto.');
    Ki_opt = 2.0; Umbral_opt = 5.0;
end

% E. Cargar Rangos para NF 4 Inputs
file_int_range = fullfile(carpeta_sim, 'rangos_integral.mat');
if isfile(file_int_range)
    load(file_int_range); 
else
    max_int_error = (max_bl - min_bl) * 5; min_int_error = -max_int_error;
end

fprintf('Iniciando simulación comparativa...\n');

%% --- 2. Escenario ---
ts = 0.25;           
t_total = 40;       
t = 0:ts:t_total;
N = length(t);

% Referencia
ref_vec = zeros(1, N);
ref_vec(t>=0 & t<10)  = min_bl + (max_bl - min_bl)*0.2; 
ref_vec(t>=10 & t<25) = min_bl + (max_bl - min_bl)*0.7; 
ref_vec(t>=25)        = min_bl + (max_bl - min_bl)*0.4; 

%% --- 3. Inicialización ---
% Sistema A: Híbrido (3 In + Int Ext)
yA_hist = zeros(1,N); yA_curr = min_bl; uA_hist = zeros(1,N); uA_prev = min_u;
buff_yA = min_bl * ones(1, lags_y); buff_uA = min_u * ones(1, lags_u + 1);
int_A = 0; % Integrador externo

% Sistema B: NF Puro (4 In)
yB_hist = zeros(1,N); yB_curr = min_bl; uB_hist = zeros(1,N); uB_prev = min_u;
buff_yB = min_bl * ones(1, lags_y); buff_uB = min_u * ones(1, lags_u + 1);
int_B = 0; % Integrador como input

yA_hist(:)=min_bl; yB_hist(:)=min_bl;

%% --- 4. Bucle de Simulación ---
for k = 1:N
    ref = ref_vec(k);
    
    % =====================================================================
    % SISTEMA A: HÍBRIDO (ANFIS 3 In + Integrador Condicional)
    % =====================================================================
    errA = ref - yA_curr;
    
    % 1. Lógica Condicional del Integrador
    if abs(errA) <= Umbral_opt
        int_A = int_A + (Ki_opt * errA * ts);
        int_A = max(-30, min(30, int_A)); % Saturación de seguridad
    else
        int_A = 0; % Reset en transitorios grandes
    end
    
    % 2. ANFIS (3 Inputs)
    uA_n = (uA_prev - min_u)/(max_u - min_u);
    yA_n = (yA_curr - min_bl)/(max_bl - min_bl);
    rA_n = (ref - min_bl)/(max_bl - min_bl);
    
    inputsA = max(0, min(1, [uA_n, yA_n, rA_n]));
    u_anfis_A = evalfis(nf_3in, inputsA) * (max_u - min_u) + min_u;
    
    % 3. Suma
    uA_curr = u_anfis_A + int_A;
    uA_curr = max(min_u, min(max_u, uA_curr));
    
    % =====================================================================
    % SISTEMA B: NF PURO (ANFIS 4 In)
    % =====================================================================
    errB = ref - yB_curr;
    
    % 1. Acumular Integral (Input)
    if uB_prev < max_u && uB_prev > min_u
        int_B = int_B + (errB * ts);
    else
        int_B = int_B * 0.98; % Anti-windup
    end
    int_B = max(min_int_error, min(max_int_error, int_B));
    
    % 2. Normalizar e Ingresar al ANFIS
    uB_n = (uB_prev - min_u)/(max_u - min_u);
    yB_n = (yB_curr - min_bl)/(max_bl - min_bl);
    rB_n = (ref - min_bl)/(max_bl - min_bl);
    iB_n = (int_B - min_int_error)/(max_int_error - min_int_error);
    
    inputsB = max(0, min(1, [uB_n, yB_n, rB_n, iB_n]));
    uB_curr = evalfis(nf_4in, inputsB) * (max_u - min_u) + min_u;
    uB_curr = max(min_u, min(max_u, uB_curr));
    
    % =====================================================================
    % SIMULACIÓN PLANTA Y ACTUALIZACIÓN
    % =====================================================================
    uA_hist(k) = uA_curr; yA_hist(k) = yA_curr;
    uB_hist(k) = uB_curr; yB_hist(k) = yB_curr;
    
    % Planta A
    buff_uA = [uA_curr, buff_uA(1:end-1)]; buff_yA = [yA_curr, buff_yA(1:end-1)];
    invA = build_narx_input(buff_uA, buff_yA, lags_u, lags_y, min_u, max_u, min_bl, max_bl);
    yA_curr = evalfis(fis_planta, invA) * (max_bl - min_bl) + min_bl;
    uA_prev = uA_curr;
    
    % Planta B
    buff_uB = [uB_curr, buff_uB(1:end-1)]; buff_yB = [yB_curr, buff_yB(1:end-1)];
    invB = build_narx_input(buff_uB, buff_yB, lags_u, lags_y, min_u, max_u, min_bl, max_bl);
    yB_curr = evalfis(fis_planta, invB) * (max_bl - min_bl) + min_bl;
    uB_prev = uB_curr;
end

%% --- 5. Cálculo de Métricas ---
eA = ref_vec - yA_hist;
eB = ref_vec - yB_hist;

% Globales
itaeA = sum(t .* abs(eA)) * ts;
itaeB = sum(t .* abs(eB)) * ts;
rmseA = sqrt(mean(eA.^2));
rmseB = sqrt(mean(eB.^2));
euA   = sum(uA_hist.^2) * ts;
euB   = sum(uB_hist.^2) * ts;

% Escalón (t=10 a 25)
t_start = 10; t_end = 25;
idx_w = t >= t_start & t < t_end;

[EssA, OSA, TsA] = calc_step_metrics(t(idx_w), yA_hist(idx_w), ref_vec(idx_w), t_start);
[EssB, OSB, TsB] = calc_step_metrics(t(idx_w), yB_hist(idx_w), ref_vec(idx_w), t_start);

fprintf('\n======================================================\n');
fprintf('         COMPARATIVA: HÍBRIDO vs NF PURO (4 In)\n');
fprintf('======================================================\n');
fprintf('Métrica                | HÍBRIDO (3In+Int) | NF PURO (4In)\n');
fprintf('-----------------------|-------------------|--------------\n');
fprintf('ITAE (Global)          | %10.2f        | %10.2f\n', itaeA, itaeB);
fprintf('RMSE (Global)          | %10.2f        | %10.2f\n', rmseA, rmseB);
fprintf('Esfuerzo Control (Eu)  | %10.2e        | %10.2e\n', euA, euB);
fprintf('-----------------------|-------------------|--------------\n');
fprintf('Error Estacionario     | %10.4f        | %10.4f\n', EssA, EssB);
fprintf('Sobreimpulso (%%)       | %9.2f %%        | %9.2f %%\n', OSA, OSB);
fprintf('Tiempo Establecim. (s) | %10.4f        | %10.4f\n', TsA, TsB);
fprintf('======================================================\n');

%% --- 6. Gráficas ---
figure('Name', 'Híbrido vs NF 4 Inputs', 'Color', 'w', 'Position', [50 50 1000 800]);

subplot(4,1,1);
plot(t, ref_vec, 'k--', 'LineWidth', 1.5); hold on;
plot(t, yA_hist, 'g', 'LineWidth', 1.5);
plot(t, yB_hist, 'b', 'LineWidth', 1.5);
ylabel('W/m^2'); title('Respuesta Global'); 
legend('Ref', 'Híbrido (3In+Int)', 'NF Puro (4In)'); grid on;

subplot(4,1,2);
plot(t, uA_hist, 'g', 'LineWidth', 1); hold on;
plot(t, uB_hist, 'b', 'LineWidth', 1);
ylabel('PWM'); title('Esfuerzo de Control'); grid on;

subplot(4,1,3);
plot(t, eA, 'g'); hold on; plot(t, eB, 'b'); yline(0, 'k-');
ylabel('Error'); title('Error de Seguimiento'); grid on;

subplot(4,1,4); % ZOOM
idx_z = t > 15 & t < 24; 
plot(t(idx_z), ref_vec(idx_z), 'k--', 'LineWidth', 1.5); hold on;
plot(t(idx_z), yA_hist(idx_z), 'g', 'LineWidth', 2);
plot(t(idx_z), yB_hist(idx_z), 'b', 'LineWidth', 2);
ylabel('W/m^2'); title('ZOOM: Estado Estacionario (Ver Offset)'); grid on;

%% --- AUXILIARES ---
function input_vec = build_narx_input(bu, by, lu, ly, minu, maxu, miny, maxy)
    input_vec = [];
    for d = 1:ly
        val_n = max(0, min(1, (by(d)-miny)/(maxy-miny)));
        input_vec = [input_vec, val_n];
    end
    for d = 1:(lu+1)
        val_n = max(0, min(1, (bu(d)-minu)/(maxu-minu)));
        input_vec = [input_vec, val_n];
    end
end

function [Ess, OS, Ts] = calc_step_metrics(t, y, ref, t_start)
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