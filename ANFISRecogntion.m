%% 1. Configuración y Carga de Datos
clear; clc; close all;

archivo = fullfile('data', 'secuencia_fr.csv');
if ~isfile(archivo)
    error('No se encuentra el archivo %s', archivo);
end

data = readtable(archivo);

% Datos Crudos
u_raw   = data.u;    
bl_raw  = data.fr_l;
time_raw = data.Tiempo_s;

%% 2. Normalización (Min-Max: 0 a 1)
fprintf('Normalizando datos...\n');

min_u = min(u_raw);  max_u = max(u_raw);
min_bl = min(bl_raw); max_bl = max(bl_raw);

u_norm  = (u_raw - min_u) / (max_u - min_u);
bl_norm = (bl_raw - min_bl) / (max_bl - min_bl);

%% 3. CONFIGURACIÓN DE RETARDOS INDEPENDIENTES (NARX)
% =========================================================================
lags_y = 2; % Retardos de SALIDA (Retroalimentación)
lags_u = 2; % Retardos de ENTRADA (Exógeno)
% =========================================================================

max_lag = max(lags_y, lags_u);
n_total = length(u_norm);

if n_total <= max_lag
    error('No hay suficientes datos para el número de retardos solicitados.');
end

num_muestras_validas = n_total - max_lag;
num_entradas = lags_y + (lags_u + 1); 

X_input = zeros(num_muestras_validas, num_entradas);
Y_target = zeros(num_muestras_validas, 1);

fprintf('Construyendo matriz con:\n -> %d retardos de salida (b_l)\n -> %d retardos de entrada (u) + actual\n', lags_y, lags_u);

for k = (max_lag + 1) : n_total
    fila_actual = k - max_lag;
    vector_fila = [];
    
    % 1. Retardos de SALIDA
    if lags_y > 0
        vector_fila = [vector_fila, bl_norm(k-1 : -1 : k-lags_y)']; 
    end
    
    % 2. Retardos de ENTRADA
    vector_fila = [vector_fila, u_norm(k : -1 : k-lags_u)'];
    
    X_input(fila_actual, :) = vector_fila;
    Y_target(fila_actual, 1) = bl_norm(k);
end

data_matrix = [X_input, Y_target];
n_samples = size(data_matrix, 1);

%% 4. División de Datos y Entrenamiento
ratio_train = 0.7;
idx_split = floor(ratio_train * n_samples);

trnData = data_matrix(1:idx_split, :);
chkData = data_matrix(idx_split+1:end, :);

radio_cluster = 0.5; 
opt_gen = genfisOptions('SubtractiveClustering', 'ClusterInfluenceRange', radio_cluster);

fprintf('Generando FIS inicial...\n');
fis_ini = genfis(trnData(:, 1:end-1), trnData(:, end), opt_gen);

opt_anfis = anfisOptions('InitialFIS', fis_ini);
opt_anfis.EpochNumber = 100;
opt_anfis.DisplayANFISInformation = 0;
opt_anfis.ValidationData = chkData;

fprintf('Entrenando modelo ANFIS...\n');
[fis_trained, trainError, stepSize, chkFIS, chkError] = anfis(trnData, opt_anfis);

writeFIS(fis_trained, fullfile('data', 'modelo_fr_dinamico.fis'));
save(fullfile('data', 'norm_params_fr.mat'), 'min_u', 'max_u', 'min_bl', 'max_bl', 'lags_u', 'lags_y');

%% 5. Evaluación y Gráficos

input_eval = chkData(:, 1:end-1);
target_eval = chkData(:, end);

% Evaluar
y_pred_norm = evalfis(fis_trained, input_eval);

% Desnormalizar
y_pred_real = y_pred_norm * (max_bl - min_bl) + min_bl;
target_real = target_eval * (max_bl - min_bl) + min_bl;

columna_u_actual = lags_y + 1; 
u_val_real = input_eval(:, columna_u_actual) * (max_u - min_u) + min_u;

% --- CÁLCULO DE MÉTRICAS DETALLADAS ---

% 1. Vector de errores (Residuos)
residuos = target_real - y_pred_real;
errores_absolutos = abs(residuos);

% 2. RMSE (Raíz del Error Cuadrático Medio) - Estándar global
rmse_global = sqrt(mean(residuos.^2));

% 3. Error Absoluto Promedio (MAE)
error_promedio = mean(errores_absolutos);

% 4. Error Absoluto Mediano (MedAE)
error_mediana = median(errores_absolutos);

% 5. R-Cuadrado
SS_res = sum(residuos.^2);
SS_tot = sum((target_real - mean(target_real)).^2);
R2 = 1 - (SS_res / SS_tot);

% --- GRAFICOS ---

figure('Name', 'Predicción Dinámica', 'NumberTitle', 'off');
subplot(2,1,1);
plot(target_real, 'b', 'LineWidth', 1.5); hold on;
plot(y_pred_real, 'r--', 'LineWidth', 1.5);
legend('Real', 'Predicción');
title(['Serie de Tiempo (RMSE Global: ' num2str(rmse_global, '%.4f') ')']);
ylabel('Luminosidad (W/m^2)'); grid on; xlim([1 length(target_real)]);

subplot(2,1,2);
plot(residuos, 'k');
yline(error_mediana, 'g--', 'Mediana Error'); % Línea visual de la mediana
title('Error Residual'); grid on; xlim([1 length(target_real)]);

figure('Name', 'Ajuste Histeresis', 'NumberTitle', 'off');
plot(u_val_real, target_real, 'bo', 'MarkerSize', 4, 'DisplayName', 'Datos Reales'); hold on;
plot(u_val_real, y_pred_real, 'r.', 'MarkerSize', 6, 'DisplayName', 'Predicción ANFIS');
xlabel('Entrada u(t) (PWM)'); ylabel('Salida b_l(t) (W/m^2)');
title({'Comportamiento Dinámico', ...
       ['R^2: ' num2str(R2, '%.4f')]});
legend('Location', 'best'); grid on;

fprintf('\n--- Resultados Finales ---\n');
fprintf('Configuración: %d retardos salida, %d retardos entrada.\n', lags_y, lags_u);
fprintf('R^2 Final:                 %.4f\n', R2);
fprintf('--------------------------------------\n');
fprintf('RMSE (Global):             %.4f\n', rmse_global);
fprintf('Error Promedio (MAE):      %.4f\n', error_promedio);
fprintf('Error Mediana (MedAE):     %.4f\n', error_mediana);