%% 1. Configuración y Carga de Datos
clear; clc; close all;

archivo = fullfile('data', 'secuencia_azul.csv');
if ~isfile(archivo)
    error('No se encuentra el archivo %s', archivo);
end

data_table = readtable(archivo);

% Datos Físicos (Sin normalizar para conservar unidades reales)
u_raw   = data_table.u;      % Entrada (PWM)
y_raw   = data_table.b_l;    % Salida (W/m^2)
t_raw   = data_table.Tiempo_s;

% Calcular el Tiempo de Muestreo (Ts) promedio
Ts = mean(diff(t_raw));
fprintf('Tiempo de muestreo detectado (Ts): %.4f s\n', Ts);

%% 2. Creación del Objeto iddata
% El toolbox de identificación requiere objetos 'iddata'
datos_id = iddata(y_raw, u_raw, Ts);

% Asignar nombres para gráficos automáticos
datos_id.InputName  = 'PWM';
datos_id.InputUnit  = 'u';
datos_id.OutputName = 'Luminosidad';
datos_id.OutputUnit = 'W/m^2';
datos_id.TimeUnit   = 'seconds';

% Opcional: Eliminar offsets (tendencias) si los datos no empiezan en 0
% Para sistemas de luces, a veces es mejor dejar que tfest estime el nivel DC.
% datos_id = detrend(datos_id, 'constant'); 

%% 3. División de Datos (Train / Validation)
ratio_train = 0.7;
n_samples = length(u_raw);
idx_split = floor(ratio_train * n_samples);

data_train = datos_id(1:idx_split);
data_valid = datos_id(idx_split+1:end);

fprintf('Datos totales: %d | Train: %d | Validation: %d\n', ...
    n_samples, size(data_train,1), size(data_valid,1));

%% 4. Estimación de la Función de Transferencia
% Configuración: Número de Polos y Ceros
% Un sistema térmico/lumínico suele ser de 1er o 2do orden.
np = 1; % Número de polos (prueba con 2 si el ajuste es pobre)
nz = 0; % Número de ceros

% Opciones: Estimación de la condición inicial para mejorar el ajuste
opt = tfestOptions('InitialCondition', 'estimate', 'Display', 'on');

fprintf('\nEstimando Función de Transferencia (%d polos, %d ceros)...\n', np, nz);
sys_tf = tfest(data_train, np, nz, opt);

% Mostrar la función obtenida
fprintf('\n--- Función de Transferencia Identificada ---\n');
sys_tf

% Guardar el modelo
save(fullfile('data', 'modelo_ft_azul.mat'), 'sys_tf', 'Ts');

%% 5. Validación y Métricas
% Simulamos el modelo con la entrada de VALIDACIÓN
% 'lsim' simula la respuesta temporal del sistema lineal
[y_pred, t_valid] = lsim(sys_tf, data_valid.u, data_valid.SamplingInstants);

% Datos reales de validación (vector numérico)
y_real = data_valid.y;

% --- Cálculo de Métricas ---
residuos = y_real - y_pred;

% 1. RMSE
rmse = sqrt(mean(residuos.^2));

% 2. R-Cuadrado (R2)
SS_res = sum(residuos.^2);
SS_tot = sum((y_real - mean(y_real)).^2);
R2 = 1 - (SS_res / SS_tot);

% 3. NRMSE (Normalized RMSE) o "Fit Percentage" clásico de Matlab
% Fit = 100 * (1 - norm(y_real - y_pred) / norm(y_real - mean(y_real)))
fit_percent = 100 * (1 - (norm(residuos) / norm(y_real - mean(y_real))));

fprintf('\n--- Resultados en Validación ---\n');
fprintf('FIT (%%):  %.2f%%\n', fit_percent);
fprintf('R^2:      %.4f\n', R2);
fprintf('RMSE:     %.4f W/m^2\n', rmse);

%% 6. Gráficos

% --- Figura 1: Comparación Serie de Tiempo (Validación) ---
figure('Name', 'Validación FT', 'NumberTitle', 'off');
subplot(2,1,1);
plot(data_valid.SamplingInstants, y_real, 'b', 'LineWidth', 1.5); hold on;
plot(data_valid.SamplingInstants, y_pred, 'r--', 'LineWidth', 1.5);
title({['Validación del Modelo Lineal (' num2str(np) ' polos)'], ...
       ['Fit: ' num2str(fit_percent, '%.2f') '% | R^2: ' num2str(R2, '%.4f')]});
legend('Datos Reales', 'Predicción FT');
ylabel('Luminosidad (W/m^2)'); grid on;

subplot(2,1,2);
plot(data_valid.SamplingInstants, residuos, 'k');
title('Error Residual');
xlabel('Tiempo (s)'); ylabel('Error'); grid on;

% --- Figura 2: Respuesta al Escalón (Step Response) ---
% Esto te dice cómo reacciona el sistema ante un cambio brusco
figure('Name', 'Respuesta al Escalón', 'NumberTitle', 'off');
step(sys_tf); 
title('Respuesta al Escalón Unitario (Dinámica del Sistema)');
grid on;

% --- Figura 3: Mapa de Polos y Ceros ---
% Para ver estabilidad (deben estar a la izquierda en plano S)
figure('Name', 'Mapa Polos-Ceros', 'NumberTitle', 'off');
pzmap(sys_tf);
title('Mapa de Polos y Ceros (Estabilidad)');
grid on;