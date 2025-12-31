clear; clc; close all; rng('shuffle');

% Desactivar advertencias de fuzzy
warning('off', 'fuzzy:general:warnEvalfisInputOutOfRange');

%% --- 1. Configuración ---
carpeta_origen = 'data';
carpeta_destino = 'data_sim';

if ~exist(carpeta_destino, 'dir')
    mkdir(carpeta_destino);
end

% Cargar parámetros y modelo
archivo_params = fullfile(carpeta_origen, 'norm_params_fr.mat');
if ~isfile(archivo_params), error('Falta %s', archivo_params); end
load(archivo_params); 

archivo_fis = fullfile(carpeta_origen, 'modelo_fr_dinamico.fis');
if ~isfile(archivo_fis), error('Falta modelo FIS'); end
fis = readfis(archivo_fis);

fprintf('Generando datos para NARMA-L2...\n');

%% --- 2. Generación de Señal de Excitación (Escalones Aleatorios) ---
% Parámetros de tiempo
ts = 0.25;           % Tiempo de muestreo (igual al del entrenamiento ANFIS)
duracion_escalon_min = 0.5; % Tiempo mínimo por escalón (s)
duracion_escalon_max = 3.0; % Tiempo máximo por escalón (s)
num_escalones = 100;        % Cantidad de cambios de nivel

% Inicialización
u_train = [];
t_acumulado = 0;

for i = 1:num_escalones
    % 1. Determinar duración aleatoria de este escalón
    duracion = duracion_escalon_min + (duracion_escalon_max - duracion_escalon_min) * rand();
    n_puntos = round(duracion / ts);
    
    % 2. Determinar amplitud aleatoria (dentro del rango físico)
    % Se asegura de cubrir todo el rango [min_u, max_u]
    amplitud = min_u + (max_u - min_u) * rand();
    
    % 3. Crear el segmento y añadirlo
    segmento = amplitud * ones(1, n_puntos);
    u_train = [u_train, segmento]; %#ok<AGROW>
end

% Crear vector de tiempo
N = length(u_train);
t_train = (0:N-1) * ts;

% Suavizar ligeramente la entrada (opcional, simula la inductancia/capacitancia real)
% u_train = smoothdata(u_train, 'gaussian', 5); 

fprintf('Señal generada: %d muestras (aprox %.1f segundos).\n', N, t_train(end));

%% --- 3. Simulación de la Planta (ANFIS NARX) ---
y_train = zeros(1, N);
y_train(:) = min_bl; % Condición inicial

% Bucle de simulación idéntico al usado anteriormente
for k = 1:N
    input_vector = [];
    
    % A. Retardos de Salida (y)
    for d = 1:lags_y
        idx = k - d;
        if idx < 1, val = min_bl; else, val = y_train(idx); end
        val_norm = max(0, min(1, (val - min_bl)/(max_bl - min_bl)));
        input_vector = [input_vector, val_norm]; %#ok<AGROW>
    end
    
    % B. Retardos de Entrada (u)
    for d = 0:lags_u
        idx = k - d;
        if idx < 1, val = min_u; else, val = u_train(idx); end
        val_norm = max(0, min(1, (val - min_u)/(max_u - min_u)));
        input_vector = [input_vector, val_norm]; %#ok<AGROW>
    end
    
    % C. Evaluar ANFIS
    y_next_norm = evalfis(fis, input_vector);
    y_next = y_next_norm * (max_bl - min_bl) + min_bl;
    
    y_train(k) = y_next;
end

%% --- 4. Guardar Datos ---
% Para el Neural Network Toolbox, es útil tener los datos como celdas o columnas.
% Guardaremos en formato estándar (.mat)

nombre_archivo = fullfile(carpeta_destino, 'datos_entrenamiento_narma.mat');

% Guardamos vectores fila (1xN)
save(nombre_archivo, 'u_train', 'y_train', 't_train', 'ts', 'min_u', 'max_u', 'min_bl', 'max_bl');

fprintf('Datos guardados en: %s\n', nombre_archivo);

%% --- 5. Graficar ---
figure('Name', 'Datos para NARMA-L2', 'Color', 'w');

subplot(2,1,1);
plot(t_train, y_train, 'b', 'LineWidth', 1.5);
title('Salida de la Planta (Datos Recolectados)');
ylabel('Luminosidad (W/m^2)'); grid on;
xlim([0 t_train(end)]);

subplot(2,1,2);
plot(t_train, u_train, 'r', 'LineWidth', 1.5);
title('Entrada de Excitación (Escalones Aleatorios)');
ylabel('PWM (u)'); xlabel('Tiempo (s)'); grid on;
xlim([0 t_train(end)]); ylim([min_u-5 max_u+5]);