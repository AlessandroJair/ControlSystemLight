% Limpiar espacio de trabajo y cerrar figuras previas
clear; clc; close all;

% ==========================================
% 1. CONFIGURACIÓN DE RUTAS Y DATOS
% ==========================================
nombre_carpeta = 'data';
archivo_entrada = fullfile(nombre_carpeta, 'NEW_DATA_LED.csv');

% Verificar existencia del archivo
if ~isfile(archivo_entrada)
    error('No se encuentra el archivo en: %s', archivo_entrada);
end

% Leer la tabla completa
data = readtable(archivo_entrada);

% Verificar que exista la columna Timestamp
if ~ismember('timestamp_ticks', data.Properties.VariableNames)
    error('El archivo CSV no tiene una columna llamada "Timestamp".');
end

u = data.control_effort;

% ==========================================
% 2. PARÁMETROS DE BÚSQUEDA
% ==========================================
val_inicio = 55.8; 
val_fin    = 18.2; 
tol        = 0.1;  
cursor     = 1; % Puntero para recorrer el vector ordenadamente

% Función auxiliar para buscar índices (Inicio -> Fin)
% Devuelve [idx_start, idx_end, next_cursor]
function [start_idx, end_idx, next_cursor] = buscar_bloque(u_data, cursor_actual, v_ini, v_fin, tol)
    % Buscar inicio
    u_temp = u_data(cursor_actual:end);
    rel_start = find(abs(u_temp - v_ini) < tol, 1, 'first');
    
    if isempty(rel_start)
        % Lanzamos error con un ID específico para buenas prácticas
        error('Busqueda:InicioNoEncontrado', 'No se encontró el inicio de la secuencia (valor %.1f).', v_ini);
    end
    start_idx = cursor_actual + rel_start - 1;
    
    % Buscar fin
    u_temp = u_data(start_idx:end);
    rel_end = find(abs(u_temp - v_fin) < tol, 1, 'first');
    
    if isempty(rel_end)
        error('Busqueda:FinNoEncontrado', 'No se encontró el final de la secuencia (valor %.1f).', v_fin);
    end
    end_idx = start_idx + rel_end - 1;
    
    % Actualizar cursor para la siguiente búsqueda
    next_cursor = end_idx + 1;
end

% ==========================================
% 3. PROCESAMIENTO DE SECUENCIAS
% ==========================================

%% --- A. SECUENCIA AZUL ---
try
    [idx_ini, idx_fin, cursor] = buscar_bloque(u, cursor, val_inicio, val_fin, tol);
    
    % Extraer datos
    tiempo_s = data.timestamp_ticks(idx_ini:idx_fin) / 1000; % Convertir ticks a segundos
    lum_azul = data.b_l(idx_ini:idx_fin);
    pwm_azul = data.control_effort(idx_ini:idx_fin);
    
    % Guardar CSV
    tabla_azul = table(tiempo_s, lum_azul, pwm_azul, 'VariableNames', {'Tiempo_s', 'b_l', 'u'});
    writetable(tabla_azul, fullfile(nombre_carpeta, 'secuencia_azul.csv'));
    
    % GRAFICAR
    figure('Name', 'Secuencia AZUL', 'NumberTitle', 'off');
    
    subplot(2,1,1);
    plot(tiempo_s, lum_azul, 'b', 'LineWidth', 1.5);
    title('Luminosidad Azul vs Tiempo');
    ylabel('Luminosidad (W/m^2)'); grid on;
    
    subplot(2,1,2);
    plot(tiempo_s, pwm_azul, 'k', 'LineWidth', 1.5);
    title('Señal de Control (u) vs Tiempo');
    xlabel('Tiempo (s)'); ylabel('PWM (u)'); grid on;
    
    fprintf('Azul procesado: %d datos.\n', height(tabla_azul));
catch ME
    % CORRECCIÓN: Uso correcto del warning con identificador y formato
    warning(ME.identifier, 'Error en secuencia Azul: %s', ME.message);
end

%% --- B. SECUENCIA ROJA ---
try
    [idx_ini, idx_fin, cursor] = buscar_bloque(u, cursor, val_inicio, val_fin, tol);
    
    tiempo_s = data.timestamp_ticks(idx_ini+1:idx_fin) / 1000;
    lum_roja = data.r_l(idx_ini+1:idx_fin);
    pwm_roja = data.control_effort(idx_ini+1:idx_fin);
    
    tabla_roja = table(tiempo_s, lum_roja, pwm_roja, 'VariableNames', {'Tiempo_s', 'r_l', 'u'});
    writetable(tabla_roja, fullfile(nombre_carpeta, 'secuencia_roja.csv'));
    
    % GRAFICAR
    figure('Name', 'Secuencia ROJA', 'NumberTitle', 'off');
    
    subplot(2,1,1);
    plot(tiempo_s, lum_roja, 'r', 'LineWidth', 1.5); 
    title('Luminosidad Roja vs Tiempo');
    ylabel('Luminosidad (W/m^2)'); grid on;
    
    subplot(2,1,2);
    plot(tiempo_s, pwm_roja, 'k', 'LineWidth', 1.5);
    title('Señal de Control (u) vs Tiempo');
    xlabel('Tiempo (s)'); ylabel('PWM (u)'); grid on;
    
    fprintf('Roja procesada: %d datos.\n', height(tabla_roja));
catch ME
    % CORRECCIÓN
    warning(ME.identifier, 'Error en secuencia Roja: %s', ME.message);
end

%% --- C. SECUENCIA FAR-RED (FR) ---
try
    [idx_ini, idx_fin, cursor] = buscar_bloque(u, cursor, val_inicio, val_fin, tol);
    
    tiempo_s = data.timestamp_ticks(idx_ini+1:idx_fin) / 1000;
    lum_fr   = data.fr_l(idx_ini+1:idx_fin);
    pwm_fr   = data.control_effort(idx_ini+1:idx_fin);
    
    tabla_fr = table(tiempo_s, lum_fr, pwm_fr, 'VariableNames', {'Tiempo_s', 'fr_l', 'u'});
    writetable(tabla_fr, fullfile(nombre_carpeta, 'secuencia_fr.csv'));
    
    % GRAFICAR
    figure('Name', 'Secuencia FAR-RED', 'NumberTitle', 'off');
    
    subplot(2,1,1);
    plot(tiempo_s, lum_fr, 'm', 'LineWidth', 1.5); 
    title('Luminosidad Far-Red vs Tiempo');
    ylabel('Luminosidad (W/m^2)'); grid on;
    
    subplot(2,1,2);
    plot(tiempo_s, pwm_fr, 'k', 'LineWidth', 1.5);
    title('Señal de Control (u) vs Tiempo');
    xlabel('Tiempo (s)'); ylabel('PWM (u)'); grid on;
    
    fprintf('FR procesada:   %d datos.\n', height(tabla_fr));
catch ME
    % CORRECCIÓN
    warning(ME.identifier, 'Error en secuencia FR: %s', ME.message);
end

fprintf('--------------------------------------------------\n');
fprintf('Proceso terminado. CSVs guardados y Gráficos generados.\n');