clear; clc; close all; rng('shuffle');

% Desactivar advertencias
warning('off', 'fuzzy:general:warnEvalfisInputOutOfRange');
warning('off', 'MPC:computation:QP1');

%% --- 1. Configuración y Carga ---
carpeta_data = 'data';
carpeta_sim  = 'data_sim';
if ~exist(carpeta_sim, 'dir'), mkdir(carpeta_sim); end

% Cargar archivos (Planta y Normalización)
load(fullfile(carpeta_data, 'norm_params_fr.mat')); 
fis = readfis(fullfile(carpeta_data, 'modelo_fr_dinamico.fis'));

% Cargar MPC Optimizado
archivo_mpc = fullfile('controladores', 'MPC_FR_Optimizado.mat');
if ~isfile(archivo_mpc)
    error('No se encuentra el archivo MPC_FR_Optimizado.mat');
end
load(archivo_mpc); 

% --- CORRECCIÓN DE VARIABLE 'sysd' ---
% El optimizador guarda 'sysd', pero a veces el código espera 'sys_d'.
% Unificamos nombres:
if exist('sysd', 'var')
    sys_d = sysd;
elseif ~exist('sys_d', 'var')
    % Si no existe ninguna de las dos, intentamos reconstruirla desde la FT
    warning('No se encontró el modelo sysd en el archivo. Intentando reconstruir...');
    load(fullfile(carpeta_data, 'modelo_ft_fr.mat'), 'sys_tf');
    sys_d = c2d(ss(sys_tf), ts);
end

% Recuperar variables si están en estructura
if exist('bestVars','var')
    w_y_opt = bestVars(3); w_du_opt = bestVars(4);
    Hp_opt = round(bestVars(1)); Hc_opt = round(bestVars(2));
end

ts = 0.25;
%% --- 2. Configurar MPC ---
% Ahora usamos 'sys_d' que seguro existe
mpcobj = mpc(sys_d, ts, Hp_opt, Hc_opt);
mpcobj.Weights.OutputVariables = w_y_opt;
mpcobj.Weights.ManipulatedVariables = 0;
mpcobj.Weights.ManipulatedVariablesRate = w_du_opt;
mpcobj.ManipulatedVariables.Min = min_u;
mpcobj.ManipulatedVariables.Max = max_u;
mpcobj.OutputVariables.Min = min_bl;
mpcobj.OutputVariables.Max = max_bl;
mpcobj.Optimizer.MinOutputECR = 1e-10;

%% --- 3. Parámetros de Generación ---
num_episodios = 100;
duracion_episodio = 40; 
samples = floor(duracion_episodio / ts);

% Rangos para normalizar la integral (Estimación)
% Asumimos que el error máximo es (max - min) y dura unos 5 segundos
max_int_error = (max_bl - min_bl) * 5; 
min_int_error = -max_int_error;

INPUTS = [];
OUTPUTS = [];

fprintf('Generando datos con estructura: [u(k-1), y(k-1), ref(k), int_e] -> u(k)\n');

%% --- 4. Bucle Principal ---
for ep = 1:num_episodios
    
    % Generación de Referencia
    y_ref_vec = zeros(1, samples);
    num_steps = randi([3, 6]);
    idx_changes = round(linspace(1, samples, num_steps+1));
    
    for i = 1:num_steps
        if rand() < 0.3 
            val = min_bl + (25 - min_bl) * rand();
        else
            val = min_bl + (max_bl - min_bl) * rand();
        end
        y_ref_vec(idx_changes(i):idx_changes(i+1)) = val;
    end
    y_ref_vec = y_ref_vec(1:samples);
    
    % Estado Inicial
    xc = mpcstate(mpcobj);
    y_curr = min_bl;
    u_prev = min_u; 
    y_curr_prev = min_bl; % Inicializamos y(k-1)
    
    % Buffers para la planta NARX
    buffer_u = min_u * ones(1, lags_u + 1);
    buffer_y = min_bl * ones(1, lags_y);
    
    integral_e = 0;
    
    % --- Simulación Paso a Paso ---
    for k = 1:samples
        ref = y_ref_vec(k);
        
        % 1. Calcular Variables de Estado
        error = ref - y_curr;
        
        % Anti-windup simple
        if u_prev < max_u && u_prev > min_u
            integral_e = integral_e + (error * ts);
        else
            integral_e = integral_e * 0.99; 
        end
        
        integral_e = max(min_int_error, min(max_int_error, integral_e));
        
        % 2. MPC calcula u(k)
        u_mpc = mpcmove(mpcobj, xc, y_curr, ref);
        u_curr = max(min_u, min(max_u, u_mpc));
        
        % 3. Guardar Datos (Normalizados 0 a 1)
        if k > 1 
            % Normalización
            u_prev_n = (u_prev - min_u) / (max_u - min_u);
            y_prev_n = (y_curr_prev - min_bl) / (max_bl - min_bl); 
            ref_n    = (ref - min_bl) / (max_bl - min_bl);
            int_n    = (integral_e - min_int_error) / (max_int_error - min_int_error);
            
            % Target
            u_curr_n = (u_curr - min_u) / (max_u - min_u);
            
            % Clamping
            row_in = max(0, min(1, [u_prev_n, y_prev_n, ref_n, int_n]));
            row_out = max(0, min(1, u_curr_n));
            
            INPUTS = [INPUTS; row_in]; %#ok<AGROW>
            OUTPUTS = [OUTPUTS; row_out]; %#ok<AGROW>
        end
        
        % 4. Simular Planta
        buffer_u = [u_curr, buffer_u(1:end-1)];
        buffer_y = [y_curr, buffer_y(1:end-1)];
        
        input_vector = [];
        for d = 1:lags_y
            val_norm = max(0, min(1, (buffer_y(d) - min_bl)/(max_bl - min_bl)));
            input_vector = [input_vector, val_norm];
        end
        for d = 1:(lags_u + 1)
            val_norm = max(0, min(1, (buffer_u(d) - min_u)/(max_u - min_u)));
            input_vector = [input_vector, val_norm];
        end
        
        y_next_norm = evalfis(fis, input_vector);
        y_next = y_next_norm * (max_bl - min_bl) + min_bl;
        
        % Actualizar estados pasados
        u_prev = u_curr;
        y_curr_prev = y_curr; 
        y_curr = y_next;      
    end
    if mod(ep, 10) == 0, fprintf('Episodio %d completado.\n', ep); end
end

%% --- 5. Guardar ---
DATA_FINAL = [INPUTS, OUTPUTS];
nombre_archivo = fullfile(carpeta_sim, 'dataset_neurofuzzy_4inputs.dat');
save(nombre_archivo, 'DATA_FINAL', '-ascii');

% Guardar rangos de la integral
save(fullfile(carpeta_sim, 'rangos_integral.mat'), 'min_int_error', 'max_int_error');

fprintf('\nDataset guardado: %s\n', nombre_archivo);
fprintf('Dimensiones: %d muestras x 5 columnas.\n', size(DATA_FINAL, 1));

%% --- 6. Verificación ---
figure; 
subplot(3,1,1); plot(INPUTS(:,3)); title('Referencia Normalizada'); axis tight;
subplot(3,1,2); plot(INPUTS(:,4)); title('Integral Normalizada'); axis tight;
subplot(3,1,3); plot(OUTPUTS); title('Salida MPC (Target)'); axis tight;