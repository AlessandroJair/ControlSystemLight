clear; clc; close all; rng('shuffle');

% Desactivar advertencias
warning('off', 'fuzzy:general:warnEvalfisInputOutOfRange');
warning('off', 'MPC:computation:QP1');

%% --- 1. Configuración y Carga ---
carpeta_data = 'data';
carpeta_sim  = 'data_sim';
if ~exist(carpeta_sim, 'dir'), mkdir(carpeta_sim); end

ts = 0.25; % Definir tiempo de muestreo

% Cargar Archivos
load(fullfile(carpeta_data, 'norm_params_fr.mat')); 

% Parche de seguridad por si las variables se llaman _fr en vez de _bl
if ~exist('min_bl', 'var') && exist('min_fr', 'var')
    min_bl = min_fr; max_bl = max_fr;
end

fis = readfis(fullfile(carpeta_data, 'modelo_fr_dinamico.fis'));
load(fullfile('controladores', 'MPC_FR_Optimizado.mat')); 

% Corrección de nombre variable sistema
if exist('sysd', 'var'), sys_d = sysd; end 
if ~exist('sys_d', 'var')
    load(fullfile(carpeta_data, 'modelo_ft_fr.mat'), 'sys_tf');
    sys_d = c2d(ss(sys_tf), ts);
end

% Recuperar variables del MPC
if exist('bestVars','var')
    w_y_opt = bestVars(3); w_du_opt = bestVars(4);
    Hp_opt = round(bestVars(1)); Hc_opt = round(bestVars(2));
end

%% --- 2. Configurar MPC (El Maestro) ---
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
duracion_episodio = 30; 
samples = floor(duracion_episodio / ts);

INPUTS = [];
OUTPUTS = [];

fprintf('Generando dataset FR (0%% y 100%% incl.): [u(k-1), y(k-1), ref(k)] -> u(k)\n');

%% --- 4. Bucle de Simulación ---
for ep = 1:num_episodios
    
    % --- GENERACIÓN DE REFERENCIA (ESTRATEGIA 10-10) ---
    y_ref_vec = zeros(1, samples);
    num_steps = randi([3, 6]);
    idx_changes = round(linspace(1, samples, num_steps+1));
    
    for i = 1:num_steps
        dado = rand();
        
        if dado < 0.10 
            % --- CASO 1: APAGADO TOTAL (10%) ---
            val = min_bl;
            
        elseif dado < 0.20
            % --- CASO 2: POTENCIA MÁXIMA (10%) ---
            val = max_bl;
            
        elseif dado < 0.50 
            % --- CASO 3: RANGO BAJO (30%) ---
            % Zona no lineal (aprender a encender suave)
            val = min_bl + (25 - min_bl) * rand();
            
        else
            % --- CASO 4: RANGO COMPLETO ALEATORIO (50%) ---
            val = min_bl + (max_bl - min_bl) * rand();
        end
        
        y_ref_vec(idx_changes(i):idx_changes(i+1)) = val;
    end
    y_ref_vec = y_ref_vec(1:samples);
    
    % Estados Iniciales
    xc = mpcstate(mpcobj);
    y_curr = min_bl;
    u_prev = min_u; 
    y_curr_prev = min_bl; 
    
    % Buffers Planta
    buffer_u = min_u * ones(1, lags_u + 1);
    buffer_y = min_bl * ones(1, lags_y);
    
    for k = 1:samples
        ref = y_ref_vec(k);
        
        % 1. MPC calcula acción óptima u(k)
        u_mpc = mpcmove(mpcobj, xc, y_curr, ref);
        u_curr = max(min_u, min(max_u, u_mpc));
        
        % 2. GUARDAR DATOS
        if k > 1
            % Normalización
            u_prev_n = (u_prev - min_u) / (max_u - min_u);
            y_prev_n = (y_curr_prev - min_bl) / (max_bl - min_bl);
            ref_n    = (ref - min_bl) / (max_bl - min_bl);
            u_curr_n = (u_curr - min_u) / (max_u - min_u);
            
            % Clamping [0, 1]
            row_in = max(0, min(1, [u_prev_n, y_prev_n, ref_n]));
            row_out = max(0, min(1, u_curr_n));
            
            INPUTS = [INPUTS; row_in]; %#ok<AGROW>
            OUTPUTS = [OUTPUTS; row_out]; %#ok<AGROW>
        end
        
        % 3. Simular Planta
        buffer_u = [u_curr, buffer_u(1:end-1)];
        buffer_y = [y_curr, buffer_y(1:end-1)];
        
        input_vector = [];
        for d = 1:lags_y
            val_norm = max(0, min(1, (buffer_y(d)-min_bl)/(max_bl-min_bl)));
            input_vector = [input_vector, val_norm];
        end
        for d = 1:(lags_u + 1)
            val_norm = max(0, min(1, (buffer_u(d)-min_u)/(max_u-min_u)));
            input_vector = [input_vector, val_norm];
        end
        
        y_next = evalfis(fis, input_vector) * (max_bl - min_bl) + min_bl;
        
        u_prev = u_curr;
        y_curr_prev = y_curr;
        y_curr = y_next;
    end
    if mod(ep, 20) == 0, fprintf('Episodio %d completado.\n', ep); end
end

%% --- 5. Guardar ---
DATA_FINAL = [INPUTS, OUTPUTS];
nombre_archivo = fullfile(carpeta_sim, 'dataset_nf_paralelo.dat'); 
save(nombre_archivo, 'DATA_FINAL', '-ascii');

fprintf('\nDataset guardado: %s\n', nombre_archivo);
fprintf('Total Muestras: %d\n', size(DATA_FINAL,1));

%% --- 6. Plot de Verificación ---
figure('Name', 'Dataset NF Paralelo FR (Con Extremos)', 'Color', 'w');
subplot(2,2,1); plot(INPUTS(:,1)); title('u(k-1)'); axis tight;
subplot(2,2,2); plot(INPUTS(:,2)); title('y(k-1)'); axis tight;
subplot(2,2,3); plot(INPUTS(:,3)); title('Ref(k) - Observar 0 y 1'); axis tight;
subplot(2,2,4); plot(OUTPUTS);     title('Target u(k)');    axis tight;