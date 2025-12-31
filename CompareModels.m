clear; clc; close all;
warning('off', 'fuzzy:general:warnEvalfisInputOutOfRange');

%% ========================================================================
%  1. CONFIGURATION & DATA LOADING
% =========================================================================
folder_data = 'data';
folder_out  = 'graficos_modelos';

if ~exist(folder_out, 'dir'), mkdir(folder_out); end

% Files for RED LAMP
file_csv    = fullfile(folder_data, 'secuencia_roja.csv');
file_norm   = fullfile(folder_data, 'norm_params_rojo.mat'); 
file_fis    = fullfile(folder_data, 'modelo_rojo_dinamico.fis');
file_ft     = fullfile(folder_data, 'modelo_ft_rojo.mat');

if ~isfile(file_csv) || ~isfile(file_norm) || ~isfile(file_fis) || ~isfile(file_ft)
    error('Missing files for RED lamp.');
end

% Load Data
data_table = readtable(file_csv);
u_real = data_table.u;       
y_real = data_table.r_l;     

load(file_norm); 
if ~exist('min_y', 'var'), min_y = min_bl; max_y = max_bl; end 

fis = readfis(file_fis);
load(file_ft, 'sys_tf');

ts = 0.25; 
N = length(u_real);
t_sim = (0:N-1)' * ts;

fprintf('Data loaded. Sample Time: %.2fs\n', ts);

%% ========================================================================
%  2. SIMULATION
% =========================================================================

% --- A. Transfer Function ---
[y_ft, ~] = lsim(sys_tf, u_real, t_sim);
y_ft = max(0, y_ft);

% --- B. ANFIS ---
y_anfis = zeros(N, 1);
y_anfis(:) = y_real(1); 
if ~exist('lags_u', 'var'), lags_u = 2; lags_y = 2; end

buffer_u = u_real(1) * ones(1, lags_u + 1);
buffer_y = y_real(1) * ones(1, lags_y);

for k = 1:N
    u_curr = u_real(k);
    buffer_u = [u_curr, buffer_u(1:end-1)];
    
    input_vec = [];
    for d = 1:lags_y
        val_n = max(0, min(1, (buffer_y(d) - min_y) / (max_y - min_y)));
        input_vec = [input_vec, val_n]; 
    end
    for d = 1:(lags_u + 1)
        val_n = max(0, min(1, (buffer_u(d) - min_u) / (max_u - min_u)));
        input_vec = [input_vec, val_n]; 
    end
    
    y_next_n = evalfis(fis, input_vec);
    y_next = y_next_n * (max_y - min_y) + min_y;
    
    y_anfis(k) = y_next;
    buffer_y = [y_next, buffer_y(1:end-1)];
end

%% ========================================================================
%  3. METRICS (MAE)
% =========================================================================
calc_r2   = @(y, y_hat) 1 - sum((y - y_hat).^2) / sum((y - mean(y)).^2);
calc_rmse = @(y, y_hat) sqrt(mean((y - y_hat).^2));
calc_mae  = @(y, y_hat) mean(abs(y - y_hat));

R2_ft = calc_r2(y_real, y_ft);   RMSE_ft = calc_rmse(y_real, y_ft);   MAE_ft = calc_mae(y_real, y_ft);
R2_an = calc_r2(y_real, y_anfis); RMSE_an = calc_rmse(y_real, y_anfis); MAE_an = calc_mae(y_real, y_anfis);

fprintf('FT:    R2=%.4f, RMSE=%.4f, MAE=%.4f\n', R2_ft, RMSE_ft, MAE_ft);
fprintf('ANFIS: R2=%.4f, RMSE=%.4f, MAE=%.4f\n', R2_an, RMSE_an, MAE_an);

%% ========================================================================
%  4. STEP IDENTIFICATION
% =========================================================================
du = [0; diff(u_real)];
idx_raw = find(abs(du) > 5);
idx_changes = [];
if ~isempty(idx_raw)
    idx_changes = idx_raw(1);
    for k = 2:length(idx_raw)
        if idx_raw(k) - idx_changes(end) > 2, idx_changes = [idx_changes; idx_raw(k)]; end
    end
end

step_errors = [];
count = 0;
for i = 1:length(idx_changes)
    idx_start = idx_changes(i);
    if i < length(idx_changes), idx_end = idx_changes(i+1) - 1; else, idx_end = N; end
    
    len_step = idx_end - idx_start + 1;
    if len_step <= 4, continue; end 
    
    y_r_loc = y_real(idx_start:idx_end);
    y_a_loc = y_anfis(idx_start:idx_end);
    y_f_loc = y_ft(idx_start:idx_end);
    
    % RMSE Local
    rmse_anfis = sqrt(mean((y_r_loc - y_a_loc).^2));
    rmse_ft    = sqrt(mean((y_r_loc - y_f_loc).^2));
    
    % Ess (Steady State Error) - Last 15%
    idx_ss = floor(len_step * 0.85); if idx_ss < 1, idx_ss=1; end
    ss_real  = mean(y_r_loc(idx_ss:end));
    ss_anfis = mean(y_a_loc(idx_ss:end));
    ss_ft    = mean(y_f_loc(idx_ss:end));
    
    count = count + 1;
    step_errors(count).idx_start = idx_start;
    step_errors(count).idx_end   = idx_end;
    step_errors(count).u_val     = u_real(idx_start);
    step_errors(count).rmse_anfis = rmse_anfis;
    step_errors(count).rmse_ft    = rmse_ft;
    step_errors(count).ess_anfis  = abs(ss_real - ss_anfis);
    step_errors(count).ess_ft     = abs(ss_real - ss_ft);
end

[~, sorted_idx] = sort([step_errors.rmse_anfis]);
best_step = step_errors(sorted_idx(1));
worst_step = step_errors(sorted_idx(end));

%% ========================================================================
%  5. PLOTTING (FORCED BLACK TEXT & WHITE BG)
% =========================================================================

% --- FIG 1: FT Performance ---
f1 = figure('Units', 'pixels', 'Position', [100 100 800 500], 'Color', 'w');
plot(t_sim, y_real, 'Color', [0.2 0.2 0.2], 'LineWidth', 1.2, 'DisplayName', 'Real Data'); hold on; 
plot(t_sim, y_ft, 'b-.', 'LineWidth', 1.5, 'DisplayName', 'Linear FT Model'); 

t_obj = title(['Linear Model (FT) Performance']);
s_obj = subtitle(sprintf('R^{2}: %.4f  |  RMSE: %.4f  |  MAE: %.4f', R2_ft, RMSE_ft, MAE_ft));
ylabel('Irradiance (W/m^2)'); xlabel('Time (s)');
lgd = legend('Location', 'best'); 
grid on; 

% FORCE STYLE
force_black_style(gca, t_obj, s_obj, lgd);
print(f1, fullfile(folder_out, 'Fig1_FT_Performance.eps'), '-depsc', '-r300');

% --- FIG 2: ANFIS Performance ---
f2 = figure('Units', 'pixels', 'Position', [150 150 800 500], 'Color', 'w');
plot(t_sim, y_real, 'Color', [0.2 0.2 0.2], 'LineWidth', 1.2, 'DisplayName', 'Real Data'); hold on;
plot(t_sim, y_anfis, 'r--', 'LineWidth', 1.5, 'DisplayName', 'ANFIS Model'); 

t_obj = title(['Non-Linear Model (ANFIS) Performance']);
s_obj = subtitle(sprintf('R^{2}: %.4f  |  RMSE: %.4f  |  MAE: %.4f', R2_an, RMSE_an, MAE_an));
ylabel('Irradiance (W/m^2)'); xlabel('Time (s)');
lgd = legend('Location', 'best');
grid on;

% FORCE STYLE
force_black_style(gca, t_obj, s_obj, lgd);
print(f2, fullfile(folder_out, 'Fig2_ANFIS_Performance.eps'), '-depsc', '-r300');

% --- FIG 3: Best Step ---
f3 = figure('Units', 'pixels', 'Position', [200 200 600 450], 'Color', 'w');
idx = best_step.idx_start:best_step.idx_end;
t_rel = t_sim(idx) - t_sim(idx(1));

plot(t_rel, y_real(idx), 'k', 'LineWidth', 2.0, 'DisplayName', 'Real Data'); hold on;
plot(t_rel, y_anfis(idx), 'r--', 'LineWidth', 2.5, 'DisplayName', 'ANFIS');
plot(t_rel, y_ft(idx), 'b:', 'LineWidth', 2.5, 'DisplayName', 'Linear FT');

t_obj = title(['Best Performance Step (To ' num2str(best_step.u_val) ' PWM)']);
str_anfis = sprintf('ANFIS: RMSE=%.4f, E_{ss}=%.4f', best_step.rmse_anfis, best_step.ess_anfis);
str_ft    = sprintf('FT:      RMSE=%.4f, E_{ss}=%.4f', best_step.rmse_ft, best_step.ess_ft);
s_obj = subtitle({str_anfis, str_ft});

ylabel('Irradiance (W/m^2)'); xlabel('Time (s)');
lgd = legend('Location', 'southeast');
grid on;

% FORCE STYLE
force_black_style(gca, t_obj, s_obj, lgd);
print(f3, fullfile(folder_out, 'Fig3_Best_Step.eps'), '-depsc', '-r300');

% --- FIG 4: Worst Step ---
f4 = figure('Units', 'pixels', 'Position', [250 250 600 450], 'Color', 'w');
idx = worst_step.idx_start:worst_step.idx_end;
t_rel = t_sim(idx) - t_sim(idx(1));

plot(t_rel, y_real(idx), 'k', 'LineWidth', 2.0, 'DisplayName', 'Real Data'); hold on;
plot(t_rel, y_anfis(idx), 'r--', 'LineWidth', 2.5, 'DisplayName', 'ANFIS');
plot(t_rel, y_ft(idx), 'b:', 'LineWidth', 2.5, 'DisplayName', 'Linear FT');

t_obj = title(['Worst Performance Step (To ' num2str(worst_step.u_val) ' PWM)']);
str_anfis = sprintf('ANFIS: RMSE=%.4f, E_{ss}=%.4f', worst_step.rmse_anfis, worst_step.ess_anfis);
str_ft    = sprintf('FT:      RMSE=%.4f, E_{ss}=%.4f', worst_step.rmse_ft, worst_step.ess_ft);
s_obj = subtitle({str_anfis, str_ft});

ylabel('Irradiance (W/m^2)'); xlabel('Time (s)');
lgd = legend('Location', 'southeast');
grid on;

% FORCE STYLE
force_black_style(gca, t_obj, s_obj, lgd);
print(f4, fullfile(folder_out, 'Fig4_Worst_Step.eps'), '-depsc', '-r300');

fprintf('Figures saved with CORRECTED COLORS (Black Text/White BG).\n');

%% ========================================================================
%  LOCAL FUNCTION TO FORCE STYLES
% =========================================================================
function force_black_style(ax, t_h, s_h, l_h)
    % 1. Force Axes Colors
    set(ax, 'FontSize', 16, 'FontName', 'Times New Roman', ...
            'LineWidth', 1.5, 'Box', 'on', ...
            'XColor', [0 0 0], 'YColor', [0 0 0], 'ZColor', [0 0 0], ...
            'Color', [1 1 1]); % White Background
        
    % 2. Force Title Color
    if ~isempty(t_h)
        set(t_h, 'Color', [0 0 0], 'FontWeight', 'bold', 'FontSize', 18);
    end
    
    % 3. Force Subtitle Color
    if ~isempty(s_h)
        set(s_h, 'Color', [0 0 0], 'FontSize', 14);
    end
    
    % 4. Force Legend Colors
    if ~isempty(l_h)
        set(l_h, 'Color', [1 1 1], ...      % Background White
                 'TextColor', [0 0 0], ...  % Text Black
                 'EdgeColor', [0 0 0], ...  % Border Black
                 'FontSize', 14);
    end
end