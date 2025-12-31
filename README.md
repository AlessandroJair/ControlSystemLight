# ControlSystemLight

Sistema de control avanzado para regulación de intensidad lumínica en lámparas LED (Azul, Rojo y Far Red) mediante múltiples estrategias de control.

## Descripción del Proyecto

Este proyecto implementa y compara diferentes estrategias de control para sistemas de iluminación LED utilizando MATLAB/Simulink. El sistema controla tres tipos de lámparas:
- **Lámpara Azul**: Control de LED azul
- **Lámpara Roja**: Control de LED rojo
- **Lámpara FR**: Control de LED Far Red (infrarrojo cercano)

## Controladores Implementados

### 1. PID (Proporcional-Integral-Derivativo)
- Controlador clásico sintonizado mediante algoritmos genéticos
- Archivos: `PIDSintotinizacion.m`
- Controladores: `PID_Azul_Optimizado.mat`, `PID_Rojo_Optimizado.mat`

### 2. SMC (Sliding Mode Control)
- Control por modos deslizantes para robustez ante perturbaciones
- Archivos: `SMCSintonizacion.m`
- Controladores: `SMC_Azul_Optimizado.mat`, `SMC_Rojo_Optimizado.mat`, `SMC_FR_Optimizado.mat`

### 3. BELBIC (Brain Emotional Learning Based Intelligent Controller)
- Controlador inteligente basado en aprendizaje emocional cerebral
- Archivos: `BELBICSintonizacion.m`, `BELBICvsHibrido.m`
- Controladores: `BELBIC_Azul_Optimizado.mat`, `BELBIC_Rojo_Optimizado.mat`, `BELBIC_FR_Optimizado.mat`

### 4. MPC (Model Predictive Control)
- Control predictivo basado en modelo
- Archivos: `MPCSintonizacion.m`
- Controladores: `MPC_Azul_Optimizado.mat`, `MPC_Rojo_Optimizado.mat`, `MPC_FR_Optimizado.mat`

### 5. NARMA-L2 (Neural ARMA with Integral Action)
- Control neuronal adaptativo con acción integral
- Archivos: `GenDataNARMA_L2.m`, `IntegralNARMASintonizacion.m`
- Controladores: `NARMA_Azul_Optimizado.mat`, `NARMA_Rojo_Optimizado.mat`, `NARMA_FR_Optimizado.mat`
- Redes neuronales: `NARMA_*_nn.mat`

### 6. Neuro-Fuzzy (ANFIS)
- Controladores difusos adaptativos
- Archivos: `ANFISRecognition.m`, `NFComparison.m`, `NFIntegradorSintonizacion.m`
- Controladores: `NF_Azul.fis`, `NF_Rojo.fis`, `NF_FR.fis`
- Con integrador: `NF_*_integrador.fis`

### 7. Sistema Híbrido Condicional
- Combinación inteligente de múltiples controladores
- Archivos: `BELBICvsHibrido.m`
- Parámetros: `Params_Hibrido_Condicional.mat`

## Estructura del Proyecto

```
ControlSystemLight/
├── data/                           # Datos experimentales y modelos
│   ├── NEW_DATA_LED.csv           # Dataset principal
│   ├── secuencia_azul.csv         # Secuencia lámpara azul
│   ├── secuencia_roja.csv         # Secuencia lámpara roja
│   ├── secuencia_fr.csv           # Secuencia lámpara FR
│   ├── modelo_ft_*.mat            # Funciones de transferencia identificadas
│   ├── modelo_*_dinamico.fis      # Modelos dinámicos neuro-fuzzy
│   └── norm_params_*.mat          # Parámetros de normalización
│
├── data_sim/                      # Datos para simulación
│   ├── datos_entrenamiento_narma.mat
│   ├── dataset_neurofuzzy_4inputs.dat
│   └── rangos_integral.mat
│
├── controladores/                 # Controladores optimizados
│   ├── PID_*.mat                  # Controladores PID
│   ├── SMC_*.mat                  # Controladores SMC
│   ├── BELBIC_*.mat               # Controladores BELBIC
│   ├── MPC_*.mat                  # Controladores MPC
│   ├── NARMA_*.mat                # Controladores NARMA
│   ├── NF_*.fis                   # Controladores Neuro-Fuzzy
│   └── Params_Hibrido_Condicional.mat
│
├── FTRecognition.m                # Identificación función transferencia
├── ANFISRecognition.m             # Identificación modelos ANFIS
├── Data.m                         # Extracción y preprocesamiento
├── NARMA.slx                      # Modelo Simulink
└── *Sintonizacion.m               # Scripts de sintonización
```

## Flujo de Trabajo

### 1. Adquisición y Preprocesamiento de Datos
```matlab
% Ejecutar para extraer y procesar datos experimentales
Data.m
```
Se extraen datos utilizando secuencias aleatorias de escalones en las 3 lámparas.

### 2. Identificación de Modelos

#### Función de Transferencia (Modelo Lineal)
```matlab
% Identifica modelos de primer orden para cada lámpara
FTRecognition.m
```
Genera: `modelo_ft_azul.mat`, `modelo_ft_rojo.mat`, `modelo_ft_fr.mat`

#### Modelo Neuro-Fuzzy (Modelo No Lineal)
```matlab
% Identifica modelos dinámicos ANFIS
ANFISRecognition.m
```
Genera: `modelo_azul_dinamico.fis`, `modelo_rojo_dinamico.fis`, `modelo_fr_dinamico.fis`

### 3. Generación de Datos de Entrenamiento

```matlab
% Para controladores NARMA-L2
GenDataNARMA_L2.m

% Para controladores Neuro-Fuzzy
GenDataNF.m
GenDataNFwithInteger.m
```

### 4. Sintonización de Controladores

Cada controlador tiene su script de optimización mediante algoritmos genéticos:

```matlab
PIDSintotinizacion.m        % Optimiza Kp, Ki, Kd
SMCSintonizacion.m          % Optimiza parámetros SMC
BELBICSintonizacion.m       % Optimiza BELBIC
MPCSintonizacion.m          % Optimiza MPC
IntegralNARMASintonizacion.m    % Entrena NARMA-L2
NFIntegradorSintonizacion.m     % Entrena Neuro-Fuzzy
```

### 5. Comparación y Análisis

```matlab
NFComparison.m         % Compara controladores Neuro-Fuzzy
BELBICvsHibrido.m      % Compara BELBIC vs Sistema Híbrido
```

## Requisitos

### Software
- MATLAB R2020b o superior
- Toolboxes requeridos:
  - Control System Toolbox
  - System Identification Toolbox
  - Fuzzy Logic Toolbox
  - Deep Learning Toolbox
  - Optimization Toolbox
  - Model Predictive Control Toolbox
  - Simulink

### Hardware
- Sistema de control de lámparas LED (opcional, para datos reales)

## Uso

### Ejemplo Rápido: PID

```matlab
% 1. Cargar modelo y controlador
load('data/modelo_ft_azul.mat', 'sys_tf');
load('controladores/PID_Azul_Optimizado.mat', 'Kp_opt', 'Ki_opt', 'Kd_opt');

% 2. Simular controlador
ts = 0.25;  % Tiempo de muestreo
t = 0:ts:30;
ref = 50 * ones(size(t));  % Referencia constante

% 3. Implementar control PID
% (Ver scripts de sintonización para detalles)
```

### Ejecutar Simulación Completa

```matlab
% Abrir modelo Simulink
open_system('NARMA.slx');

% Cargar controlador deseado
load('controladores/NARMA_Azul_Optimizado.mat');

% Ejecutar simulación
sim('NARMA');
```

## Métricas de Desempeño

Los controladores son evaluados usando:
- **ITAE**: Integral del Tiempo multiplicado por el Error Absoluto
- **Overshoot**: Sobrepaso máximo
- **Rise Time**: Tiempo de subida
- **Settling Time**: Tiempo de establecimiento
- **Control Effort**: Esfuerzo de control (variabilidad de la señal)

## Resultados

Los controladores optimizados se encuentran en la carpeta `controladores/` con métricas de desempeño incluidas. Cada archivo `.mat` contiene:
- Parámetros optimizados del controlador
- Métricas de desempeño (ITAE, overshoot, etc.)
- Configuración de optimización utilizada

## Contribuciones

Para contribuir al proyecto:
1. Fork el repositorio
2. Crea una rama para tu feature
3. Realiza tus cambios
4. Envía un pull request

## Licencia

Este proyecto es parte de investigación académica.

## Contacto

Para preguntas o colaboraciones, por favor abrir un issue en el repositorio.