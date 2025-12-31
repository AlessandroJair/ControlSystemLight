# ControlSystemLight

<div align="center">

![MATLAB](https://img.shields.io/badge/MATLAB-R2020b+-blue.svg)
![Simulink](https://img.shields.io/badge/Simulink-Required-orange.svg)
![License](https://img.shields.io/badge/License-Academic-green.svg)
![Status](https://img.shields.io/badge/Status-Active-success.svg)

**Sistema de control avanzado para regulación de intensidad lumínica en lámparas LED**

*Control de LED Azul, Rojo y Far Red mediante múltiples estrategias de control inteligente*

</div>

---

## Tabla de Contenidos

- [Descripción del Proyecto](#descripción-del-proyecto)
- [Controladores Implementados](#controladores-implementados)
  - [PID](#1-pid-proporcional-integral-derivativo)
  - [SMC](#2-smc-sliding-mode-control)
  - [BELBIC](#3-belbic-brain-emotional-learning-based-intelligent-controller)
  - [MPC](#4-mpc-model-predictive-control)
  - [NARMA-L2](#5-narma-l2-neural-arma-with-integral-action)
  - [Neuro-Fuzzy](#6-neuro-fuzzy-anfis)
  - [Sistema Híbrido](#7-sistema-híbrido-condicional)
- [Estructura del Proyecto](#estructura-del-proyecto)
- [Flujo de Trabajo](#flujo-de-trabajo)
- [Requisitos](#requisitos)
- [Uso](#uso)
- [Métricas de Desempeño](#métricas-de-desempeño)
- [Resultados](#resultados)
- [Contribuciones](#contribuciones)
- [Licencia](#licencia)

---

## Descripción del Proyecto

Este proyecto implementa y compara diferentes estrategias de control para sistemas de iluminación LED utilizando MATLAB/Simulink. El sistema controla tres tipos de lámparas con características distintas:

| Lámpara | Tipo de LED | Aplicación |
|---------|-------------|------------|
| **Azul** | LED azul (450-495 nm) | Fotosíntesis y crecimiento vegetativo |
| **Roja** | LED rojo (620-750 nm) | Floración y fotomorfogénesis |
| **FR** | LED Far Red (infrarrojo cercano) | Respuestas fitocromo y elongación |

### Arquitectura del Sistema

```
┌─────────────────┐      ┌──────────────────┐      ┌─────────────┐
│   Referencia    │─────▶│   Controlador    │─────▶│  Lámpara    │
│   (Setpoint)    │      │   (7 tipos)      │      │  LED        │
└─────────────────┘      └──────────────────┘      └─────────────┘
                                 ▲                         │
                                 │       ┌─────────────────┘
                                 │       │
                                 │       ▼
                                 │  ┌─────────────┐
                                 └──│   Sensor    │
                                    │ (Feedback)  │
                                    └─────────────┘
```

---

## Controladores Implementados

### 1. PID (Proporcional-Integral-Derivativo)

Controlador clásico de tres términos optimizado mediante algoritmos genéticos para minimizar ITAE.

| Característica | Detalle |
|----------------|---------|
| **Tipo** | Lineal clásico |
| **Parámetros** | Kp, Ki, Kd |
| **Optimización** | Algoritmos Genéticos |
| **Archivos** | `PIDSintotinizacion.m` |
| **Modelos** | `PID_Azul_Optimizado.mat`, `PID_Rojo_Optimizado.mat` |

### 2. SMC (Sliding Mode Control)

Control por modos deslizantes que proporciona robustez ante perturbaciones y variaciones paramétricas.

| Característica | Detalle |
|----------------|---------|
| **Tipo** | No lineal robusto |
| **Ventajas** | Alta robustez, respuesta rápida |
| **Archivos** | `SMCSintonizacion.m` |
| **Modelos** | `SMC_Azul_Optimizado.mat`, `SMC_Rojo_Optimizado.mat`, `SMC_FR_Optimizado.mat` |

### 3. BELBIC (Brain Emotional Learning Based Intelligent Controller)

Controlador inteligente inspirado en el sistema límbico del cerebro, que combina aprendizaje emocional con control adaptativo.

| Característica | Detalle |
|----------------|---------|
| **Tipo** | Inteligente bio-inspirado |
| **Base** | Sistema límbico cerebral |
| **Archivos** | `BELBICSintonizacion.m`, `BELBICvsHibrido.m` |
| **Modelos** | `BELBIC_Azul_Optimizado.mat`, `BELBIC_Rojo_Optimizado.mat`, `BELBIC_FR_Optimizado.mat` |

### 4. MPC (Model Predictive Control)

Control predictivo que optimiza la trayectoria futura del sistema considerando restricciones operacionales.

| Característica | Detalle |
|----------------|---------|
| **Tipo** | Predictivo basado en modelo |
| **Ventajas** | Manejo de restricciones, optimización multivariable |
| **Archivos** | `MPCSintonizacion.m` |
| **Modelos** | `MPC_Azul_Optimizado.mat`, `MPC_Rojo_Optimizado.mat`, `MPC_FR_Optimizado.mat` |

### 5. NARMA-L2 (Neural ARMA with Integral Action)

Controlador neuronal adaptativo que aprende la dinámica inversa del sistema mediante redes neuronales.

| Característica | Detalle |
|----------------|---------|
| **Tipo** | Neuronal adaptativo |
| **Componentes** | Red neuronal + Acción integral |
| **Archivos** | `GenDataNARMA_L2.m`, `IntegralNARMASintonizacion.m` |
| **Modelos** | `NARMA_Azul_Optimizado.mat`, `NARMA_Rojo_Optimizado.mat`, `NARMA_FR_Optimizado.mat` |
| **Redes** | `NARMA_*_nn.mat` |
| **Simulink** | `NARMA.slx` |

### 6. Neuro-Fuzzy (ANFIS)

Controladores híbridos que combinan lógica difusa con aprendizaje neuronal adaptativo.

| Característica | Detalle |
|----------------|---------|
| **Tipo** | Híbrido neuro-difuso |
| **Base** | ANFIS (Adaptive Neuro-Fuzzy Inference System) |
| **Archivos** | `ANFISRecognition.m`, `NFComparison.m`, `NFIntegradorSintonizacion.m` |
| **Modelos base** | `NF_Azul.fis`, `NF_Rojo.fis`, `NF_FR.fis` |
| **Con integrador** | `NF_*_integrador.fis` |

### 7. Sistema Híbrido Condicional

Estrategia avanzada que combina múltiples controladores mediante lógica de conmutación inteligente basada en condiciones operacionales.

| Característica | Detalle |
|----------------|---------|
| **Tipo** | Híbrido multi-controlador |
| **Estrategia** | Conmutación condicional |
| **Archivos** | `BELBICvsHibrido.m` |
| **Parámetros** | `Params_Hibrido_Condicional.mat` |

---

## Estructura del Proyecto

```
ControlSystemLight/
│
├── 📁 data/                              # Datos experimentales y modelos
│   ├── NEW_DATA_LED.csv                  # Dataset principal con mediciones
│   ├── secuencia_azul.csv                # Secuencia de prueba lámpara azul
│   ├── secuencia_roja.csv                # Secuencia de prueba lámpara roja
│   ├── secuencia_fr.csv                  # Secuencia de prueba lámpara FR
│   ├── modelo_ft_*.mat                   # Funciones de transferencia identificadas
│   ├── modelo_*_dinamico.fis             # Modelos dinámicos neuro-fuzzy
│   └── norm_params_*.mat                 # Parámetros de normalización
│
├── 📁 data_sim/                          # Datos para simulación
│   ├── datos_entrenamiento_narma.mat     # Dataset entrenamiento NARMA-L2
│   ├── dataset_neurofuzzy_4inputs.dat    # Dataset Neuro-Fuzzy
│   └── rangos_integral.mat               # Rangos para acción integral
│
├── 📁 controladores/                     # Controladores optimizados
│   ├── PID_*.mat                         # Controladores PID
│   ├── SMC_*.mat                         # Controladores SMC
│   ├── BELBIC_*.mat                      # Controladores BELBIC
│   ├── MPC_*.mat                         # Controladores MPC
│   ├── NARMA_*.mat                       # Controladores NARMA-L2
│   ├── NF_*.fis                          # Controladores Neuro-Fuzzy
│   └── Params_Hibrido_Condicional.mat    # Parámetros sistema híbrido
│
├── 📄 FTRecognition.m                    # Identificación función transferencia
├── 📄 ANFISRecognition.m                 # Identificación modelos ANFIS
├── 📄 Data.m                             # Extracción y preprocesamiento
├── 📄 NARMA.slx                          # Modelo Simulink principal
│
├── 📄 PIDSintotinizacion.m               # Optimización PID
├── 📄 SMCSintonizacion.m                 # Optimización SMC
├── 📄 BELBICSintonizacion.m              # Optimización BELBIC
├── 📄 MPCSintonizacion.m                 # Optimización MPC
├── 📄 IntegralNARMASintonizacion.m       # Entrenamiento NARMA-L2
├── 📄 NFIntegradorSintonizacion.m        # Entrenamiento Neuro-Fuzzy
│
├── 📄 GenDataNARMA_L2.m                  # Generación datos NARMA
├── 📄 GenDataNF.m                        # Generación datos NF
├── 📄 GenDataNFwithInteger.m             # Generación datos NF + integral
│
├── 📄 NFComparison.m                     # Comparación controladores NF
├── 📄 BELBICvsHibrido.m                  # Comparación BELBIC vs Híbrido
│
└── 📄 README.md                          # Este archivo
```

---

## Flujo de Trabajo

### Pipeline General

```
┌─────────────────────────────────────────────────────────────────────┐
│                      1. ADQUISICIÓN DE DATOS                        │
│                         Data.m                                       │
└────────────────────────────────┬────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│                   2. IDENTIFICACIÓN DE MODELOS                      │
│                                                                     │
│  ┌─────────────────────┐           ┌──────────────────────────┐   │
│  │  FTRecognition.m    │           │  ANFISRecognition.m      │   │
│  │  (Modelo Lineal)    │           │  (Modelo No Lineal)      │   │
│  └──────────┬──────────┘           └────────────┬─────────────┘   │
└─────────────┼───────────────────────────────────┼─────────────────┘
              │                                   │
              ▼                                   ▼
     modelo_ft_*.mat                    modelo_*_dinamico.fis
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│               3. GENERACIÓN DATOS DE ENTRENAMIENTO                  │
│                                                                     │
│    GenDataNARMA_L2.m   │   GenDataNF.m   │   GenDataNFwithInt.m   │
└────────────────────────────────┬────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│                4. OPTIMIZACIÓN DE CONTROLADORES                     │
│                    (Algoritmos Genéticos)                           │
│                                                                     │
│  PID  │  SMC  │  BELBIC  │  MPC  │  NARMA-L2  │  NF  │  Híbrido   │
└────────────────────────────────┬────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│                  5. VALIDACIÓN Y COMPARACIÓN                        │
│                                                                     │
│            NFComparison.m   │   BELBICvsHibrido.m                   │
└─────────────────────────────────────────────────────────────────────┘
```

### Paso 1: Adquisición y Preprocesamiento de Datos

```matlab
% Ejecutar para extraer y procesar datos experimentales
Data.m
```

**Proceso:**
- Extracción de datos utilizando secuencias aleatorias de escalones
- Preprocesamiento y normalización
- Generación de secuencias de validación

### Paso 2: Identificación de Modelos

#### Función de Transferencia (Modelo Lineal)

```matlab
% Identifica modelos de primer orden para cada lámpara
FTRecognition.m
```

**Genera:**
- `modelo_ft_azul.mat`
- `modelo_ft_rojo.mat`
- `modelo_ft_fr.mat`

#### Modelo Neuro-Fuzzy (Modelo No Lineal)

```matlab
% Identifica modelos dinámicos ANFIS
ANFISRecognition.m
```

**Genera:**
- `modelo_azul_dinamico.fis`
- `modelo_rojo_dinamico.fis`
- `modelo_fr_dinamico.fis`

### Paso 3: Generación de Datos de Entrenamiento

```matlab
% Para controladores NARMA-L2
GenDataNARMA_L2.m

% Para controladores Neuro-Fuzzy (sin integral)
GenDataNF.m

% Para controladores Neuro-Fuzzy (con acción integral)
GenDataNFwithInteger.m
```

### Paso 4: Sintonización de Controladores

Cada controlador tiene su script de optimización mediante algoritmos genéticos:

```matlab
PIDSintotinizacion.m            % Optimiza Kp, Ki, Kd
SMCSintonizacion.m              % Optimiza parámetros SMC
BELBICSintonizacion.m           % Optimiza BELBIC
MPCSintonizacion.m              % Optimiza MPC
IntegralNARMASintonizacion.m    % Entrena NARMA-L2
NFIntegradorSintonizacion.m     % Entrena Neuro-Fuzzy con integral
```

### Paso 5: Comparación y Análisis

```matlab
NFComparison.m         % Compara controladores Neuro-Fuzzy
BELBICvsHibrido.m      % Compara BELBIC vs Sistema Híbrido
```

---

## Requisitos

### Software

| Componente | Versión Mínima | Requerido |
|------------|----------------|-----------|
| MATLAB | R2020b | ✓ |
| Simulink | - | ✓ |
| Control System Toolbox | - | ✓ |
| System Identification Toolbox | - | ✓ |
| Fuzzy Logic Toolbox | - | ✓ |
| Deep Learning Toolbox | - | ✓ |
| Optimization Toolbox | - | ✓ |
| Model Predictive Control Toolbox | - | ✓ |

### Hardware

- **Mínimo:** PC con 8GB RAM, procesador multi-core
- **Recomendado:** 16GB+ RAM, GPU compatible con MATLAB para entrenamiento neural
- **Opcional:** Sistema de control de lámparas LED (para adquisición de datos reales)

---

## Uso

### Inicio Rápido

#### 1. Ejemplo con Controlador PID

```matlab
% Cargar modelo y controlador
load('data/modelo_ft_azul.mat', 'sys_tf');
load('controladores/PID_Azul_Optimizado.mat', 'Kp_opt', 'Ki_opt', 'Kd_opt');

% Configurar simulación
ts = 0.25;                    % Tiempo de muestreo (segundos)
t = 0:ts:30;                  % Vector de tiempo
ref = 50 * ones(size(t));     % Referencia constante 50%

% Crear controlador PID
C = pid(Kp_opt, Ki_opt, Kd_opt, ts);

% Simular sistema en lazo cerrado
sys_cl = feedback(C*sys_tf, 1);
y = step(sys_cl, t);

% Visualizar resultados
figure;
plot(t, ref, 'r--', 'LineWidth', 2); hold on;
plot(t, y*100, 'b-', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Intensidad (%)');
legend('Referencia', 'Salida');
grid on;
```

#### 2. Ejemplo con Controlador NARMA-L2 en Simulink

```matlab
% Abrir modelo Simulink
open_system('NARMA.slx');

% Cargar controlador y red neuronal
load('controladores/NARMA_Azul_Optimizado.mat');

% Configurar parámetros de simulación
set_param('NARMA', 'StopTime', '30');

% Ejecutar simulación
sim('NARMA');

% Los resultados se guardan automáticamente en el workspace
```

#### 3. Ejemplo con Controlador Neuro-Fuzzy

```matlab
% Cargar sistema FIS
fis_controller = readfis('controladores/NF_Azul_integrador.fis');

% Evaluar controlador con entrada específica
% Inputs: [referencia, salida_actual, error_integral]
u = evalfis(fis_controller, [50, 45, 2.5]);

% Visualizar superficie de control
figure;
gensurf(fis_controller, [1 2], 3);  % Superficie ref-salida con integral fijo
title('Superficie de Control Neuro-Fuzzy');
xlabel('Referencia (%)');
ylabel('Salida (%)');
zlabel('Señal de Control');
```

### Flujo Completo: Desde Datos hasta Controlador

```matlab
% 1. Procesar datos experimentales
Data.m

% 2. Identificar modelos
FTRecognition.m          % Modelo lineal
ANFISRecognition.m       % Modelo no lineal

% 3. Generar datos de entrenamiento
GenDataNARMA_L2.m        % Para NARMA-L2

% 4. Optimizar controlador
IntegralNARMASintonizacion.m

% 5. Validar en Simulink
open_system('NARMA.slx');
sim('NARMA');

% 6. Analizar resultados
% Los scripts de sintonización generan automáticamente gráficas de:
% - Respuesta temporal
% - Error de seguimiento
% - Señal de control
% - Métricas de desempeño
```

---

## Métricas de Desempeño

Los controladores son evaluados usando múltiples criterios de desempeño:

| Métrica | Descripción | Fórmula | Objetivo |
|---------|-------------|---------|----------|
| **ITAE** | Integral del Tiempo × Error Absoluto | ∫ t·\|e(t)\|dt | Minimizar |
| **ISE** | Integral del Error Cuadrático | ∫ e²(t)dt | Minimizar |
| **Overshoot** | Sobrepaso máximo | (y_max - y_ss)/y_ss × 100% | < 5% |
| **Rise Time** | Tiempo de subida (10%-90%) | t₉₀ - t₁₀ | Minimizar |
| **Settling Time** | Tiempo de establecimiento (±2%) | t cuando \|e(t)\| ≤ 0.02·r | Minimizar |
| **Control Effort** | Variabilidad de la señal de control | Var(u) + λ·∑\|Δu\| | Minimizar |

### Función Objetivo de Optimización

```matlab
% Función multi-objetivo utilizada en algoritmos genéticos
J = w1*ITAE + w2*overshoot + w3*settling_time + w4*control_effort
```

Donde los pesos (w1, w2, w3, w4) se ajustan según prioridades de la aplicación.

---

## Resultados

### Ubicación de Archivos

Los controladores optimizados se encuentran en `controladores/` con la siguiente información:

```matlab
% Ejemplo de contenido en archivo .mat de controlador
load('controladores/PID_Azul_Optimizado.mat');

% Variables disponibles:
% - Parámetros optimizados: Kp_opt, Ki_opt, Kd_opt
% - Métricas: ITAE, overshoot, rise_time, settling_time
% - Configuración GA: ga_options, population_size, generations
% - Función de transferencia utilizada: sys_tf
```

### Estructura de Archivos de Resultados

| Tipo | Extensión | Contenido |
|------|-----------|-----------|
| Controladores PID/SMC/BELBIC/MPC | `.mat` | Parámetros + métricas |
| Controladores NARMA-L2 | `.mat` | Red neuronal + parámetros integral |
| Controladores Neuro-Fuzzy | `.fis` | Sistema de inferencia difusa |
| Modelos identificados | `.mat`/`.fis` | Función de transferencia o FIS |

### Comparación Cualitativa

| Controlador | Complejidad | Robustez | Tiempo Cómputo | Aplicación Ideal |
|-------------|-------------|----------|----------------|------------------|
| PID | Baja | Media | Muy bajo | Sistemas lineales simples |
| SMC | Media | Muy alta | Bajo | Sistemas con perturbaciones |
| BELBIC | Alta | Alta | Medio | Sistemas no lineales complejos |
| MPC | Alta | Alta | Alto | Restricciones y optimización |
| NARMA-L2 | Muy alta | Media | Muy alto | Sistemas altamente no lineales |
| Neuro-Fuzzy | Alta | Media-Alta | Medio-Alto | Conocimiento experto + aprendizaje |
| Híbrido | Muy alta | Muy alta | Variable | Máximo desempeño multimodo |

---

## Contribuciones

### Cómo Contribuir

1. **Fork** el repositorio
2. Crea una **rama** para tu feature:
   ```bash
   git checkout -b feature/nueva-funcionalidad
   ```
3. **Commit** tus cambios:
   ```bash
   git commit -m "Agregar nueva funcionalidad X"
   ```
4. **Push** a la rama:
   ```bash
   git push origin feature/nueva-funcionalidad
   ```
5. Abre un **Pull Request**

### Áreas de Mejora

- Implementación de controladores adicionales (H-infinity, LQR, etc.)
- Validación experimental con hardware real
- Interfaz gráfica (GUI) para configuración y monitoreo
- Análisis de robustez mediante simulaciones Monte Carlo
- Implementación en tiempo real (Raspberry Pi, Arduino, etc.)

---

## Licencia

Este proyecto es parte de investigación académica.

**Uso permitido para:**
- Investigación académica
- Educación
- Proyectos no comerciales

Para uso comercial o industrial, contactar a los autores.

---

## Contacto

Para preguntas, colaboraciones o reportar problemas:

- Abrir un **[Issue](../../issues)** en el repositorio
- Sugerencias de mejora mediante **Pull Requests**

---

<div align="center">

**Desarrollado con MATLAB/Simulink**

*Control Inteligente de Sistemas de Iluminación LED*

</div>
