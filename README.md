# 🏎️ Follow the Gap (FTG) Avanzado - Modo "Senna"

Este repositorio contiene la implementación del algoritmo de navegación reactiva **Follow the Gap (FTG)** para el vehículo autónomo F1Tenth. Esta versión ha sido optimizada para circuitos de alta velocidad (como Interlagos), incorporando lógica de frenado dinámico, estabilización de visión y un sistema de cronometraje de vueltas.

## 🧠 Algoritmo Follow the Gap (FTG) Implementado

El núcleo del controlador se basa en la búsqueda de espacios libres ("gaps") en la lectura del LiDAR para calcular la trayectoria segura. El proceso sigue estos pasos:

1.  **Pre-procesamiento (Cleaning & Cropping):**
    * Se limpian los datos del LiDAR (Inf/NaN).
    * **Visión de Túnel:** Se recortan los datos laterales (`FOV_CROP`) para que el algoritmo FTG se concentre únicamente en el pasillo central y no intente girar hacia huecos falsos en las paredes laterales.

2.  **Burbuja de Seguridad (Safety Bubble):**
    * Se identifica el obstáculo más cercano y se establece un radio de seguridad alrededor de él. Esto impide que el algoritmo elija una trayectoria que roce las esquinas.

3.  **Selección del Mejor Hueco (Max Gap):**
    * El código busca la secuencia continua de rayos láser más ancha.
    * Selecciona el punto medio del hueco y ajusta el ángulo de dirección (`steering_angle`) hacia él.

4.  **Control de Velocidad "Senna" (Optimización):**
    * **Rectas:** Si el ángulo hacia el hueco es cercano a 0, acelera a **11.0 m/s**.
    * **Frenado Anticipado:** Si el LiDAR detecta que el hueco se cierra a la distancia, frena agresivamente antes de iniciar el giro.

## 📂 Archivos del Proyecto

* `senna.py`: Nodo principal del controlador FTG.
* `lap_timer.py`: Herramienta de telemetría que calcula los tiempos de vuelta y detecta récords usando odometría.
* `saopaulo.png` / `.yaml`: Archivos del mapa utilizado.


## 🗺️ Configuración del Mapa (Map Setup)

Para replicar los resultados, es necesario configurar el simulador con el mapa de Sao Paulo incluido en este repositorio.

1.  Copia los archivos `saopaulo.png` y `saopaulo.yaml` a la carpeta de mapas del simulador:
    `~/F1Tenth-Repository/src/f1tenth_gym_ros/maps/`

2.  Edita el archivo de configuración `sim.yaml` (`.../f1tenth_gym_ros/config/sim.yaml`) con los siguientes parámetros:

    ```yaml
    # Ruta del mapa
    map_path: '/home/TU_USUARIO/.../maps/saopaulo'
    map_img_ext: '.png'

    # Posición Inicial (Starting Pose) - CRÍTICO PARA NO CHOCAR AL INICIO
    ego_pose_x: 28.46
    ego_pose_y: 34.56
    ego_pose_theta: -1.37
    ```
## 🚀 Instrucciones de Ejecución

Para ver el sistema completo en funcionamiento, necesitarás **3 terminales**:

1.  **Terminal 1: Lanzar el Simulador**
    ```bash
    colcon build
    source install/setup.bash
    ros2 launch f1tenth_gym_ros gym_bridge_launch.py
    ```
2.  **Terminal 2: Ejecutar el Controlador FTG**
    Inicia el piloto automático:
    ```bash
    python3 senna.py
3.  **Terminal 3: Iniciar Telemetría (Cronómetro)**
    Ejecuta este script antes de mover el auto para registrar los tiempos:
    ```bash
    python3 lap_timer.py
    ```


    ```

## ⚙️ Parámetros de Tuning

| Parámetro | Valor | Descripción |
| :--- | :--- | :--- |
| `MAX_SPEED` | `11.0` | Velocidad máxima al seguir un hueco recto. |
| `BRAKE_DIST` | `4.0` | Distancia para anticipar curvas. |
| `FOV_CROP` | `200` | Recorte lateral para estabilizar el FTG. |
