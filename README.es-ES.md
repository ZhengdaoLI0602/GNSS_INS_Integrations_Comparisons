

# Descripción
Basado en el artículo titulado [Intelligent Environment-Adaptive GNSS/INS Integrated Positioning with Factor Graph Optimization](https://www.mdpi.com/2072-4292/16/1/181) en la revista [Remote Sensing](https://www.mdpi.com/journal/remotesensing), este repositorio tiene como objetivo comparar soluciones de posicionamiento 2D entre diferentes métodos de integración GNSS/INS, incluyendo Filtro de Kalman de ganancia fija (KF), Filtro de Kalman Adaptativo (AKF), Optimización por Gráficos de Factores (FGO) y Optimización Adaptativa por Gráficos de Factores (AFGO). Los scripts relacionados con FGO están modificados a partir del ejemplo oficial de MATLAB [Factor Graph-Based Pedestrian Localization with IMU and GPS Sensors](https://uk.mathworks.com/help/nav/ug/factor-graph-based-pedestrian-localization-imu-gps.html). Si encuentra algún problema, por favor envíe un correo electrónico a: zhengdao.li@connect.polyu.hk , apreciaremos sus comentarios.

# Cómo ejecutar el programa
## PASO 1: Navegue a la carpeta "DataPrepare", ejecute "Make_Format.m" dentro de la carpeta "DataPrepare"
Antes de ejecutar todos los scripts, agregue la carpeta "Functions" como subcarpetas en MATLAB.
"EnvFolder" es la carpeta que almacena todos los conjuntos de datos preparados y los resultados procesados. Está diseñada para contener:
- Una carpeta vacía llamada "IniResults" que almacena los resultados intermedios y finales
- MAT_FILE_OF_EXTRACTED_GNSS_FEATURES.mat
- MAT_FILE_OF_CNN_PREDICTION_FOR_EACH_AVAILABLE_GNSS_EPOCH.mat
- TXT_FILE_OF_GROUND_TRUTH_INFORMATION.txt
- CSV_FILE_OF_INS_DATA.csv
- MAT_FILE_OF_GNSS_DATA.mat

Después de ejecutar los códigos, se espera que los siguientes archivos se generen en la carpeta "EnvFolder/IniResults" 
- PROCESSED_DATA_FOR_KALMAN_FILTER.mat
- PROCESSED_DATA_FOR_FACTOR_GRAPH_OPTIMIZATION.mat
- COMMOM_INFORMATION_FOR_BOTH_INTEGRATION_METHODS.mat

Y se espera que los siguientes archivos se generen en la carpeta "DataPrepare" 
- MAT_FILE_STOREING_STRING_OF_PATH_OF_EnvFolder.mat

## PASO 2: Navegue a la carpeta raíz, ejecute "Main_GNSS_INS_Integration.m"
La variable "CurFolder" en el script representa exactamente la carpeta "EnvFolder/IniResults"

Configure las variables "KF_ON" y "FGO_ON" para ejecutar los siguientes cuatro métodos de integración en secuencia:
- KF_ON = [1 0], FGO_ON = [0 0]; ejecute el script (KF)
- KF_ON = [0 1], FGO_ON = [0 0]; ejecute el script (AKF)
- KF_ON = [0 0], FGO_ON = [1 0]; ejecute el script (FGO)
- KF_ON = [0 0], FGO_ON = [0 1]; ejecute el script (AFGO)

Después de ejecutar el script, se espera que los siguientes archivos se generen en la carpeta "EnvFolder/IniResults" 
- POSITIONING_SOLUTIONS_OF_ALL_INTEGRATION_METHODS.mat

Y se espera que las siguientes figuras de MATLAB se generen en la carpeta "EnvFolder/IniResults" después de la ejecución:
- Después de ejecutar KF
  - Traj_KF.fig (trayectoria 2D de Verdad/GNSS/KF)
  - 2dE.fig (error 2D de Verdad/GNSS/KF a lo largo del marco temporal)
- Después de ejecutar AKF
  - Traj_AKF.fig (trayectoria 2D de Verdad/GNSS/KF/AKF)
  - 2dE.fig (error 2D de Verdad/GNSS/KF/AKF a lo largo del marco temporal)
- Después de ejecutar FGO
  - Traj_FGO.fig (trayectoria 2D de Verdad/GNSS/KF/AKF/FGO)
  - 2dE.fig (error 2D de Verdad/GNSS/KF/AKF/FGO a lo largo del marco temporal)
  - FGO_satelite_img.fig (trayectoria 2D de Verdad/FGO mostrada con imagen satelital)
- Después de ejecutar AFGO
  - Traj_AFGO.fig (trayectoria 2D de Verdad/GNSS/KF/AKF/FGO/AFGO)
  - 2dE.fig (error 2D de Verdad/GNSS/KF/AKF/FGO/AFGO a lo largo del marco temporal)
  - AFGO_satelite_img.fig (trayectoria 2D de Verdad/AFGO mostrada con imagen satelital)

# Ejemplo
Se ha proporcionado un ejemplo de un conjunto de datos de un receptor u-blox recopilado en [Kowloon Bay, Hong Kong](https://www.google.com/maps/place/%E4%B9%9D%E9%BE%99%E6%B9%BE/@22.3207264,114.2052806,16z/data=!4m6!3m5!1s0x34040139ce5cb28d:0xebb076fb9d3032f4!8m2!3d22.3080749!4d114.2018982!16zL20vMDIyeng3?entry=ttu) en el repositorio. Los archivos de datos para apoyar la ejecución completa del código están disponibles en la carpeta EnvFolder de ejemplo en "EnvFolder.zip" en el [release 1](https://github.com/ZhengdaoLI0602/GNSS_INS_Integrations_Comparisons/releases/tag/EnvFolder). Para su análisis adicional, encuentre los archivos de observación GNSS en bruto, efemérides y verdad terrestre en "Raw.files.zip" en el [release 2](https://github.com/ZhengdaoLI0602/GNSS_INS_Integrations_Comparisons/releases/tag/RawFiles). La recopilación fue realizada por el grupo del [Intelligent Positioning And Navigation Laboratory](https://github.com/IPNL-POLYU) utilizando estándares similares a los del [UrbanNav dataset](https://github.com/IPNL-POLYU/UrbanNavDataset). Los resultados objetivo se muestran en las siguientes gráficas y se almacenan en la carpeta "Target Plots".
- Resumen
||KF|AKF|FGO|AFGO|
|---|---|---|---|---|
|Error cuadrático medio 2D [m]|70.18|66.84|24.35|19.35|
|Desviación estándar 2D [m]|61.72|59.19|18.71|14.09|
- Trayectoria 2D
  ![ublox_traj](https://github.com/ZhengdaoLI0602/GNSS_INS_Integrations_Comparisons/assets/80500317/ba8c2d80-bd2f-40f7-ac64-141f05f35818)
- Error 2D a lo largo del marco temporal
  ![ublox_2dE](https://github.com/ZhengdaoLI0602/GNSS_INS_Integrations_Comparisons/assets/80500317/0daa70d9-9e24-4ee8-b670-b51fd9794b71)
