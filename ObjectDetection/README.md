# Detección de objetos

Esta carpeta contiene todo lo necesario para **detectar objetos** en imágenes utilizando redes YOLO preentrenadas (yoloV4, YOLOX) y desplegadas en una *NVIDIA Jetson Nano* mediante **MATLAB**, **GPU Coder** y **ROS**. El objetivo es generar una biblioteca CUDA optimizada, integrarla en un nodo ROS en C++ que procese la cámara en tiempo real y publicar las detecciones como coordenadas de bounding boxes, puntuaciones y etiquetas.

---

## 📂 Estructura del directorio

```text
ObjectDetection/
├── JetsonCode/             # Nodo ROS C++ para detección (detallado más abajo)
│   └── ucsp_detect_nn/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── launch/
│       │   └── detector_usb.launch
│       └── src/
│           ├── detector.cpp
│           └── detector.hpp
├── Models/                 # Detectores preentrenados en formato .mat
│   └── yoloV4_coco.mat
├── myDetector.m            # Detección genérica en CPU
├── myDetectorGPU.m         # Detección optimizada para GPU Coder
├── myDetectorGPU_generation_script.m
├── saveDetectorToFile.m    # Guarda detectores entrenados en .mat
├── testDetector.m          # Prueba de detección en MATLAB (CPU/GPU)
└── visualizeROSDetector.m  # Visualiza detecciones vía ROS
````

---

## 🧠 Flujos de MATLAB

### `saveDetectorToFile.m`

Guarda un detector YOLO entrenado en un archivo `.mat`:

* **Entradas:**

  * `detector`: objeto `yolov4ObjectDetector` o `yoloxObjectDetector`.
  * `filename`: nombre del archivo `.mat` de destino.
* **Salida:** crea un archivo que contiene una variable `detector` con la red y sus clases; no se almacenan anclas ni listas separadas porque el objeto ya las incluye.
* **Uso:**

  ```matlab
  det = yolov4ObjectDetector("csp-darknet53-coco", "cocoClasses", ...);
  saveDetectorToFile(det, "yoloV4_coco.mat");
  ```

### `myDetector.m`

* Función de **detección en CPU**.
* Carga el detector si el archivo `.mat` cambia, asegura que la imagen sea RGB, ejecuta `detect` con un umbral de confianza y devuelve:

  * `bboxes`: M×4 con \[x y ancho alto].
  * `scores`: M×1 con la probabilidad de cada detección.
  * `labels`: M×1 cell array con las etiquetas (clases) detectadas.
* Ejemplo:

  ```matlab
  [bboxes,scores,labels] = myDetector(im, "yoloV4_coco.mat", 0.7);
  ```

### `myDetectorGPU.m`

Versión lista para **GPU Coder** con firma compatible para despliegue en Jetson:

* **Entradas:**

  * `im`: imagen RGB o en escala de grises (`uint8`) de cualquier tamaño.
  * `detectorFile`: archivo `.mat` con el detector (debe pasarse como constante en codegen).
  * `threshold`: umbral mínimo \[0,1].
* **Salidas:**

  * `bboxes`: coordenadas de las cajas en la imagen original.
  * `scores`: puntajes de cada detección.
  * `labelsIdx`: índices (1-based) de las clases (se pueden mapear a nombres).
* Utiliza `coder.loadDeepLearningNetwork` para cargar el detector una sola vez, convierte imágenes grises a RGB y llama a `detect` sin redimensionar la entrada. Devuelve los resultados como `double` para compatibilidad con codegen.

### `myDetectorGPU_generation_script.m`

* Genera la biblioteca CUDA (`libmyDetectorGPU.so`) para Jetson.

* Pasos principales:

  1. Define `detectorFile` y carga el detector para obtener `inputSize`.
  2. Configura `cfg = coder.gpuConfig("dll","ecoder",true)` y especifica el objetivo `NVIDIA Jetson`.
  3. Define prototipos de entrada:

     * `im`: tipo `uint8` con tamaño fijo `[H W 3]` (basado en `inputSize` del detector).
     * `detectorFile`: constante de compilación.
     * `threshold`: escalar `double`.
  4. Ejecuta `codegen`:

     ```matlab
     codegen -config cfg -o myDetectorGPU ...
             myDetectorGPU -args {imType, coder.Constant(detectorFile), coder.typeof(0)} -report
     ```

* El resultado incluye la biblioteca `.so`, cabeceras C++ (`myDetectorGPU.h`/`.cpp`), archivos de pesos y un informe HTML.

### `testDetector.m`

Permite probar detección en **CPU** o **GPU** desde MATLAB:

1. Define `runMode` en `"CPU"` o `"GPU"`, y ajusta `detectorFile` e `imgPath`.
2. Limpia los persistentes (`clear myDetector`/`clear myDetectorGPU`) para forzar recarga.
3. Carga la imagen y el detector para obtener `ClassNames`.
4. Ejecuta `myDetector` o `myDetectorGPU` según el modo.
5. Normaliza las etiquetas a `cellstr` (función `normalizeLabels`).
6. Dibuja las cajas y anota cada detección con el formato `clase: score`.
7. Muestra el resultado en una figura.

### `visualizeROSDetector.m`

Visualiza en tiempo real las detecciones publicadas desde Jetson a través de ROS:

* **Suscripciones:**

  * `/network_view`: imágenes recortadas (tipo `sensor_msgs/Image`).
  * `/network_detections`: detecciones en un `std_msgs/Float32MultiArray`; cada fila representa `[x, y, w, h, score, labelIdx]`.
* Conexión ROS: utiliza `rosinit(jetsonIP)` y cierra la sesión al finalizar.
* Carga el `.mat` del detector para obtener `ClassNames`.
* **Bucle principal:**

  * Lee la última imagen (`imgSub.LatestMessage`) o reutiliza la anterior.
  * Lee el último vector de detecciones (`detectSub.LatestMessage`), lo reestructura en una matriz *N×6* y extrae `bboxes`, `scores` y `labelsIdx`.
  * Mapea los índices de clase a nombres, compone anotaciones y utiliza `insertObjectAnnotation` para dibujar cajas y etiquetas en verde.
  * Actualiza la figura con `drawnow limitrate` a una frecuencia controlada por `rateControl(loopHz)`.
* Muestra un placeholder si no hay frames iniciales y adapta el canal (RGB/gris) automáticamente.

---

## 🤖 Despliegue y nodo ROS

### Descripción general

El nodo ROS de detección está implementado en la clase `DetectorNode` (archivos `detector.cpp` y `detector.hpp`). Al inicializar el nodo:

* Lee parámetros como `input_topic` (tópico de la cámara), `sub_h`, `sub_w`, `channels` y `threshold` desde el servidor de parámetros. Si `channels` no es 3, fuerza a 3 (RGB8).
* Configura publicadores para `/network_view` y `/network_detections` y se suscribe al tópico de la cámara.
* Inicializa un mensaje de imagen (`view_msg_`) con codificación RGB8 y el tamaño de recorte (`sub_h×sub_w`).
* Llama a `myDetectorGPU_init()` para inicializar la biblioteca generada.

#### Callback de imagen

Cuando llega una imagen:

1. **Validación**: comprueba que el `encoding` sea `rgb8`, `bgr8` o `mono8`; en caso contrario, emite una advertencia.
2. **Verificación de tamaño**: si el frame original es más pequeño que el recorte deseado (`sub_h×sub_w`), descarta la imagen.
3. **Cálculo de ROI**: centra el recorte dentro de la imagen original.
4. **Copia y conversión**:

   * Si la entrada es `rgb8`, copia directamente el subframe al buffer de la vista.
   * Si es `bgr8`, intercambia los canales BGR→RGB antes de copiar.
   * Si es `mono8`, replica el canal de luminancia en los tres canales RGB.
5. **Publicación de vista**: llama a `publishView_()` para enviar la imagen recortada en el tópico `/network_view`.
6. **Inferencia**: invoca `runInference_()` para convertir la imagen de row-major a column-major y llamar a la biblioteca `myDetectorGPU`; después publica las detecciones en `/network_detections`.

#### Conversión e inferencia

* `runInference_()` reorganiza la imagen de `uint8` a formato column-major (H×W×C) esperado por MATLAB, llamando a `myDetectorGPU()` con el umbral configurado. Posteriormente, publica las detecciones y libera las estructuras `emxArray`.
* `publishDetections_()` empaqueta las cajas (`bboxes`), puntajes (`scores`) e índices de clase (`labels`) en un mensaje `std_msgs::Float32MultiArray`. Cada detección ocupa 6 valores: `[x, y, w, h, score, labelIdx]`, y el vector se dimensiona a `N×6` donde `N` es el número de objetos detectados.
* `publishView_()` copia la subimagen preprocesada (RGB8) al mensaje `view_msg_` reutilizable y la publica en `/network_view`.

#### `CMakeLists.txt` y `package.xml`

* El archivo `CMakeLists.txt` (no incluido aquí) define un proyecto `ucsp_detect_nn`, busca las dependencias ROS (`roscpp`, `std_msgs`, `sensor_msgs`, `image_transport`), configura la ruta al código generado (`CODEGEN_DIR`) y crea un ejecutable `detector` enlazado contra la biblioteca `myDetectorGPU.so` (más librerías CUDA y cuDNN).
* Se debe ajustar `CODEGEN_FCN_NAME` a `myDetectorGPU` y `CODEGEN_DIR` a la carpeta donde se generó la biblioteca con GPU Coder. Además, se copia la `.so` resultante a `devel/lib` para que ROS la encuentre.
* En `package.xml` se declaran dependencias en tiempo de compilación y de ejecución (incluyendo `usb_cam`) y se especifica el nombre y versión del paquete `ucsp_detect_nn`.

#### `detector_usb.launch`

El archivo de lanzamiento prepara dos nodos:

* **Nodo de detección** (`detector_node`):

  * Pertenece al paquete `ucsp_detect_nn` y ejecuta el binario `detector_node`.
  * Expone parámetros configurables:

    * `input_topic`: tópico de entrada de la cámara, por defecto `/usb_cam/image_raw`.
    * `sub_h` y `sub_w`: tamaño del recorte que espera el detector (ej. 416×416 para YOLOv4; ajustable según tu modelo).
    * `channels`: número de canales (3).
    * `threshold`: umbral de confianza mínimo.
* **Nodo de cámara** (`usb_cam_node`):

  * Captura vídeo desde `/dev/video0` a 640×360 px y 30 FPS, en formato `yuyv` (latencia baja).
  * Desactiva balance de blancos y enfoque automáticos para obtener imágenes consistentes.

---

## 🧪 Flujo recomendado

1. **Entrenar o cargar un detector** en MATLAB usando `yolov4ObjectDetector`, `yoloxObjectDetector` u otro.
2. **Guardar el detector** en `.mat` con `saveDetectorToFile`.
3. **Generar la biblioteca** para Jetson ejecutando `myDetectorGPU_generation_script.m`; esto produce `myDetectorGPU.so`, `myDetectorGPU.h` y los archivos de pesos.
4. **Ajustar y compilar el paquete ROS**:

   * Copiar `myDetectorGPU.so` y las cabeceras generadas a `JetsonCode/ucsp_detect_nn/codegen/dll/myDetectorGPU/`.
   * Editar `CMakeLists.txt` para que `CODEGEN_DIR` apunte a esa ruta y `CODEGEN_FCN_NAME` sea `myDetectorGPU`.
   * Ejecutar `catkin_make` en el workspace.
5. **Lanzar la detección** con `roslaunch ucsp_detect_nn detector_usb.launch`.
6. **Visualizar las detecciones** en tu PC con `visualizeROSDetector.m`, que se conecta a `/network_view` y `/network_detections`, dibuja las cajas con `insertObjectAnnotation` y actualiza la imagen en tiempo real.

---

## 🔗 Referencias

* La arquitectura del nodo ROS para detección seguirá el patrón descrito en el despliegue de AlexNet: suscripción a la cámara, recorte, conversión de formato, inferencia con la biblioteca generada y publicación de resultados.
* Consulta el README principal para detalles sobre hardware, software y configuración de Jetson.
