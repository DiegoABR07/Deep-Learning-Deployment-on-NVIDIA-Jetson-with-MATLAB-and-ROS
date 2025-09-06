# Entrenar detectores personalizados (YOLOv4)

Esta carpeta ofrece un **pipeline completo** para preparar datasets en formato COCO/Roboflow, convertirlos a tablas MATLAB, **entrenar YOLOv4** (incluida la variante *tiny*), evaluar y probar el detector tanto en imágenes como **en vivo con webcam**.

---

## 📂 Estructura

```text
TrainCustomModels/
├── coco2table.m                 # COCO/Roboflow JSON → tabla MATLAB (bbox + clases)
├── convertDataset.m             # Orquesta la conversión por splits (train/valid/test)
├── formatForYOLO.m              # Adapter de muestras para datastores combinados
├── preprocessDataYOLOv4.m       # (Opcional) Resize/augment para flujo alternativo
├── runYOLOv4.m                  # Script de experiencia: entrena y prueba
├── trainBoxesYOLOv4.m           # Entrenamiento/validación/evaluación (núcleo)
├── webcamPredict.m              # Inferencia en vivo con webcam
└── (salidas) checkpoints/, trainedDetectorYOLOv4.mat, curvas P–R, etc.
```

---

## 🧪 Flujo recomendado

1) **Convierte** tu dataset COCO (JSON) a tablas: `convertDataset.m`  
2) **Entrena** con `runYOLOv4.m` (llama internamente a `trainBoxesYOLOv4`).  
3) **Valida/visualiza** métricas y guarda `trainedDetectorYOLOv4.mat`.  
4) **Prueba** detección en imágenes o **en vivo** con `webcamPredict.m`.

---

## 📌 Scripts y funciones

### `coco2table.m` — COCO JSON → tabla MATLAB
**Qué hace:** parsea el JSON COCO de Roboflow y construye una **tabla** con:
- `imageFilename` (`string`): ruta absoluta de cada imagen  
- `objectBoundingBoxes` (`cell{M×4 double}`): `[x y w h]`  
- `objectClass` (`cell{M×1 categorical}`)

Omite imágenes sin cajas y normaliza cada bbox a 1×4. **Salida consistente** para entrenamiento.

**Entradas**
- `jsonFile`: ruta al `_annotations.coco.json`  
- `imgFolder`: carpeta de imágenes

**Salida**
- `tbl` (`table`): rutas, bboxes y etiquetas

---

### `convertDataset.m` — Conversión por *splits*
**Qué hace:** recorre `train/valid/test`, valida el `_annotations.coco.json` y las imágenes, llama a `coco2table` y **guarda** en cada carpeta un `.mat` con `tbl` (ej.: `train_cocoTable.mat`).

**Entradas**
- Usa `pwd` como raíz; espera `datasets/<proyecto>/{train,valid,test}`  
- Nombres estándar: `_annotations.coco.json` + imágenes

**Salidas**
- En cada split: `<split>_cocoTable.mat` con la variable `tbl`

---

### `formatForYOLO.m` — *Adapter* para datastores
**Qué hace:** estandariza a `{I, boxes, labels}` para flujos con `combine(imageDatastore, boxLabelDatastore)`:
- Si la entrada ya es `{I, boxes, labels}`, la deja igual  
- Si es `{I, tbl}` (o `{I, {tbl}}`), **extrae** `objectBoundingBoxes` y `objectClass`

**Entrada**: `dataIn` = `{I, boxes, labels}` **o** `{I, tbl}` / `{I, {tbl}}`  
**Salida**: `dataOut` = `{I, boxes, labels}`

---

### `preprocessDataYOLOv4.m` — (Opcional) Resize/augment
**Qué hace:** ejemplo de *preprocess* sencillo (no usado por defecto). Hace `imresize` a un `target` (p. ej., 416×416), ajusta cajas con el *scale* correspondiente y devuelve `{I_resized, bboxes_rescaled}`. Útil para experimentar con pipeline manual.

**Entradas**: `in = {I, b}`, `target = [H W C]`  
**Salida**: `out = {I_resized, b_resized}`

---

### `trainBoxesYOLOv4.m` — Entrenar y evaluar (núcleo)
**Pasos principales:**
1. **Carga/limpieza** de tablas `train/valid/test`; elimina categorías vacías  
2. **Clip/Drop** de cajas fuera del encuadre real de cada imagen  
3. **Datastores combinados** y `formatForYOLO` → `{I, boxes, labels}`  
4. **Estimación de anchors**: 9 por defecto; para *tiny* usa 6 y los divide 3+3  
5. **Detector base** con **InputSize** `[416 416 3]` por defecto  
6. **Entrenamiento** y **checkpoints**  
7. **Evaluación**: curvas P–R, métricas (incluye *mAP@0.5*, *precision*, *recall*) y figuras

**Entradas**
- `dataDir`: raíz con `train/valid/test` (cada split con su `*_cocoTable.mat` + imágenes)  
- `checkpointsFolder`: carpeta para *snapshots*  
- `modelName`: `"csp-darknet53-coco"` o `"tiny-yolov4-coco"`

**Salidas**
- `detector`: `yolov4ObjectDetector` listo para `detect(...)`  
- Carpeta `checkpoints/` y métricas/figuras generadas

**Notas**
- *Anchors*: ordenados por área y divididos en grupos (2 si *tiny*, 3 si estándar)  
- *InputSize*: el *resize* se maneja en el *reader*

---

### `runYOLOv4.m` — Experimento de punta a punta
**Qué hace:** define carpetas y `modelName`, **llama** a `trainBoxesYOLOv4` y **predice** sobre una imagen de *test*, dibujando cajas + *scores* (ejemplo mínimo de *inference-after-train*).

**Entradas**: variables internas (`datasetFolder`, `projectName`, `modelName`)  
**Salidas**: variable `detector` en el *workspace* y figura con detecciones

---

### `webcamPredict.m` — *Live demo* con webcam
**Qué hace:** carga `trainedDetectorYOLOv4.mat` (variable `detector`), abre la webcam (Support Package), **detecta en vivo** y dibuja `label: score` en cada cuadro. Muestra FPS y permite bajar resolución para ganar rendimiento.

**Entrada**: `trainedDetectorYOLOv4.mat` en la misma carpeta  
**Salida**: ventana con *preview* y detecciones en tiempo real (FPS en título)

---

## 🧩 Piezas relacionadas (detección e inferencia)

- **`myDetector.m`**: detector **CPU**; garantiza RGB y llama a `detect(det,I,Threshold=...)`.  
  **Salidas**: `[bboxes, scores, labels]` (labels como *cellstr*).

- **`myDetectorGPU.m`**: detector **GPU/Codegen** con firma  
  `[b,s,lIdx] = myDetectorGPU(im, detectorFile, threshold)`; carga el detector una vez, asegura 3 canales y devuelve **índices** de clase (1-based).

- **`myDetectorGPU_generation_script.m`**: prepara **codegen** para Jetson (dll + *Embedded Coder*), fija `inputSize` desde el detector, define tipos y genera `myDetectorGPU.so`.

- **`testDetector.m`**: *harness* para comparar CPU/GPU y **normalizar etiquetas** a texto antes de dibujar anotaciones.

> Para despliegue en ROS (fuera de esta carpeta), los nodos C++ publican:  
> • `/network_view` (imagen RGB recortada)  
> • `/network_detections` (Float32MultiArray con filas `[x y w h score labelIdx]`)  
> Visualízalo desde MATLAB con `visualizeROSDetector.m`.

---

## ▶️ Ejemplos rápidos

### Entrenar (fin a fin) y probar una imagen
```matlab
% 1) Asegúrate de tener datasets/<proyecto>/{train,valid,test}
convertDataset        % crea train_cocoTable.mat, etc.

% 2) Lanza experimento
runYOLOv4             % entrena y prueba sobre una imagen de test
```

### Cargar detector y detectar en vivo (webcam)
```matlab
% tras el entrenamiento, asegúrate de tener trainedDetectorYOLOv4.mat
webcamPredict
```

---

## 📦 Salidas típicas

- `trainedDetectorYOLOv4.mat` (variable `detector` lista para `detect(...)`)  
- Carpeta `checkpoints/<proyecto>/` con snapshots y logs de entrenamiento  
- Figuras/curvas de evaluación (P–R, *mean IoU* de anchors, etc.)  
- `*_cocoTable.mat` en cada split (train/valid/test)

---

## 💡 Consejos prácticos

- **Calidad de anotaciones**: el *clip/drop* ya está incluido para evitar cajas fuera de imagen y obtener *anchors* más estables.  
- **Anchors y tamaño**: usa 416×416×3 como base; si cambias la resolución, re-estima anchors tras el *resize* esperado.  
- **`formatForYOLO`** simplifica las combinaciones `{I, tbl}` → `{I, boxes, labels}` en *mini-batches*.  
- **Webcam**: baja la resolución si tu cámara es 1080p/4K para ganar FPS.  
- **Tiny vs. full**: *tiny-yolov4-coco* usa 6 anchors (dos grupos de 3) y es más ligero; el modelo completo usa 9 anchors (tres grupos).

---

## 🧯 Problemas comunes y soluciones (breve)

- **“No encuentra el JSON”**  
  Verifica la estructura `datasets/<proyecto>/{train,valid,test}` y el nombre **`_annotations.coco.json`** en cada split. Ejecuta `convertDataset.m` desde la raíz del proyecto y comprueba rutas absolutas en `imageFilename`.

- **“Anchors raros / resultados inestables”**  
  Asegúrate de que las cajas se estimen al **tamaño de entrada efectivo** (por defecto 416×416). El pipeline ya hace *clip/drop*; si cambias `InputSize`, **re-estima anchors**. Para *tiny*, usa 6 anchors y división 3+3.

- **“Out of memory (entrenamiento)”**  
  Reduce el **mini-batch**, usa la variante **`tiny-yolov4-coco`**, y/o disminuye la resolución de entrada coherentemente con una nueva estimación de anchors. Mantén activados los **checkpoints** para reanudar.

- **“La webcam no abre o va muy lenta”**  
  Instala el **Support Package** de webcams, baja la **resolución** en `webcamPredict.m` y ajusta el `Threshold` si hace falta. 

- **“Dibuja mal las cajas / imagen en B/N”**  
  Usa los *wrappers* (`myDetector.m` / `myDetectorGPU.m`) que garantizan **RGB** y formateo de salida `[bboxes, scores, labels]` (o índices en GPU).

---