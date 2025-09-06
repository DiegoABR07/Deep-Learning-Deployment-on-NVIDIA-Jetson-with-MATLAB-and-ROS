function tbl = coco2table(jsonFile,imgFolder)
%COCO2TABLE  Convierte anotaciones COCO-Roboflow a una tabla MATLAB.
%  La tabla final tiene tres columnas:
%     imageFilename        string      — ruta absoluta a la imagen
%     objectBoundingBoxes  cell{M×4 double}  [x y w h] por imagen
%     objectClass          cell{M×1 categorical}
%
%  • Omite imágenes sin cajas.
%  • Asegura que cada bbox sea fila 1×4 (reshape).

raw  = fileread(jsonFile);
data = jsondecode(raw);

% ---------- Mapas de categoría e imagen ---------------------------------
catMap = containers.Map('KeyType','double','ValueType','char');
for i = 1:numel(data.categories)
    c = data.categories(i);
    catMap(c.id) = c.name;
end

imgMap = containers.Map('KeyType','double','ValueType','char');
for i = 1:numel(data.images)
    im = data.images(i);
    imgMap(im.id) = fullfile(imgFolder,im.file_name);
end

% ---------- Acumuladores de cajas y etiquetas ---------------------------
bMap = containers.Map('KeyType','double','ValueType','any');
lMap = containers.Map('KeyType','double','ValueType','any');

for i = 1:numel(data.annotations)
    a  = data.annotations(i);
    if a.category_id == 0, continue, end    % ignore super-category
    id = a.image_id;

    box = reshape(double(a.bbox),1,4);      % 1×4
    cls = catMap(a.category_id);

    if ~isKey(bMap,id)
        bMap(id) = [];  lMap(id) = {};
    end
    bMap(id) = [bMap(id); box];
    lMap(id) = [lMap(id); {cls}];
end

% ---------- Construir tabla ---------------------------------------------
ids  = imgMap.keys;
n    = numel(ids);
imgF = strings(n,1); bboxC = cell(n,1); labelC = cell(n,1);

for k = 1:n
    key = ids{k};
    if ~isKey(bMap,key), continue, end      % sin cajas ⇒ salto
    imgF(k)  = string(imgMap(key));
    bboxC{k} = bMap(key);
    labelC{k}= categorical(lMap(key));
end

has = cellfun(@(b) ~isempty(b), bboxC);
tbl = table(imgF(has),bboxC(has),labelC(has), ...
    'VariableNames',{'imageFilename','objectBoundingBoxes','objectClass'});
end
