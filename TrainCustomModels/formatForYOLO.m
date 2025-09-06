function dataOut = formatForYOLO(dataIn)
%FORMATFORYOLO  Convierte {I, tbl} ó {I, boxes, labels} a {I, boxes, labels}

    % Caso ①: input ya viene separado  {I, boxes, labels}
    if numel(dataIn) == 3
        I      = dataIn{1};
        boxes  = dataIn{2};
        labels = dataIn{3};

    % Caso ②: input es {I, tbl} o {I, {tbl}}
    elseif numel(dataIn) == 2
        I   = dataIn{1};
        tbl = dataIn{2};

        % Si viene anidado como celda, desenvuélvelo
        if iscell(tbl)
            tbl = tbl{1};
        end

        % Validación mínima
        assert(istable(tbl), ...
            'formatForYOLO:UnexpectedInput', ...
            'Se esperaba una tabla o un cell que contenga una tabla.');

        % Extrae datos usando llaves para no heredar celdas
        boxes  = tbl.objectBoundingBoxes{1};
        labels = tbl.objectClass{1};

    else
        error('formatForYOLO:BadInputArity', ...
              'Se esperaban 2 ó 3 elementos en dataIn.');
    end

    dataOut = {I, boxes, labels};
end
