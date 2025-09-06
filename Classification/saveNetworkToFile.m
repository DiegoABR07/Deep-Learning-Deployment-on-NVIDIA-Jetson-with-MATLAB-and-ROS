function saveNetworkToFile(net, filename)
% saveNetworkToFile  Guarda cualquier red de clasificación en un archivo MAT.
%
%   saveNetworkToFile(net, filename)
%   ej: net = resnet18; saveNetworkToFile(net, 'resnet18.mat');
%
%   net      : Objeto SeriesNetwork, DAGNetwork o dlnetwork entrenado.
%   filename : Nombre o ruta (string o char) del archivo .mat donde se guardará.
%
% DESCRIPCIÓN:
%   Esta función recibe un objeto de red ('net') y lo almacena en
%   'filename' bajo la variable 'net'. No guarda las clases por separado:
%   para recuperar los nombres de las clases, basta usar:
%       data   = load(filename);
%       net    = data.net;
%       classes = cellstr(net.Layers(end).Classes);
%
% EJEMPLO DE USO:
%   net = resnet18;                   
%   saveNetworkToFile(net, 'resnet18.mat');
%
% (c) 2025 – Proyecto genérico de clasificación
% -------------------------------------------------------------------------

    % Comprueba que llegaron dos argumentos
    if nargin < 2
        error('saveNetworkToFile requiere dos argumentos: net y filename.');
    end

    % Guarda la red con el nombre de variable 'net'
    save(filename, 'net');
end
