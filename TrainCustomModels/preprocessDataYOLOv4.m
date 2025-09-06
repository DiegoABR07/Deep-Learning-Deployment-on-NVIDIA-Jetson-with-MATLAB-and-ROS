function out = preprocessDataYOLOv4(in,target)
% No se usa en el flujo anterior; útil si quieres augmentar
I = in{1}; b = in{2};
if iscell(b), b = b{1}; end
I = im2single(imresize(I,target(1:2)));
s = target(1:2)./size(in{1},[1 2]);
b(:,1:2) = b(:,1:2).*fliplr(s);
b(:,3:4) = b(:,3:4).*fliplr(s);
out = {I,b};
end
