function [r,g,b] = chromaticity(img)
%chromaticity Find the RGB chromaticity of an image
%   Input:  img - An RGB image to analyse
%   Output: r = red chromatic channel (image)
%           g = green chromatic channel (image)
%           b = blue chromatic channel (image)
img = double(img);
sum = img(:,:,1)+img(:,:,2)+img(:,:,3);
r = img(:,:,1)./sum;
g = img(:,:,2)./sum;
b = img(:,:,3)./sum;
end