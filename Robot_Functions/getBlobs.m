function [Rblobs, Yblobs, Bblobs] = getBlobs(img)

% Add black border to the image

% Get chromacity values
[imgR, imgG, imgB] = chromaticity(img);

% Threshold red, green, blue to get binary images
rBin = imgR > 0.6 & imgG < 0.6;
yBin = imgG > 0.4 & imgR > 0.4;
bBin = imgB > 0.5 & imgG < 0.31;

% Median filter and extract blobs
try
    Rblobs = iblobs(rBin, 'area', [60 10000], 'boundary', 'touch', 0);
    Bblobs = iblobs(bBin, 'area', [60 10000], 'boundary', 'touch', 0);
    Yblobs = iblobs(yBin, 'area', [60 10000], 'boundary', 'touch', 0);
catch ME
    disp(".")
    Rblobs = [];
    Bblobs = [];
    Yblobs = [];
    return
end