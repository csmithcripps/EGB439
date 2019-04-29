function [Rblobs, Yblobs, Bblobs, imgRblobs, imgYBlobs, imgBblobs] = getBlobs(img)

% Add black border to the image

% Get chromacity values
[imgR, imgG, imgB] = chromaticity(img);

% Threshold red, green, blue to get binary images
imgRblobs = imgR > 0.6 & imgG < 0.6;
imgYBlobs = imgG > 0.4 & imgR > 0.4;
imgBblobs = imgB > 0.5 & imgG < 0.31;

% Median filter and extract blobs
if (mean(mean(imgRblobs)) > 0 && mean(mean(imgRblobs)) < 1)
    imgRblobs = ordfilt2(imgRblobs, 24, ones(5,5));
    imgRblobs([1 end],:,:) = 0;
    imgRblobs(:,[1 end],:) = 0;
    Rblobs = iblobs(imgRblobs, 'area', [200 20000], 'boundary', 'touch', 0);
else
    Rblobs = [];
end

if (mean(mean(imgYBlobs)) > 0 && mean(mean(imgYBlobs)) < 1)
    imgYBlobs = ordfilt2(imgYBlobs, 24, ones(5,5));
    imgYBlobs([1 end],:,:) = 0;
    imgYBlobs(:,[1 end],:) = 0;
    Yblobs = iblobs(imgYBlobs, 'area', [200 20000], 'boundary', 'touch', 0);
else
    Yblobs = [];
end
if (mean(mean(imgBblobs)) > 0 && mean(mean(imgBblobs)) < 1)
    imgBblobs = ordfilt2(imgBblobs, 24, ones(5,5));
    imgBblobs([1 end],:,:) = 0;
    imgBblobs(:,[1 end],:) = 0;
    Bblobs = iblobs(imgBblobs, 'area', [200 20000], 'boundary', 'touch', 0);
else
    Bblobs = [];
end