%function ID = idBeacon(cam)

%Initial Varaibla values
addpath('Beacon_images/Beacon_images')
load('Arena_img.mat')
load('imgall3_3.mat')
pb = PiBot('172.19.232.178','172.19.232.12',6);
Beacon = [];
char ID;

img = imread('GetImage.png');
%cam = pb.getImage;

[r,g,b] = chromaticity(img);

[Rblobs, Yblobs, Bblobs, imgRblobs, imgYblobs, imgBblobs] = getBlobs(img);

%Sorts the blobs into uc values
for i = 1:length(Rblobs)
    for j = 1:length(Rblobs)
        if Rblobs(i).uc < Rblobs(j).uc
            temp = Rblobs(i);
            Rblobs(i) = Rblobs(j);
            Rblobs(j) = temp;
            
        end
        
        if Yblobs(i).uc < Yblobs(j).uc
            temp = Yblobs(i);
            Yblobs(i) = Yblobs(j);
            Yblobs(j) = temp;
            disp('Hey')
        end
        
        if Bblobs(i).uc < Bblobs(j).uc
            temp = Bblobs(i);
            Bblobs(i) = Bblobs(j);
            Bblobs(j) = temp;
        end
    end
end

for i = 1:length(Rblobs)
    %Returns the beacon ID
    if Rblobs(i).vc < Yblobs(i).vc && Rblobs(i).vc < Bblobs(i).vc && Yblobs(i).vc < Bblobs(i).vc
        ID = '011110';
        ID = bin2dec(ID);
        disp('1')
    elseif Rblobs(i).vc < Yblobs(i).vc && Rblobs(i).vc < Bblobs(i).vc && Bblobs(i).vc < Yblobs(i).vc
        ID = '011011';
        disp('2')
        ID = bin2dec(ID);

    elseif Bblobs(i).vc < Yblobs(i).vc && Bblobs(i).vc < Rblobs(i).vc && Yblobs(i).vc < Rblobs(i).vc
        ID = '101101';
        ID = bin2dec(ID);  
        disp('3')
    elseif Bblobs(i).vc < Yblobs(i).vc && Bblobs(i).vc < Rblobs(i).vc && Rblobs(i).vc < Yblobs(i).vc
        ID = '100111';
        ID = bin2dec(ID);
        disp('4')
    elseif Yblobs(i).vc < Bblobs(i).vc && Yblobs(i).vc < Rblobs(i).vc && Bblobs(i).vc < Rblobs(i).vc
        ID = '111001';
        ID = bin2dec(ID);
        disp('5')
    elseif Yblobs(i).vc < Bblobs(i).vc && Yblobs(i).vc < Rblobs(i).vc && Rblobs(i).vc < Bblobs(i).vc
        ID = '110110';
        ID = bin2dec(ID);
        disp('6')
    end
end







