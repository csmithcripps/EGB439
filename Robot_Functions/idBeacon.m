function IDList = idBeacon(cam)

    %Initial Varaibla values
    char ID;
    [Rblobs, Yblobs, Bblobs, imgRblobs, imgYblobs, imgBblobs] = getBlobs(cam);

    %Sorts the blobs into uc values
    for i = 1:length(Rblobs)
        for j = 1:length(Rblobs)
            if Rblobs(i).uc > Rblobs(j).uc
                temp = Rblobs(i);
                Rblobs(i) = Rblobs(j);
                Rblobs(j) = temp;

            end

            if Yblobs(i).uc > Yblobs(j).uc
                temp = Yblobs(i);
                Yblobs(i) = Yblobs(j);
                Yblobs(j) = temp;
                disp('Hey')
            end

            if Bblobs(i).uc > Bblobs(j).uc
                temp = Bblobs(i);
                Bblobs(i) = Bblobs(j);
                Bblobs(j) = temp;
            end
        end
    end
    
    IDList = [];
    for i = 1:length(Rblobs)
        %Returns the beacon ID
        if Rblobs(i).vc > Yblobs(i).vc && Rblobs(i).vc > Bblobs(i).vc && Yblobs(i).vc > Bblobs(i).vc
            ID = '011110';
            ID = bin2dec(ID);
            disp(ID)
            IDList = [IDList;ID];
        elseif Rblobs(i).vc > Yblobs(i).vc && Rblobs(i).vc > Bblobs(i).vc && Bblobs(i).vc > Yblobs(i).vc
            ID = '011011';
            ID = bin2dec(ID);
            disp(ID)
            IDList = [IDList;ID];
        elseif Bblobs(i).vc > Yblobs(i).vc && Bblobs(i).vc > Rblobs(i).vc && Yblobs(i).vc > Rblobs(i).vc
            ID = '101101';
            ID = bin2dec(ID);  
            disp(ID)
            IDList = [IDList;ID];
        elseif Bblobs(i).vc > Yblobs(i).vc && Bblobs(i).vc > Rblobs(i).vc && Rblobs(i).vc > Yblobs(i).vc
            ID = '100111';
            ID = bin2dec(ID);
            disp(ID)
            IDList = [IDList;ID];
        elseif Yblobs(i).vc > Bblobs(i).vc && Yblobs(i).vc > Rblobs(i).vc && Bblobs(i).vc > Rblobs(i).vc
            ID = '111001';
            ID = bin2dec(ID);
            disp(ID)
            IDList = [IDList;ID];
        elseif Yblobs(i).vc > Bblobs(i).vc && Yblobs(i).vc > Rblobs(i).vc && Rblobs(i).vc > Bblobs(i).vc
            ID = '110110';
            ID = bin2dec(ID);
            disp(ID)
            IDList = [IDList;ID];
        end
    end
end





