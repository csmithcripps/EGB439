function [x,y] = getBeaconPos(height, xPx, pose)
    
        % Constants
        focalLen = 3.04 * 10^-3;  % m
        beaconHeight = 0.157;   % m
        pixSize = 11.5 * 10^-6; % m
        imWidth = 320;         % px
        xFov = deg2rad(62.2);   % rad

        % Find Range
        distance = beaconHeight / (height * pixSize) * focalLen;

        % Find Bearing
        angle = -(xFov/2) * (xPx - (imWidth/2)) / (imgWidth/2);
        
        x = pose(1) + distance * cos(pose(3) + angle);
        y = pose(2) + distance * sin(pose(3) + angle);
        
end