function [distance, angle] = getBeaconRangeBearing(height, xPx)
    
        % Constants
        focalLen = 3.04 * 10^-3;  % m
        beaconHeight = 0.057;   % m
        pixSize = 11.5 * 10^-6; % m
        imWidth = 320;         % px
        xFov = deg2rad(62.2);   % rad

        % Find Range
        distance = (beaconHeight / (height * pixSize)) * focalLen * 1.02;

        % Find Bearing
        angle = deg2rad(-(xFov/2) * (xPx - (imWidth/2)) / (imWidth/2));
        
end