%%Test
cam = pb.getImage();
idisp(cam)

idList = idBeacon(cam);
for i = 1:size(idList,1)
    [r,b] = getBeaconRangeBearing(idList(i,2),idList(i,3))
end