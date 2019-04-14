load queensland_towns.mat;
[p,f,e] = graph_planner(distanceMatrix,placeCoords,placeNames,5,1)

pathCost = 0;
g = 0;
for n = p
    disp(placeNames{n})
end
for i = 2:length(p)
    g = g + distanceMatrix(p(i), p(i-1));
end
disp(['final cost' , string(g)])