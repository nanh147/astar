function circlepts = gencircle2(radius, x0, y0)
% uses midpoint circle algorithm: https://en.wikipedia.org/wiki/Midpoint_circle_algorithm
% ported C > MATLAB by Richard Arthurs

circlepts = [];
x = radius - 1;
y = 0;
dx = 1;
dy = 1;
err = dx - 2*radius;

while (x >= y)
    circlepts(end + 1, :) = [x0 + x, y0 + y];
    circlepts(end + 1, :) = [x0 + y, y0 + x];
    circlepts(end + 1, :) = [x0 - y, y0 + x];
    circlepts(end + 1, :) = [x0 - x, y0 + y];
    circlepts(end + 1, :) = [x0 - x, y0 - y];
    circlepts(end + 1, :) = [x0 - y, y0 - x];
    circlepts(end + 1, :) = [x0 + y, y0 - x];
    circlepts(end + 1, :) = [x0 + x, y0 - y];

        if (err <= 0)
            y = y+1;
            err = err + dy;
            dy = dy + 2;
        end
        if (err > 0)
            x = x - 1;
            dx = dx + 2;
            err = err + dx - 2*radius;
        end
end
% circlepts = sortrows(circlepts);
% for x = min - max
% find largest and smallest y in each group 
% append pts 
uniques = unique(circlepts(:,1));
extrapts = [];
for i = 1:size(uniques,1)
    miny = min(circlepts(circlepts(:,1) == uniques(i),2));
    maxy = max(circlepts(circlepts(:,1) == uniques(i),2));
    
    tempy = [miny:maxy]';
    tempx = repmat([uniques(i)],size(tempy));
    extrapts = vertcat(extrapts,[tempx,tempy]);
end

circlepts = vertcat(circlepts,extrapts);



end
