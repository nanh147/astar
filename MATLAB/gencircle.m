function circlepts = gencircle(radius, x0, y0)
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

end
