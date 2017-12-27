% circle plotting algorithm
% https://en.wikipedia.org/wiki/Midpoint_circle_algorithm
close all; clear all;

gridsize = 200;
radius = 10;
circlepts = [];
x = radius - 1;
y = 0;
dx = 1;
dy = 1;
err = dx - 2*radius;

x0 = 50;
y0 = 50;

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
axis([0,gridsize,0,gridsize])
hold on
grid on
plot(circlepts(:,1),circlepts(:,2),'kx')
