close all; clear all;
%% Settings
% Based on: https://www.mathworks.com/matlabcentral/fileexchange/26248-a---a-star--search-for-path-planning-tutorial
% A star with randomly placed obstacles

gridsize = 20;
obstaclepercentage = 0.2;
startx = 1;
starty = 1;
x_target = gridsize;
y_target = gridsize;
MAX_X = gridsize;
MAX_Y = gridsize;

tic()
% 0 = target, 1 = robot, 2 = free space, -1 = obstacle
obstacles = rand(gridsize) > (1-obstaclepercentage); % generate the random distrbution of objects
MAP = zeros(gridsize);
MAP(obstacles) = -1; % obstacles
MAP(~obstacles) = 2; % free space

MAP(y_target,x_target) = 0; % hard code the target
MAP(starty, startx) = 1; % robot init

%% Plot
hold on
axis([0,gridsize,0,gridsize])
grid on
plot(gridsize,gridsize,'gd') % plot the target
[y,x] = find(MAP == -1); % locations of obstacles
plot(x,y,'ko') % plot the obstacles

% [y_robot,x_robot] = find(MAP == 1); % locations of rovot
plot(startx,starty,'rd') % plot the init

% h = distance to next from start
% g = distance next node-target
% f = g + h

% put all obstacles on the closed list
% CLOSED: y, x
[CLOSED(:,1),CLOSED(:,2)] = find(MAP == -1);
closed_count = size(CLOSED,1);

% Note: functions take x,y order. But data structures have it in y,x

% put the starting location as the first node
x_node = startx;
y_node = starty;
open_count = 1;
path_cost = 0;
goal_dist = radistance(x_node, y_node, x_target, y_target);
OPEN(open_count, :) = insertOpen(x_node, y_node, x_node, y_node, path_cost, goal_dist, goal_dist);
OPEN(open_count, 1) = 0; % first one is not on the list
closed_count = closed_count + 1;
CLOSED = add2closed(CLOSED, x_node, y_node);
no_path = 1;

%% algorithm
while((x_node ~= x_target || y_node ~= y_target) && no_path == 1)
    exp_array = expandArray(x_node,y_node,path_cost,x_target,y_target,CLOSED,MAX_X,MAX_Y);
    exp_count=size(exp_array,1);
    
    % update open list with successors
    for i = 1:exp_count
        flag = 0;
        for j = 1:open_count
            if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3))
                OPEN(j,8) = min(OPEN(j,8), exp_array(i,5)); %make node on open the one with the minimum cost function from expanded. This is a bad way to do it. 
                if OPEN(j,8) == exp_array(i,5) % once we find the one with min cost from expanded, update it
                    %update parents
                    OPEN(j,4) = y_node;
                    OPEN(j,5) = x_node;
                    OPEN(j,6) = exp_array(i,3);
                    OPEN(j,7) = exp_array(i,4);
                end
                flag = 1;
            end
        end % j for
        if flag == 0
            open_count = open_count + 1;
            OPEN(open_count, :) = insertOpen(exp_array(i,2),exp_array(i,1),x_node, y_node,exp_array(i,3), exp_array(i,5), exp_array(i,5)); 
        end
    end % i for
    % temp array of all items that are on open list (1 in 1st col)
    % if the goal is on the open list, that becomes j_min
    % return the index of smallest col 8 in open array
    % return -1 if temp is empty
    
    min_index = minFn(OPEN);
    if (min_index ~= -1)
        % new node = one with minimum cost function
        x_node = OPEN(min_index, 3);
        y_node = OPEN(min_index, 2);
        path_cost = OPEN(min_index, 6);
        % move the node to the closed list
        closed_count = closed_count +1;
        CLOSED(closed_count,:) = [y_node, x_node];
        OPEN(min_index,1) = 0; % mark it off the open list
    else
        % No path exists
        NoPath = 0; % exit the loop
    end
end

% optimal path starts with the final node
finali = 1;
% possible xy issue. Previously 2, 1
% closed: y, x
optimal(finali,:) = [CLOSED(end,1),CLOSED(end,2)]; % optimal path starts with end of closed list
xval = CLOSED(end,2);
yval = CLOSED(end,1);
finali = finali + 1;

if optimal(1,1) == y_target && optimal(1,2) == x_target
    inode = 0;
    %traverse open to find parent nodes
    correct_index = nodeIndex(OPEN, xval, yval);
    parent_x = OPEN(correct_index,5); 
    parent_y = OPEN(correct_index,4);
    
    % Richard: and vs OR
    while( parent_x ~= startx && parent_y ~= starty)
        optimal(finali,2) = parent_x;
        optimal(finali,1) = parent_y;
        %Get the grandparents:-)
        inode = nodeIndex(OPEN,parent_x,parent_y);
        parent_x = OPEN(inode,5);%node_index returns the index of the node
        parent_y = OPEN(inode,4);
        finali=finali+1;
    end
        optimal(end,:) = [starty, startx];
   plot(optimal(:,2),optimal(:,1),'-bx')
else
   disp('No Path is possible') 
end

toc()

function index = nodeIndex(OPEN, xval, yval) % finds the index of the entry in OPEN with the required y and x values
gh = 1;
    while(OPEN(gh,2) ~= yval && OPEN(gh,3) ~= xval)
        gh=gh+1;
    end

    index=gh;
end

function min_index = minFn(OPEN)  % find index of node with minimum cost 
OPEN = [OPEN, [1:size(OPEN, 1)]']; % augment open with indices
temp = OPEN(OPEN(:,1) == 1, :); % temp is all nodes on open list (first col in OPEN = 1)
if isempty(temp)
    min_index = -1; % nothing on OPEN 
else
    [~, min_index] = min(temp(:,8)); % index of entry in temp with minimum cost function
    min_index = temp(min_index, 9); % return index of OPEN list (col 9) from temp (permissible nodes)
end
end

function expanded = expandArray(x_node,y_node,hn,x_target,y_target,CLOSED,MAX_X,MAX_Y) % generate expansion (search area) around a node
expanded = [];
exp_count = 1;
c2 = size(CLOSED,1);%Number of elements in CLOSED including the zeros

for k = 1:-1:-1 % expand outward
    for j = 1:-1:-1
        if (k ~= j || k ~= 0)
            tempx = x_node + k;
            tempy = y_node + j;
            
            % if tempx and tempy are valid and not on closed list
            if((tempx > 0 && tempx <= MAX_X)) && ((tempy > 0 && tempy <= MAX_Y))
                flag = 1;
                for c1 = 1:c2
                    if(tempx == CLOSED(c1,2) && tempy == CLOSED(c1, 1))
                        flag = 0;
                    end
                end % check if in closed list for
                if (flag == 1)
                    expanded(exp_count,1) = tempy;
                    expanded(exp_count,2) = tempx;
                    expanded(exp_count,3) = hn + radistance(x_node, y_node, tempx, tempy); % dist from start-node
                    expanded(exp_count,4) = radistance(x_target, y_target, tempx, tempy); % dist node-target
                    expanded(exp_count,5) = expanded(exp_count,4) + expanded(exp_count,3); % dist start-target through node
                    exp_count = exp_count + 1;
                end
            end % if valid expansion
        end
    end %
end % k
end

function closed = add2closed(closed, x,y)
count = size(closed, 1);
closed(count, 1) = y;
closed(count, 2) = x;
end

function dist = radistance(xcurr, ycurr, xdest, ydest) % heuristic function
dist = sqrt((xdest-xcurr)^2 + (ydest-ycurr)^2);
end

function new_row = insertOpen(x,y,parentx,parenty, hn, gn, fn) 
new_row = [1,y,x,parenty,parentx,hn,gn,fn];
end

