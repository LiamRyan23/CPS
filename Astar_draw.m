function PathTestWithAlternates
    fig = figure('Name','Amazing Pathfinding System','Position',[100,100,600,600]);
    
    hAxes = axes('Parent',fig,'Units','pixels','Position',[75,120,450,450]);
    
    setappdata(hAxes, 'obstacleStage', 0);
    
    % Load map immediately at start
    map = create_base_map();
    setappdata(hAxes, 'map', map);
    draw_map(hAxes, map);
    
    % Button to edit map
    uicontrol('Style','pushbutton','Position',[25,17,250,60], ...
        'String','Edit Map','Callback',@(s,e)edit_mode_function(hAxes, fig));

    % Find path button
    uicontrol('Style','pushbutton','Position',[325,17,250,60], ...
        'String','Find Path','Callback',@(s,e)find_path(hAxes));

end

function edit_mode_function(hAxes, fig)
    % Check editing state
    editing = getappdata(fig, 'editing');
    if isempty(editing)
        editing = false;
    end

    % Invert state
    editing = ~editing;
    setappdata(fig, 'editing', editing);

    if editing
        disp("Edit mode on");
        set(fig, 'WindowButtonDownFcn', @(s, e)mouse_edit(hAxes, fig));
    else
        disp("Edit mode off");
        set(fig, 'WindowButtonDownFcn', []);
    end
end

function mouse_edit(hAxes, fig)
    editing = getappdata(fig, 'editing');
    if ~editing
        return;
    end
    cp = get(hAxes, 'CurrentPoint');
    x = round(cp(1,1));
    y = round(cp(1,2));

    map = getappdata(hAxes, 'map');

    if x < 1 || x > size(map,2) || y < 1 || y > size(map,1)
        return;
    end

    map(y, x) = 1 - map(y, x);

    setappdata(hAxes, 'map', map);
    draw_map(hAxes, map);
end


function map = create_base_map()

    map = zeros(16, 16);
    
    % Centre blocks
    map(3:4, 3:14) = 1;
    map(6:8, 3:14) = 1;
    map(10:16, 3:14) = 1;
    
    % Outer border
    map(1, :)   = 1;
    map(end, :) = 1;
    map(:, 1)   = 1;
    map(:, end) = 1;

end

function draw_map(hAxes, map)
    cla(hAxes);
    imshow(1 - map, 'Parent', hAxes);
    colormap(hAxes, gray);
    axis(hAxes, 'equal');
    axis(hAxes, 'on');
    hold(hAxes, 'on');
    plot(hAxes, 2, 15, 'o','MarkerSize',10,'MarkerFaceColor','blue','MarkerEdgeColor','blue');
    plot(hAxes, 15,15,'o','MarkerSize',10,'MarkerFaceColor','red','MarkerEdgeColor','red');
end

function find_path(hAxes)
    % Initialises path finding and plots path
    % Called by "Find Path" button

    map = getappdata(hAxes, 'map');
    if isempty(map), return; end
    
    % Initialise start and end goal
    start = [15 2];
    goal  = [15 15];
    
    % Call A* algorithm function
    [visited, found, pathRC] = astar4(map, start, goal);
    
    % Plot path
    cla(hAxes);
    imshow(1 - map, 'Parent', hAxes);
    colormap(hAxes, gray);
    hold(hAxes, 'on');
    axis(hAxes,'equal');
    axis(hAxes,'on');
    % Plot start/end markers
    plot(hAxes, 2, 15, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
    plot(hAxes, 15,15,'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    if found
        pathXY = [pathRC(:,2) pathRC(:,1)];  % convert to [x y]
        [vr, vc] = find(visited);      % row, col of all visited nodes
        visitedXY = [vc, vr];
        
        plot(hAxes, pathXY(:,1), pathXY(:,2), 'r-', 'LineWidth', 2);
        plot(hAxes, visitedXY(:,1), visitedXY(:,2), 'g.');

        writematrix(pathXY,'path.csv');
    else
        disp("No path found!")
    end

end

function [visited, found, path] = astar4(map, startRC, goalRC)
    n = size(map, 1);
    found = false;
    path = [];
    
    % Initialise g and f costs
    g = Inf(n, n);
    f = Inf(n, n);
    % Initialise open and closed lists
    closed_list = false(n, n); 
    open_list = false(n, n);
    % Initialise parent tensor
    parent = zeros(n, n, 2, 'uint16');
    
    % Push start node onto open list and give g and f costs
    open_list(startRC(1), startRC(2)) = true;
    g(startRC(1), startRC(2)) = 0;
    f(startRC(1), startRC(2)) = manhattan_distance(startRC, goalRC);

    nbr = int16([-1 0; 1 0; 0 -1; 0 1]);

    while any(open_list(:)) % Continue if any node is in open list
        % Any nodes not in open list, set f_mask value to infinity
        f_mask = f;
        f_mask(~open_list) = Inf;
        
        % Locate lowest f_cost node
        [~, idx] = min(f_mask(:));
        [row, col] = ind2sub(size(f), idx);
        
        % Pop current node from open list
        open_list(row, col) = false;
        % Push onto closed list
        closed_list(row, col) = true;
        
        % If current node is goal, reconstruct path and stop algorithm
        if row == goalRC(1) && col == goalRC(2)
            found = true;
            path = reconstruct_path(parent, startRC, goalRC);
            break;
        end
        
        % Check Von-Neumann neighbours
        for k = 1:4
            r = row + nbr(k, 1);
            c = col + nbr(k, 2);
            
            % Bound check
            if r < 1 || r > n || c < 1 || c > n
                continue;
            end
            
            % Check for obstacle
            if map(r, c) == 1
                continue;
            end
            
            % Skip neighbour if in closed list
            if closed_list(r, c)
                continue;
            end
            
            new_g = g(row, col) + 1;
            if new_g < g(r, c)  % Update g and f cost if better alternative is found
                parent(r, c, :) = [row, col];
                g(r, c) = new_g;
                f(r, c) = new_g + manhattan_distance([r, c], goalRC);
                open_list(r, c) = true;
            end

        end
        
    end
    visited = closed_list;

end

% Manhattan distance heuristic
function h = manhattan_distance(p1, p2)
    h = abs(p2(1) - p1(1)) + abs(p2(2) - p1(2));
end

function p = reconstruct_path(parent, startRC, goalRC)
    cur = goalRC;
    p = cur;
    while any(cur ~= startRC)
        pr = parent(cur(1), cur(2), :);
        pr = double([pr(1) pr(2)]);
        if all(pr == 0)
            p = [];
            return;
        end
        p = [pr; p];
        cur = pr;
    end
end

