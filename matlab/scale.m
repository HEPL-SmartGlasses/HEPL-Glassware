%% Map scaling

h = 5.93 * 2.4 / 0.15;
w = 7.88 * 2.4 / 0.15;

map = readmatrix("map.txt");
%% Build Graph Data Structure
[nodes, edges] = buildGraphFromMap(map);
graph = struct;
graph.nodes = nodes;
graph.edges = edges;

% currentPos = [39.36 62.48];
% destination = [12.96 81.84];
startIdx = 13;
destinationIdx = 8;
currentPos = [graph.nodes(startIdx,1) graph.nodes(startIdx,2)];
destination = [graph.nodes(destinationIdx,1) graph.nodes(destinationIdx,2)];
% startIdx = findNode(nodes, currentPos);
% destinationIdx = findNode(nodes, destination);
path = findShortestPath(graph, startIdx, destinationIdx);
heading = findHeading(graph, path);

%% Plot map
figure()
close all;
hold on
scatter(currentPos(1), currentPos(2), 50, "filled");
scatter(destination(1), destination(2),50, "filled")
for i = 1:size(map,1)
    x1 = map(i,1); y1 = map(i,2); x2 = map(i,3); y2 = map(i,4);
    plot([x1 x2], [y1 y2], "b", "LineWidth",2)
end
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');

for i = 1:size(graph.nodes,1)
    text(graph.nodes(i,1), graph.nodes(i,2), num2str(i));
end

% plot path
for i = 1:(size(path,2)-1)
    plot([graph.nodes(path(i),1) graph.nodes(path(i+1),1)], [graph.nodes(path(i),2) graph.nodes(path(i+1),2)], "--r","LineWidth",3);
end

xlabel("x (m)")
ylabel("y (m)")

legend("Current Position", "Destination","","","","","","","","","","","","","","","","","","",...
       "","","","","","","","","","","","","","","","","","","Shortest Path")

%% READ SVG FILE

doc = xmlread('atrium.svg');
lines = doc.getElementsByTagName('line');

for i = 0:lines.getLength()-1
    x1 = str2double(lines.item(i).getAttribute('x1'));
    y1 = str2double(lines.item(i).getAttribute('y1'));
    x2 = str2double(lines.item(i).getAttribute('x2'));
    y2 = str2double(lines.item(i).getAttribute('y2'));
    disp(['Line ', num2str(i+1), ': (', num2str(x1), ',', num2str(y1), ') - (', num2str(x2), ',', num2str(y2), ')']);
end

%%

function [d] = convert2eecs(m)
    d = m * 2.4 / 0.15;
end

function [] = points2svg(map)
% min y
miny = min([map(:,2); map(:,4)]);
% Define the file name
filename = 'map.svg';

% Open the file for writing
fid = fopen(filename, 'w');

% Write the SVG header
fprintf(fid, '<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n');
fprintf(fid, '<svg  width="1000" height="1000" xmlns="http://www.w3.org/2000/svg" version="1.1">\n');

% Generate line with map points
for i = 1:size(map,1)
    x1 = map(i,1); y1 = map(i,2); x2 = map(i,3); y2 = map(i,4);
    fprintf(fid, '<line x1 = "%f" x2 = "%f" y1 = "%f" y2 = "%f" stroke="black" stroke-width="2"/>\n',...
                    x1*4, x2*4, (y1 - miny)*4, (y2 - miny)*4);
end

% Write the SVG footer
fprintf(fid, '</svg>\n');

% Close the file
fclose(fid);

end

% add Node to graph
function [nodes] = addNode(nodes, coords)
    x = coords(1); y = coords(2);

    for i = 1:size(nodes,1)
        x1 = nodes(i,1); y1 = nodes(i,2);
        if (x == x1 && y == y1)
            return;
        end
    end
    nodes(end + 1, :) = [x y];
end

% find Node index
function [idx] = findNode(nodes, coords)
    idx = NaN;
    for i = 1:size(nodes,1)
        x1 = nodes(i,1); y1 = nodes(i,2);

        x = coords(1); y = coords(2);

        if (x == x1 && y == y1)
            idx = i;
        end
    end
end

% build Nodes from Map
function [nodes, edges] = buildGraphFromMap(map)
    nodes = [];
    for i = 1:size(map,1)
        x1 = map(i,1); y1 = map(i,2); x2 = map(i,3); y2 = map(i,4);
    
        nodes = addNode(nodes, [x1 y1]);
        nodes = addNode(nodes, [x2 y2]);
    end

    edges = [];
    for i = 1:size(map,1)
        x1 = map(i,1); y1 = map(i,2); x2 = map(i,3); y2 = map(i,4);
        
        node1Idx = findNode(nodes, [x1 y1]);
        node2Idx = findNode(nodes, [x2 y2]);

       edges(end + 1, :) = [node1Idx node2Idx];
    end
end

function [openN, q] = findLeastF(open)
    f = Inf;
    for i = 1:size(open,1)
        f_new = open(i,2);
        if f_new < f
            f = f_new;
            q = open(i,:);
        end
    end
    
    % pop from open
    openN = [];
    j = 1;
    for i = 1:size(open,1)
        if q(1) ~= open(i,1)
            openN(j,:) = open(i,:);
            j = j + 1;
        end
    end
end

function [succ] = getSucc(q, edges)
    succ = [];
    for i = 1:size(edges,1)
        idx1 = edges(i,1); idx2 = edges(i,2);

        if q(1) == idx1 
            succ(end+1, :) = [idx2, q(1), NaN, NaN, NaN];
        elseif q(1) == idx2
            succ(end+1, :) = [idx1, q(1), NaN, NaN, NaN];
        end
    end
end

function [d] = distance(idx1, idx2, nodes)
     coords1 = nodes(idx1); coords2 = nodes(idx2);
     d = norm([coords1; coords2]);
end

function [isSkip] = findFList(idx, f, open)
    isSkip = 0;
    for i = 1:size(open,1)
        if open(i,1) == idx && open(i,2) < f
            %f = open(i,2);
            isSkip = 1;
            return;
        end
    end
end

function [path] = backtrack(closed)
    path = [];
    
    path(1) = closed(end,1);
    parent = closed(end,3);

    for i = (size(closed,1)-1):-1:1
        nodeIdx = closed(i,1);
        if parent == nodeIdx
            path(end + 1) = nodeIdx;
            parent = closed(i,3);
        else
            continue;
        end
    end
    path = flip(path, 2);
end

function [walkingNodes] = findShortestPath(G, startIdx, destinationIdx)
    % initialize lists
    open = [];
    closed = [];
    walkingNodes = [];

    % add starting point to list
    open(1,:) = [startIdx, 0, 0];

    while ~isempty(open)

        % find smalled F and pop from open
        [open, q] = findLeastF(open);

        % find successors and set parent to q
        succ = getSucc(q, G.edges);

        % loop through successors
        for i = 1:size(succ,1)
            succIdx = succ(i,1);

            if succIdx == destinationIdx
%                 walkingNodes(end + 1) = q(1);
%                 walkingNodes(end + 1) = succIdx;

                closed(end+1,:) = q;
                closed(end+1,:) = [succIdx q(2) q(1)];

                walkingNodes = backtrack(closed);
                return;
            end

            % compute g and h for successor
            g = q(2) + distance(q(1), succIdx, G.nodes);
            h = distance(destinationIdx, succIdx, G.nodes);

            f = g + h;
            % save heuristics
            succ(i,3) = g;
            succ(i,4) = h;
            succ(i,5) = f;

%           iii) if a node with the same position as 
%           successor is in the OPEN list which has a 
%           lower f than successor, skip this successor
            isSkip = findFList(succIdx, f, open);
            if isSkip == 1
                continue;
            end

%         iV) if a node with the same position as 
%             successor  is in the CLOSED list which has
%             a lower f than successor, skip this successor
%             otherwise, add  the node to the open list
            isSkipClosed = findFList(succIdx, f, closed);
            if isSkipClosed == 1
                continue;
            end

            open(end+1,:) = [succIdx, f, q(1)];
        
        end

        % push q to closed list
        closed(end+1,:) = q;
%         walkingNodes(end+1) = q(1);
    end
    walkingNodes = backtrack(closed);
end

function [head] = findHeading(graph, path)
    head = [];
    for i = 1:(size(path,2)-1)
        cur = path(i);
        next = path(i + 1);

        coords1 = graph.nodes(cur,:);
        coords2 = graph.nodes(next,:);
        
        v1 = [1 0 0];
        v2 = [(coords2(1) - coords1(1)) (coords2(2) - coords1(2)) 0];
        theta = acos(dot(v1, v2) / (norm(v2)));
        if cross(v1, v2) > 0
            head(end + 1) = theta;
        else
            head(end + 1) = 2*pi - theta;
        end
    end

end

