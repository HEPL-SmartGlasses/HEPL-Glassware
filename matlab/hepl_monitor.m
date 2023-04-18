%% HEPL DISPLAY

% Create serial port object
% ser = serialport("COM4", 9600);
% 
% % Configure serial port
% configureTerminator(ser, "LF");
% configureTerminator(ser, "CR/LF");
% 
% % Read data
% data = read(ser);
% 
% % Close serial port
% delete(ser);
%% Refine map
refineMap();

%% Live Plotting

% create fake xbee data
% xbeePath = [13 12 11 9 7 8]; % map
xbeePath = [162 156 145 128 113 121]; % refineMap

% open file for writing
fid = fopen('data.txt', 'w');

% write x and y positions in each line
for i = 1:(length(xbeePath)-1)
    x1 = nodes(xbeePath(i),1);
    y1 = nodes(xbeePath(i),2);

    x2 = nodes(xbeePath(i+1),1);
    y2 = nodes(xbeePath(i+1),2);

    X = linspace(x1, x2, 3);
    Y = linspace(y1, y2, 3);
    disp(X)
    disp(Y)
    disp("   ")
    for j = 1:size(X,2)
        fprintf(fid, '%s %s %s\n', float2hex(X(j)), float2hex(Y(j)), "00000000");
    end
end

% close file
fclose(fid);


%% Plot map
figure()
close all;
hold on
scatter(destination(1), destination(2),50, "filled",'DisplayName', 'Destination')
for i = 1:size(map,1)
    x1 = map(i,1); y1 = map(i,2); x2 = map(i,3); y2 = map(i,4);
    plot([x1 x2], [y1 y2], "b", "LineWidth",2,'DisplayName', '', 'HandleVisibility', 'off')
end
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');

for i = 1:size(graph.nodes,1)
    text(graph.nodes(i,1), graph.nodes(i,2), num2str(i));
end

xlabel("x (m)")
ylabel("y (m)")

h = scatter([], [], 40,"red","filled",'DisplayName', 'User Trace');

% legend("Destination","","","","","","","","","","","","","","","","","","",...
%        "","","","","","","","","","","","","","","","","","","User Trace");
legend('show');

% imshow('atrium.png');
% [img, ~, alphachannel] = imread('newAtrium.png');
% img = imresize(img,0.1);
% image(img, 'AlphaData', alphachannel);
% image(img);

test_pos = linspace(16.96, 68.96, 25);

pos = [];
i = 1;
while true
    x = test_pos(i);
    y = 70.48;
    pos = addPos(pos, [x ; y]);
    %scatter(pos(1,:), pos(2,:),40,"red","filled");
    set(h, 'XData', pos(1,:), 'YData', pos(2,:));
    pause(0.5);

    i = i + 1;
    if i == (size(test_pos,2) + 1)
        i = 1;
    end
end

function [pos] = addPos(pos, X)
    posSize = size(pos, 2);
       
    % if trace is exceeded update array
    if posSize >= 5
       pos(:,1) = [];
    end

     pos(:,end + 1) = X;
end

function [hexStr] = float2hex(f)
    % convert float to hex
    bytes = typecast(single(f), 'uint8');
    hexStr = dec2hex(swapbytes(bytes));
    hexStr = reshape(flip(hexStr.', 2), 1, []);
end

function [] = refineMap()
    map = readmatrix("map.txt");

    % open file for writing
    fid = fopen('refineMap.txt', 'w');

    precision = 1; %m
    for i = 1:size(map,1)
        x1 = map(i,1); y1 = map(i,2); x2 = map(i,3); y2 = map(i,4);
        
        Nx = ceil(max(abs(x1-x2), abs(y1-y2))/precision);

        X = linspace(x1, x2, Nx);
        Y = linspace(y1, y2, Nx);

        for j = 1:(size(X,2)-1)
            fprintf(fid, '%f %f %f %f\n', X(j), Y(j), X(j+1), Y(j+1));
        end
    end



    % close file
    fclose(fid);

end
