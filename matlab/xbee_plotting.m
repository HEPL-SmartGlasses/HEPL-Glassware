% computer xbee addr 00 13 A2 00 41 76 E7 FD


%% Set up for serial reading
if (exist('s', 'var') == 1) 
    fclose(s);
end
s = serial('COM13', 'BaudRate', 9600, 'Terminator', 'CR', 'StopBit', 1, 'Parity', 'None');
fopen(s);

start_x = 40.0;
start_y = 70.0;
curr_x = 0;
curr_y = 0;
dest = 0;

%% Plot map

figure()
close all;
hold on
d = scatter(destination(1), destination(2),50, "filled",'DisplayName', 'Destination');
h = scatter([], [], 40,"red","filled",'DisplayName', 'User Trace');
for i = 1:size(map,1)
    x1 = map(i,1); y1 = map(i,2); x2 = map(i,3); y2 = map(i,4);
    plot([x1 x2], [y1 y2], "b", "LineWidth",2,'DisplayName', '', 'HandleVisibility', 'off')
end
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');

% for i = 1:size(graph.nodes,1)
%     text(graph.nodes(i,1), graph.nodes(i,2), num2str(i));
% end

xlabel("x (m)")
ylabel("y (m)")

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


%% Reading while loop
while(1)
    num_bytes = s.BytesAvailable;
    if(num_bytes==0)
        pause(0.5);
    else
        frame = fread(s, num_bytes, 'uint8');

        if (frame(3) == 16 && (frame(end-4) == 'R') && (frame(end-3) == 'S') && (frame(end-2) == 'T')) % reset everything for plotting
            curr_x = start_x;
            curr_y = start_y;
            dest = frame(end-1);
            destination = [graph.nodes(dest,1) graph.nodes(dest,2)];
            set(d, 'XData', destination(1), 'YData', destination(2));
            pos = addPos(pos, [curr_x ; curr_y]);
        elseif (frame(3) == 24) %parse location
            curr_x = typecast(uint8( flip(frame((num_bytes-12):(num_bytes-9))) ), 'single') + start_x;
            curr_y = typecast(uint8( flip(frame((num_bytes-8):(num_bytes-5))) ), 'single') + start_y;
            % heading = typecast(uint8( flip(frame((end-4):(end-1))) ), 'single');
            pos = addPos(pos, [curr_x ; curr_y]);
        end

        % update plot
        set(h, 'XData', pos(1,:), 'YData', pos(2,:));
        %pause(0.1);
    
        i = i + 1;
        if i == (size(test_pos,2) + 1)
            i = 1;
        end
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

