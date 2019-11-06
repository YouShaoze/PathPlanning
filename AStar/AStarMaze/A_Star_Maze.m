%% load map
clear;
ImpRgb = imread('maze.png');
Imp    = rgb2gray(ImpRgb);
Imp    = im2bw(Imp)*255;
MAX_X=size(Imp,1);
MAX_Y=size(Imp,2);
distanceFcn = @(p1,p2) norm(p1-p2);
%% AStar
GlbTab      = zeros(MAX_X, MAX_Y);  % 0|new 1|open 2|close
PathTab     = zeros(MAX_X, MAX_Y, 2);
nodeStartXY  = [1, 1];
nodeTargetXY = [250, 250];
startGn     = 0;
startHn     = distanceFcn(nodeTargetXY,nodeStartXY);
startFn     = startGn + startHn;
% [fn | gn | hn | x | y]
nodeStart  = [startFn, startGn, startHn, nodeStartXY]; 

%% loop
openset = [nodeStart];
foundpath = 0;
while(~isempty(openset))
    [~,minIdx] = min(openset(:,1));
    node = openset(minIdx,:);
    openset(minIdx,:) = [];
    if isequal(node(4:5), nodeTargetXY) 
       foundpath = 1;
       break;
    end
    node_x  = node(4);
    node_y  = node(5);
    node_gn = node(2);
    GlbTab(node_x, node_y) = 2;
    
    for k= 1:-1:-1
        for j= 1:-1:-1
            if (k~=j || k~=0)  %The node itself is not its successor
                s_x  = node_x+k;
                s_y  = node_y+j;
                if((s_x >0 && s_x <=MAX_X) && (s_y >0 && s_y <=MAX_Y))%node within array bound
                    % exist close node
                    if GlbTab(s_x, s_y) == 2 || Imp(s_x,s_y) == 0 
                        continue;
                    end
                    s_gn = node_gn + distanceFcn([node_x,node_y], [s_x,s_y]);
                    s_hn = distanceFcn(nodeTargetXY, [s_x,s_y]);
                    s_fn = s_gn + s_hn;
                    if GlbTab(s_x, s_y) == 0
                        % new node
                        GlbTab(s_x, s_y) = 1;
                        openset = [openset; [s_fn, s_gn, s_hn, s_x, s_y]];
                        PathTab(s_x, s_y, :) = [node_x, node_y];
                    else
                        % exist open node
                        x_set = openset(:,4);
                        y_set = openset(:,5);
                        existIdx = find(x_set == s_x & y_set ==s_y, 1);
                        assert(~isempty(existIdx))
                        exist_gn = openset(existIdx, 2);
                        if exist_gn > s_gn
                            openset(existIdx,:) = [s_fn, s_gn, s_hn, s_x, s_y];
                            PathTab(s_x, s_y, :) = [node_x, node_y];
                        end
                    end
                end
            end
        end
    end
end

%% get path
if foundpath == 1
    node_x = nodeTargetXY;
    Imp_R = Imp;
    Imp_G = Imp;
    Imp_B = Imp;
    while ~isequal(node_x, nodeStartXY)
        Imp_R(node_x(1),node_x(2)) = 255;
        Imp_G(node_x(1),node_x(2)) = 0;
        Imp_B(node_x(1),node_x(2)) = 0;
        node_x = PathTab(node_x(1),node_x(2),:);
        node_x = node_x(:)';
    end
    figure(1);
    imshow(cat(3, Imp_R,Imp_G,Imp_B))
else
    disp('Not find solution')
end
