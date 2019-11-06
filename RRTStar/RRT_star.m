%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% ?????
clear all; close all;
x_I=1; y_I=1;           % ?????
x_G=700; y_G=700;       % ?????
Thr=50;                 % ???????
Delta= 30;              % ??????
NearDelta= 60;          % ??near?????
%% ?????
T.v(1).x = x_I;         % T????????v??????????????T???
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % ??????????????
T.v(1).yPrev = y_I;
T.v(1).dist=0;          % ????????????????????
T.v(1).rootDist=0;      % ????????????????????
T.v(1).indPrev = 0;     %
T.v(1).p = [];          % ?????????????

%% ?????——????
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);
yL=size(Imp,2);
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
count=1;
found = 0;
updateFlag = 0;
solutionEndIdx = -1;
solutionIdxSet = [];
solutionEllipse = inf;

for iter = 1:4000
    %Step 1: ???????????x_rand,?????????????
    x_rand=[xL, yL].*rand(1,2);
    if norm(x_rand - [x_I,y_I])+norm(x_rand - [x_G,y_G])> solutionEllipse
        continue;
    end
    
    %Step 2: ?????????????? x_phase1_near 
    x_phase1_near=[];
    min_dis_toT = inf;
    for i = 1:length(T.v)
        t_node = T.v(i);
        t_nodexy = [t_node.x, t_node.y];
        dis_toT = norm(t_nodexy - x_rand);
        if dis_toT < min_dis_toT
            min_dis_toT = dis_toT;
            x_phase1_near = t_nodexy;
        end
    end
    
    %Step 3: ????x_new??
    dxy = x_rand - x_phase1_near;
    dxynorm = dxy/norm(dxy);
    x_new = x_phase1_near + dxynorm*Delta;
    if ~collisionChecking(x_new,[],Imp)
        continue;
    end

    %Step 3: ??????x_new?????x_link,???near??
    near_set = [];
    min_dis_toT = inf;
    x_link_idx = -1;
    for i = 1:length(T.v)
        t_node = T.v(i);
        t_nodexy = [t_node.x, t_node.y];
        dis_toT = norm(t_nodexy - x_new);
        if dis_toT < NearDelta && collisionChecking(x_new,t_nodexy,Imp)
            near_set = [near_set;t_nodexy, i];
            curDist = dis_toT + T.v(i).rootDist;
            if min_dis_toT > curDist
                min_dis_toT = curDist;
                x_link_idx = i;
            end
        end
    end
    if x_link_idx == -1
        continue;
    end 
    x_link = [T.v(x_link_idx).x,T.v(x_link_idx).y];

    %Step 4: ?x_new???T 
    %??????x_new?????x_link
    count=count+1;
    T.v(count).x = x_new(1);         
    T.v(count).y = x_new(2);
    T.v(count).xPrev = x_link(1);     
    T.v(count).yPrev = x_link(2);
    T.v(count).dist=norm(x_new - x_link);
    T.v(count).rootDist= T.v(x_link_idx).rootDist + norm(x_new - x_link);
    T.v(count).indPrev = x_link_idx;     
    T.v(count).p = plot([x_link(1), x_new(1)],[x_link(2), x_new(2)], 'r', 'marker', '.');
    drawnow;
    
    %Step 5:??rewire
    updateFlag = 0;
    x_new_idx = count;
%     disp(['cur is ', num2str(curNodeIdx), 'link to ', num2str(selected_nea_idx)])
    for i = 1:size(near_set, 1)
        node_xy = near_set(i,1:2);
        node_idx = near_set(i,3);
        % ???x_new???
        if node_idx == x_new_idx
            continue;
        end
        dist = norm(x_new - node_xy);
        if  T.v(node_idx).rootDist > T.v(x_new_idx).rootDist + dist
            updateFlag = 1;
            T.v(node_idx).rootDist = T.v(x_new_idx).rootDist + dist;
            T.v(node_idx).xPrev = x_new(1);
            T.v(node_idx).yPrev = x_new(2);
            T.v(node_idx).dist = dist; 
            T.v(node_idx).indPrev = x_new_idx;
            delete(T.v(node_idx).p)
            T.v(node_idx).p = plot([x_new(1), node_xy(1)],[x_new(2), node_xy(2)], 'r', 'marker', '.');
            drawnow;
        end
    end
    
    %Step 6:????????????? 
    if norm(x_new-[x_G,y_G]) < Thr && collisionChecking(x_new,[x_G,y_G],Imp) && found == 0
        % ??[x_G y_G]??
        count=count+1;
        solutionEndIdx = count;
        T.v(count).x = x_G;         
        T.v(count).y = y_G;
        T.v(count).xPrev = x_new(1);     
        T.v(count).yPrev = x_new(2);
        T.v(count).dist=norm(x_new - [x_G,y_G]);
        T.v(count).rootDist= T.v(x_new_idx).rootDist + norm(x_new - [x_G,y_G]);
        T.v(count).indPrev = x_new_idx;     
        T.v(count).p = plot([x_new(1), x_G],[x_new(2), y_G], 'r', 'marker', '.');
        found = 1;
        % ??[x_I x_I]?x_new ???????????
        pathIndex = solutionEndIdx;
        solutionIdxSet = [pathIndex];
        set(T.v(pathIndex).p, 'color', 'b','Linewidth', 3);
        for k = 1:1000
            pathIndex = T.v(pathIndex).indPrev;
            if pathIndex == 1
                break
            end
            solutionIdxSet = [solutionIdxSet, pathIndex];
            set(T.v(pathIndex).p, 'color', 'b','Linewidth', 3);
        end  % ????????
        drawnow;
    end
    
   %Step 7: ????????????????????????????????????
	if found == 1 && updateFlag == 1
        % ??????
        solutionEllipse = -inf;
        for i = 1:length(solutionIdxSet)  
            set(T.v(solutionIdxSet(i)).p, 'color', 'r','Linewidth', 1);
            node_xy = [T.v(solutionIdxSet(i)).x,T.v(solutionIdxSet(i)).y];
            if solutionEllipse < norm(node_xy - [x_I,y_I])+norm(node_xy - [x_G,y_G])
                solutionEllipse = norm(node_xy - [x_I,y_I])+norm(node_xy - [x_G,y_G]);
            end
        end
        
        % ???????????
        pathIndex = solutionEndIdx;
        solutionIdxSet = [pathIndex];
        set(T.v(pathIndex).p, 'color', 'b','Linewidth', 3);
        for k = 1:1000
            pathIndex = T.v(pathIndex).indPrev;
            if pathIndex == 1
                break
            end
            solutionIdxSet = [solutionIdxSet, pathIndex];
            set(T.v(pathIndex).p, 'color', 'b','Linewidth', 3);
        end  % ????????
        drawnow;
    end
end

