function [x,y,th,D,delta] = HybridAStar(Start,End,Vehicle,Configure,ObstInfo)  
    veh = Vehicle;
    cfg = Configure; 
    costmap = GridAStar(ObstInfo.ObstList,End,Configure.XY_GRID_RESOLUTION);
    % use grid a star search result as heuristic cost(hn)
    
    [xidx,yidx,thidx] = state2grid(Start,cfg);  
    StartIdx = [xidx,yidx,thidx];
    
    % open set
    % [x | y | theta | actual cost(gn) | xIdx | yIdx | thetaIdx | direction
    % | Steer Angle]
    gres    = cfg.XY_GRID_RESOLUTION;
    yawres  = cfg.YAW_GRID_RESOLUTION;
    MAX_X   = ceil((cfg.MAXX-cfg.MINX)/gres);
    MAX_Y   = ceil((cfg.MAXY-cfg.MINY)/gres);
    MAX_YAW = ceil((cfg.MAXYAW-cfg.MINYAW)/yawres);
    PathTab = zeros(MAX_X, MAX_Y, MAX_YAW, 8);
    Open  = [Start(1),Start(2),Start(3), 0, xidx, yidx, thidx, cfg.MOTION_RESOLUTION, 0];
    Close = [];
    lastNode  = [];
    foundPath = 0;
    while ~isempty(Open)
        % pop the least cost node from open to current node
        minIdx = findMinCostNode(Open, cfg, costmap);
        wknode = Open(minIdx, :);
        Open(minIdx, :) = [];
        
        % push current node to close set
        % if current node is already in close set, update
        idx = findNodeInTable(wknode,Close);
        if ~isempty(idx)
            Close(idx,:) = wknode;
        else
            Close = [Close;wknode];
        end 
        
        % try to connect current node to Target point 
        path = canConnectToTarget(wknode(1:3),End,veh,cfg,ObstInfo);
        if  ~isempty(path)
            foundPath = 1;
            lastNode  = wknode;
            break
        end
        
        % find the next node
        mres = cfg.MOTION_RESOLUTION; % motino resolution
        smax = veh.MAX_STEER; % maximum steering angle
        sres = smax/cfg.N_STEER; % steering resolution
        for D = [-mres,mres]
            for delta = [-smax:sres:-sres,0,sres:sres:smax]
                tnode = CalcNextNode(wknode,D,delta,veh,cfg,ObstInfo);
                % next node is invalid or already in close set
                if isempty(tnode) || (~isempty(findNodeInTable(tnode,Close)))
                    continue
                end
                idx = findNodeInTable(tnode,Open);
                if ~isempty(idx)
                    % exist in open set, compare actual cost and update
                    exist_node = Open(idx,:);
                    if tnode(4) < exist_node(4)
                        Open(idx, :) = tnode;
                        PathTab(tnode(5), tnode(6),tnode(7),  :) = wknode([5,6,7,1,2,3,8,9]);
                    end
                else
                    % new node, add to open set
                    Open = [Open; tnode];
                    PathTab(tnode(5), tnode(6), tnode(7), :) = wknode([5,6,7,1,2,3,8,9]);
                end
            end
        end
    end   
    
    [x,y,th,D,delta] = deal([]);
    if foundPath == 1
        [x,y,th,D,delta] = getFinalPath(StartIdx, lastNode, path,PathTab,veh,cfg);
    end
end

function [xidx,yidx,thidx] = state2grid(State,cfg)
    x      = State(1);
    y      = State(2);
    theta  = State(3);
    gres   = cfg.XY_GRID_RESOLUTION;
    yawres = cfg.YAW_GRID_RESOLUTION;
    xidx   = ceil((x-cfg.MINX)/gres);
    yidx   = ceil((y-cfg.MINY)/gres);
    theta  = mod2pi(theta);
    thidx  = ceil((theta-cfg.MINYAW)/yawres);
    if (xidx <=0 || xidx > ceil((cfg.MAXX-cfg.MINX)/gres))||...
            (yidx <=0 || yidx > ceil((cfg.MAXY-cfg.MINY)/gres))
        [xidx,yidx,thidx]=deal([]);
    end
end

function idx = findMinCostNode(nodes, cfg, costmap)
    gres = cfg.XY_GRID_RESOLUTION;
    gn = nodes(:, 4);
    hn = zeros(size(nodes,1),1);
    for i = 1:length(hn)
        x      = nodes(i,1);
        y      = nodes(i,2);
        xIdx   = nodes(i,5);
        yIdx   = nodes(i,6);
        hcost  = cfg.H_COST*costmap(yIdx, xIdx);
        xshift = x - (gres*(xIdx-0.5)+cfg.MINX);
        yshift = y - (gres*(yIdx-0.5)+cfg.MINY);
        hn(i)  = hcost + cfg.H_COST*norm([xshift,yshift]);
    end
    [~, idx] = min(gn+hn);
end

function idx = findNodeInTable(wknode,Tab)
    if isempty(Tab)
        idx = [];
        return
    end
    idx = find(wknode(5) == Tab(:,5) & wknode(6) == Tab(:,6) & wknode(7) == Tab(:,7), 1);
end

function tnode = CalcNextNode(wknode,D,delta,veh,cfg,ObstInfo)
    px  = wknode(1);
    py  = wknode(2);
    pth = wknode(3);
    
    % check path from wknode to next node is Collision free
    gres = cfg.XY_GRID_RESOLUTION;
    obstline = ObstInfo.ObstLine;
    nlist = floor(gres*1.5/cfg.MOTION_RESOLUTION)+1;
    for idx = 1:nlist
        [px,py,pth] = VehicleDynamic(px,py,pth,D,delta,veh.WB);
        if rem(idx,5) == 0
            tvec = [px,py,pth];
            isCollision = VehicleCollisionCheck(tvec,obstline,veh);
            if isCollision
                tnode = [];
                return;
            end
        end
    end
    
    [xidx,yidx,thidx] = state2grid([px,py,pth],cfg);
    if isempty(xidx)
        tnode = [];
        return;
    end
    % calc next node cost;
    prev_cost  = wknode(4);
    prev_Dir   = wknode(8);
    prev_Theta = wknode(9);
    if D > 0
        cost = prev_cost + gres*1.5;
    else
        cost = prev_cost + cfg.BACK_COST*gres*1.5;
    end
    if D ~= prev_Dir
        cost = cost + cfg.SB_COST;
    end
    cost = cost + cfg.STEER_COST*abs(delta);
    cost = cost + cfg.STEER_CHANGE_COST*abs(delta-prev_Theta);
    tnode = [px,py,pth,cost,xidx,yidx,thidx,D,delta];
end

function [x,y,theta] = VehicleDynamic(x,y,theta,D,delta,L)
    x = x+D*cos(theta);
    y = y+D*sin(theta);
    theta = theta+D/L*tan(delta);
    theta = mod2pi(theta);
end

function v = mod2pi(x)
    v = rem(x,2*pi);
    if v < -pi
        v = v+2*pi;
    elseif v > pi
        v = v-2*pi;
    end
end

function [x,y,th,D,delta] = getFinalPath(StartIdx, lastNode, path,PathTab,veh,cfg) 
    node_x = lastNode(5:7);
    nodes = lastNode([5,6,7,1,2,3,8,9]);
    while ~isequal(node_x, StartIdx)
        node = PathTab(node_x(1),node_x(2),node_x(3),:);
        node = node(:)';
        nodes = [nodes; node];
        node_x = node(1:3);
    end
    nodes(end, :) = [];
    nodes = flip(nodes);
    
    rmin = veh.MIN_CIRCLE;
    smax = veh.MAX_STEER;
    mres = cfg.MOTION_RESOLUTION;
    gres = cfg.XY_GRID_RESOLUTION;
    % decrease one step, caz node origin is consider in
    nlist = floor(gres*1.5/cfg.MOTION_RESOLUTION)+1;
    x = [];
    y = [];
    th = [];
    D = [];
    delta = [];
    flag = 0;
    
    if size(nodes,1) >= 2
        % caz first node is start point, we ignore this node
        for i = 1:size(nodes,1)-1
            tnode = nodes(i,:);
            ttnode = nodes(i+1,:);
            % initial point
            px = tnode(4);
            py = tnode(5);
            pth = tnode(6);
            cur_D = ttnode(7);
            cur_Delta = ttnode(8);
            x = [x, px];
            y = [y, py];
            th = [th, pth];
            D = [D, cur_D];
            delta = [delta, cur_Delta];
            
            for idx = 1:nlist
                [px,py,pth] = VehicleDynamic(px,py,pth,cur_D,cur_Delta,veh.WB);
                x = [x, px];
                y = [y, py];
                th = [th, pth];
                D = [D, cur_D];
                delta = [delta, cur_Delta];
            end
            % delete last point
            if i ~= size(nodes,1)-1
                x(end) = [];
                y(end) = [];
                th(end) = [];
                D(end) = [];
                delta(end) = [];
            end
        end
    else
        % if just rs path so we will get path without node
        flag = 1;
        tnode = nodes(1,:);
        px = tnode(4);
        py = tnode(5);
        pth = tnode(6);
        x = [x, px];
        y = [y, py];
        th = [th, pth];
    end
    
    % add the path from lastNode to targetNode
    types = path.type;
    t = rmin*path.t;
    u = rmin*path.u;
    v = rmin*path.v;
    w = rmin*path.w;
    segs = [t,u,v,w,rmin*path.x;];% avoid duplicate of x
    for i = 1:5
        if segs(i) == 0
            continue
        end
        s = sign(segs(i));
        tdelta = calcSteerFromType(types(i), smax);
        if flag == 1
            % initialization if only rs path
            D = [D, s*mres];
            delta = [delta, tdelta];
            flag = 0;
        end
        for idx = 1:round(abs(segs(i))/mres)                
           	[px,py,pth] = VehicleDynamic(px,py,pth,s*mres,tdelta,veh.WB);
            x = [x, px];
            y = [y, py];
            th = [th, pth];
            D = [D, s*mres];
            delta = [delta, tdelta];         
        end
    end
end

function path = canConnectToTarget(curNode,targetNode,Vehicle,Configure,ObstInfo)
%   check is there a path from curNode to targetNode derectly.
    pvec = targetNode-curNode;
    x = pvec(1);
    y = pvec(2);
    phi = curNode(3);
    phi = mod2pi(phi);
    dcm = angle2dcm(phi, 0, 0);
    tvec = dcm*[x; y ; 0];
    x = tvec(1);
    y = tvec(2);
    path = FindRSPath(x,y,pvec(3),Vehicle);
    pth = pathCollision(curNode, path, Vehicle, Configure, ObstInfo.ObstLine);
    if isempty(pth) || (mod2pi(pth) - targetNode(3)) > deg2rad(5)
       path = []; 
    end
end

function delta = calcSteerFromType(type, smax)
    switch type
        case 'S'
            delta = 0;
        case 'L'
            delta = smax;
        case 'R'
            delta = -smax;
        case default
            delta = 0;
    end
end

function pth = pathCollision(Start, path, veh, cfg, obstline)
% check the path return by FindRSPath is Collision free
    rmin = veh.MIN_CIRCLE;
    types = path.type;
    t = rmin*path.t;
    u = rmin*path.u;
    v = rmin*path.v;
    w = rmin*path.w;
    x = rmin*path.x;
    segs = [t,u,v,w,x];
    pvec = Start;
    for i = 1:5
        if segs(i) == 0
            continue
        end
        px =pvec(1);
        py = pvec(2);
        pth = pvec(3);
        s = sign(segs(i));
        mres = cfg.MOTION_RESOLUTION;
        D = s*mres;
        smax = veh.MAX_STEER;
        delta = calcSteerFromType(types(i), smax);
        for idx = 1:round(abs(segs(i))/mres)                
           	[px,py,pth] = VehicleDynamic(px,py,pth,D,delta,veh.WB);
            if rem(idx,5) == 0
                tvec = [px,py,pth];
                if VehicleCollisionCheck(tvec,obstline,veh)
                    pth = [];
                    return
                end
            end
        end
        pvec = [px,py,pth];
    end
end