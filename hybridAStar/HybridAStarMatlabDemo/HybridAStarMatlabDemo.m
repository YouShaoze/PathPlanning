clear;
load parkingLotCostVal.mat     % costVal
width  = 2;
height = 4;

vehicleDims  = vehicleDimensions(2*height,2*width);
distFromSide = 0.175;
centerPlacements = [distFromSide 0.5 1-distFromSide];
inflationRadius  = 1.2;

ccConfig = inflationCollisionChecker(vehicleDims,'CenterPlacements',centerPlacements,'InflationRadius',inflationRadius);
ccConfig.InflationRadius = 1.3;

map = vehicleCostmap(costVal);
map.CollisionChecker  = ccConfig;
map.FreeThreshold     = 0.5;
map.OccupiedThreshold = 0.9;

validator = validatorVehicleCostmap(stateSpaceSE2);
validator.Map = map;

planner   = plannerHybridAStar(validator,'MinTurningRadius',10,'MotionPrimitiveLength',6);
startPose = [85 75 0]; % [meters, meters, radians]
goalPose  = [90 54 -pi/2];
refpath   = plan(planner,startPose,goalPose);

plot(map,'Inflation','off');
hold on
for i = 1:refpath.NumStates
    h = draw(refpath.States(i,:),width,height);
    drawnow();
    pause(0.1);
    if i <refpath.NumStates
        delete(h);
    end
end

function h = draw(pos,width,height)
    x = pos(1);
    y = pos(2);
    cosT = cos(pos(3));
    sinT = sin(pos(3));
    V = [ cosT, sinT];
    T = [-sinT, cosT];
    plot(x,y,'.');
    height1 = height;
    height2 = height;
    p1 = [x,y]+width*T+height1*V;
    p2 = [x,y]-width*T+height1*V;
    p3 = [x,y]+width*T-height2*V;
    p4 = [x,y]-width*T-height2*V;
    h = plot([p1(1),p2(1),p4(1),p3(1),p1(1)],[p1(2),p2(2),p4(2),p3(2),p1(2)],'k');
end