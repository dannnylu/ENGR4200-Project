bw = imread('knappMap.png');   % input of a png image file
gbw = im2gray(bw);  % convert to grayscale
ebw = edge(gbw);    % find edges of image
ebw_h = height(ebw);    % get image height and width in pixels
ebw_w = width(ebw);
imshow(ebw)
title('Original image')
g = binaryImageGraph(ebw);  % transform png to binary image
figure
plotImageGraph(g)

[y,x]=find(ebw);    % Get coords of pixels in binary image
y = y*(-1)+ebw_h;   % move image to 1st quadrant
v = voronoi(x,y);   % find Voronoi line from points
L2 = v(2);
L2x = get(L2, 'XData');     % Get x and y coords of Voronoi line
L2y = get(L2, 'YData');

hold on

p = [x y];                      % used to show Voronoi calculations
DT = delaunayTriangulation(p);
[V,r] = voronoiDiagram(DT);
triplot(DT)

hold on

i = 1;                                  % Filters out any voronoi points-
j = 1;                                  % -outside of image bounds.
edgeL = (length(L2x)+length(L2y))/2;
while i <= edgeL    % "for every edge detected..."
     tempX = L2x(i);
     tempY = L2y(i);                    % VVV if tempX and tempY in bounds
     if and((0 <= tempX) & (tempX <= ebw_w),(0 <= tempY) & (tempY <= ebw_h))
         fltrX(j) = tempX;              % Then store in fltrX and fltrY
         fltrY(j) = tempY;
         j = j+1;
     end
     i = i+1;
end

fInput = [fltrX; fltrY];            % Filters out any repeating points from-
fltrIn = unique(fInput','rows');    % -fltrX and fltrY to make fInput.
fX = fltrIn(:,1);
fY = fltrIn(:,2);
scatter(fX, fY);

hold on

i = 1;                              % Filters out any Voronoi points that-
j = 1;                              % -fall too close or inside of walls. 
k = 1;
thresh = 8;     % variable to determine distance from wall 
pWrite = false;
fltrH = (height(fX)+height(fY))/2;
boundH = (height(x)+height(y))/2;
while i <= fltrH    % "for every voronoi point..."
    tempX = fX(i);
    tempY = fY(i);
    xHigh = tempX+thresh;   % creates bounds for logic
    xLow = tempX-thresh;
    yHigh = tempY+thresh;
    yLow = tempY-thresh;
    pWrite = true;
    j = 1;
    while j<= boundH    % "for every binary image, 'wall', point..."
        refX = x(j);
        refY = y(j);    % VVV if that point is within the bounds
        if and((refX <= xHigh) & (xLow <= refX),(refY <= yHigh) & (yLow <= refY))
            pWrite = false;   % then do not store it and break this while
            break
        end
        j = j+1;
    end
    if pWrite == true    % if never changed to forbid storing
        f2X(k) = tempX;  % then store x and y values
        f2Y(k) = tempY;
        k = k+1;
    end
    i = i+1;
end 
scatter(f2X, f2Y);

hold off

%tX = input('Input goal X = ')      % Input a target location of map 
%tY = input('Input goal Y = ')
pNum = length(x);
map = binaryOccupancyMap(ebw_w,ebw_h,1);    % generate map of filered binary image
setOccupancy(map, [x y], ones(pNum,1));
inflate(map, 10);
figure;
show(map);
planner = plannerAStarGrid(map);   
start = [200 100];   % preset as bot home
goal = [400 400];   % ONLY preset for example
path = plan(planner,start,goal);    % calculates shortest path of start->end
show(planner);
start = goal;
figure



i = 1;                              % Increases space between path coordinates-
j = 1;                              % -and fixes y value for 1st quadrant.
pathStep= 10;   % used to determine how many steps are allowed
pathH = height(path);
while i<=pathH  % "for every path point..."
    sPath(j,:) = path(i,:);
    sPath(j,1) = ebw_h-sPath(j,1);
    j = j+1;
    i = i+pathStep;
end
sPath(j,2) = path(pathH,2);     % ensures goal is final move command
sPath(j,1) = ebw_h - path(pathH,1);

i = 1;                              % Compares path points to Voronoi points-
j = 1;                              % -and assigns path point as closest-
sPathH = height(sPath);             % -Voronoi point.
fVoro = [f2X' f2Y'];
fVoroH = height(fVoro);
while (i <= sPathH)     % "for every filtered path point..."
    tempP = sPath(i,:);
    j = 1;
    pPathDist = 1000000000;
    while (j <= fVoroH)     % "for every Voronoi point..." %VVV Calculate distance 
        pDist = sqrt(((tempP(1,2)-fVoro(j,1))^2)+((tempP(1,1)-fVoro(j,2))^2));
        if (pDist <= pPathDist)     % if distance is shorter than the last point
            pPathDist = pDist;      % then store as new closest Voronoi point
            vPathX = fVoro(j,1);
            vPathY = fVoro(j,2);
        end
        j = j+1;
    end
    fPath(i,:) = [vPathX vPathY];
    i = i+1;
end
fPath(i,1) = sPath(sPathH,2);   % Again assure goal is last move
fPath(i,2) = sPath(sPathH,1);
scatter(fPath(:,1),fPath(:,2))



writematrix(fPath,'pathdata.txt');  % Alternative way to communicate move commands-
type pathdata.txt;                  % -by generating .txt file.

% rosinit                           % Origional idea of sending commands-
%                                   % -via a ROS Master node,nonfunctional.
% datapub = rospublisher("/chatter","std_msgs/String","DataFormat","struct");
% msg = rosmessage(datapub);
% msg.Data = 'test phrase';
% send(datapub,msg);
% 
% rosshutdown
