% AreaMeasurement(video,frame)
% ScaleIdentification(video,1);

%% pre processing
clc;
clear;
close all;
global cellNumList
cellNumList = [];
filePath = fullfile('VideoFile','20200615_111546.mp4');
video = VideoReader(filePath);

frameEnd = video.NumFrames;
scalebar = ScaleIdentification(video,1);
% scalebar = 1;

startFrame = 2;

frame=startFrame;
while true
    [operation, frame] = videoPlayer(video, frame, frameEnd,startFrame);
    switch operation
        case 1 % cell length
            [cellNum, frame, lengthCellX, lengthCellY, ...
                lengthCellX_actual, lengthCellY_actual, lengthCell, lengthCell_actual]=AreaMeasurement2(video,frame,scalebar);
            cellNumList(cellNum) = cellNum;
            cellList(cellNum).frame_length = frame;
            %             cellList(cellNum).lengthCellX = lengthCellX;
            %             cellList(cellNum).lengthCellY = lengthCellY;
            %             cellList(cellNum).lengthCellX_actual = lengthCellX_actual;
            %             cellList(cellNum).lengthCellY_actual = lengthCellY_actual;
            cellList(cellNum).lengthCell = lengthCell;
            cellList(cellNum).lengthCell_actual = lengthCell_actual;
            clear lengthCellX lengthCellY lengthCellX_actual lengthCellX_actual lengthCell lengthCell_actual
        case 2 % distance
            [cellNum, frame, pixelLength, actualLength, pixelLength_x,...
                actualLength_x, pixelLength_y, actualLength_y] = DistanceMeasurement(video,frame,scalebar);
            cellNumList(cellNum) = cellNum;
            cellList(cellNum).frame_distance = frame;
            cellList(cellNum).pixelDistance = pixelLength;
            cellList(cellNum).actualDistance = actualLength;
            cellList(cellNum).pixelDistance_x = pixelLength_x;
            cellList(cellNum).actualDistance_x = actualLength_x;
            cellList(cellNum).pixelDistance_y = pixelLength_y;
            cellList(cellNum).actualDistance_y = actualLength_y;
            clear pixelLength actualLength pixelLength_x actualLength_x pixelLength_y actualLength_y
        case 3 % speed
            [cellNum, frame, endFrame, SpeedResult,vx_actual,vy_actual] = SpeedMeasurement(video,frame,scalebar);
            cellNumList(cellNum) = cellNum;
            cellList(cellNum).frame_speed = frame;
            cellList(cellNum).frame_speed_end = endFrame;
            cellList(cellNum).SpeedResult = SpeedResult;
            cellList(cellNum).vx_actual=vx_actual;
            cellList(cellNum).vy_actual=vy_actual;
            
        case 4 % area
            [cellNum, frame, Area]=AreaMeasurement(video,frame,scalebar);
            cellNumList(cellNum) = cellNum;
            cellList(cellNum).frame_area = frame;
            cellList(cellNum).Area = Area;
            clear Area
        case 5 % length manual
            [cellNum, frame, Xlength, Ylength, Xlength_actual, Ylength_actual]=CellLength(video,frame,scalebar);
            cellNumList(cellNum) = cellNum;
            cellList(cellNum).frame_length = frame;
            cellList(cellNum).Xlength = Xlength;
            cellList(cellNum).Ylength = Ylength;
            cellList(cellNum).Xlength_actual = Xlength_actual;
            cellList(cellNum).Ylength_actual = Ylength_actual;
            cellList(cellNum).Length_actual = sqrt(Xlength_actual^2+Ylength_actual^2);
            clear Xlength Ylength Xlength_actual Ylength_actual
        case 6 % average speed
            [cellNum, frame, endFrame, SpeedResult,vx_actual,vy_actual] = averageSpeed(video,frame,scalebar);
            cellNumList(cellNum) = cellNum;
            cellList(cellNum).frame_speed = frame;
            cellList(cellNum).frame_speed_end = endFrame;
            cellList(cellNum).SpeedResult = SpeedResult;
            cellList(cellNum).vx_actual=vx_actual;
            cellList(cellNum).vy_actual=vy_actual;
        case 9
            break;
        otherwise
    end
    
    % data struct
    
end







%%
function currentCells()
global cellNumList
disp('current cells:')
if isempty(cellNumList)
    disp('No cells')
else
    disp(cellNumList)
end
end


function [x, i]= videoPlayer(video, frameStart, frameEnd, startFrame)
figure(1)
clc;
prompt = 'Input num. for the required operations: ';

PlayDirection = 1;
i=frameStart;
while true
    videoFrame = read(video,i);
    imshow(videoFrame);
    title(['Frame: ',num2str(i)])
    functionList();
    keypressed = getkey(1,'non-ascii');
    
    if strcmp('0',keypressed)
        PlayDirection=input('input play steps: ');
        x = [];
    elseif strcmp('rightarrow',keypressed)
        i=i+PlayDirection;
    elseif strcmp('leftarrow',keypressed)
        i=i-PlayDirection;
    elseif strcmp('1',keypressed)
        x = 1;
        return;
    elseif strcmp('2',keypressed)
        x = 2;
        return;
    elseif strcmp('3',keypressed)
        x = 3;
        return;
    elseif strcmp('4',keypressed)
        x = 4;
        return;
    elseif strcmp('5',keypressed)
        x = 5;
        return;
    elseif strcmp('6',keypressed)
        x = 6;
        return;
    elseif strcmp('9',keypressed)
        x = 9;
        return;
    else
        
        continue;
    end
    
    
    
    cla;
    clc;
    
    if i <= startFrame
        i = startFrame;
    end
    if i >= frameEnd
        i = frameEnd;
    end
end

return;

end
function [cellNum, frame, pixelLength, actualLength, pixelLength_x,...
    actualLength_x, pixelLength_y, actualLength_y] = DistanceMeasurement(video,frame,scalebar)
clc;
currentCells()
cellNum = inputGenerat('input cell number : ');
figure(2)
videoFrame = read(video,frame);
imshow(videoFrame);hold on;
[x,y] = two_point_region();
pixelLength = sqrt((x(1)-x(2))^2+(y(1)-y(2))^2);
actualLength = pixelLength * scalebar;

pixelLength_x = x(1)-x(2);
actualLength_x = pixelLength_x * scalebar;

pixelLength_y = y(1)-y(2);
actualLength_y = pixelLength_y * scalebar;


disp(['pixel length: ',num2str(pixelLength)])
disp(['actual length: ',num2str(actualLength)])
disp('press any key to continue')
pause();
clc

close 2
end
function [cellNum, frame, Area]=AreaMeasurement(video,frame,scalebar)
clc;
currentCells()
cellNum = input('input cell number : ');


figure(2)
videoFrame = read(video,frame);
imshow(videoFrame); hold on
[x,y] = four_point_region();
patch(x,y,'r','FaceAlpha',0.1);

figure(3);

imshow(videoFrame);hold on

xlim( [min(x), max(x)])
ylim( [min(y), max(y)])


disp('Input 1 to stop selection');
j=1;

AeraX= -1* ones(40,1);
AeraY= AeraX;
while true
    [AeraX(j),AeraY(j)] = ginput(1);
    plot(AeraX(j),AeraY(j), 'rx')
    ip = input('');
    if ip == 1
        break
    end
    j=j+1;
end
AeraX(AeraX == -1) = [];
AeraY(AeraY == -1) = [];

patch(AeraX,AeraY,'r','FaceAlpha',0.1);
Area = polyarea(AeraX,AeraY);
AreaReal = polyarea(AeraX*scalebar,AeraY*scalebar);
clc;
disp(['the area of the cell is ',num2str(Area),' pixels'])
disp(['or ',num2str(AreaReal)])
disp('press any key to continue')
pause();

close 3
close 2
return
end
function [x,y]= four_point_region()
hold on;
x=zeros(4,1);
y=zeros(4,1);
for j= 1:1:4
    [x(j),y(j)] = ginput(1);
    plot(x(j),y(j), 'rx')
end
return
end
function functionList()
disp('1, Measuring cell length')
disp('2, Measuring distance')
disp('3, Measuring speed')
disp('4, Measuring cell area (manual)')
disp('5, Measuring cell length (manual)')
disp('6, Measuring avergae speed (manual)')
disp('9, Exit')
disp('0, play settings')
end
function scalebar = ScaleIdentification(video,frame)
figure(1);
clc

disp('select two points to identify the scale of the video ')
videoFrame = read(video,frame);
imshow(videoFrame);hold on;
title('select two points to identify the scale of the video ')
[x,y]= two_point_region();
realLength = input('input actual length: ');
pixelLength = sqrt((x(1)-x(2))^2+(y(1)-y(2))^2);
scalebar = realLength/pixelLength;
close 1;
return
end
function [x,y]= two_point_region()
x=zeros(2,1);
y=zeros(2,1);
for j= 1:1:2
    [x(j),y(j)] = ginput(1);
    plot(x(j),y(j), 'b*')
end
plot(x,y,'b-')
return
end
function [cellNum, frame, lengthCellX, lengthCellY, ...
    lengthCellX_actual, lengthCellY_actual, lengthCell, lengthCell_actual]=AreaMeasurement2(video,frame,scalebar)
clc;
currentCells()
cellNum = inputGenerat('input cell number : ');

frameDiffStart = 20;
frameDiffEnd = 20;

frameStart = frame - frameDiffStart;
frameEnd = frame + frameDiffEnd;

if frameStart < 1
    frameStart = 1;
end
if frameEnd > video.NumFrames
    frameEnd = video.NumFrames;
end

videoFrames=zeros(video.Height,video.Width,3,  frameEnd - frameStart +1);
frameList = frameStart:1:frameEnd;
for i = 1:1:length(frameList)
    videoFrames(:,:,:,i) = read(video,frameList(i));
end
midbackground = uint8(median(videoFrames,4));
targetFrame = read(video,frame);

diffIM = rgb2gray(midbackground - targetFrame);

figure(2)
imshow(targetFrame)
title('four points region')
[x,y] = four_point_region();
patch(x,y,'r','FaceAlpha',0.1);
maxX = round(max(x));
minX = round(min(x));
maxY = round(max(y));
minY = round(min(y));
cla;


lengthCellX = 0;
lengthCellY = 0;
while true
    
    threshold = input('Set threshold (0~0.2, press enter to accept current result): ');
    
    if isempty(threshold)
        break;
    end
    
    cla;
    imshow(targetFrame)
    
    
    diffIMBW = imbinarize(diffIM(minY:maxY,minX:maxX),threshold);
    S = regionprops(diffIMBW,'Centroid'); %% center of the objects in the tube
    if length(S) >= 3
        [SI1,SI2]=maxdistance(S);
        plot( minX+[S(SI1).Centroid(1),S(SI2).Centroid(1)],minY+[S(SI1).Centroid(2),S(SI2).Centroid(2)],'r-')
        lengthCellX = S(SI1).Centroid(1)-S(SI2).Centroid(1);
        lengthCellY = S(SI1).Centroid(2)-S(SI2).Centroid(2);
    elseif length(S) == 2
        plot( minX+[S(1).Centroid(1),S(2).Centroid(1)],minY+[S(1).Centroid(2),S(2).Centroid(2)],'r-')
        lengthCellX = S(1).Centroid(1)-S(2).Centroid(1);
        lengthCellY = S(1).Centroid(2)-S(2).Centroid(2);
    elseif length(S) <= 1
        disp('no cell found, please reset threshold')
        lengthCellX = 0;
        lengthCellY = 0;
    end
end
lengthCellX = abs(lengthCellX);
lengthCellY = abs(lengthCellY);
lengthCellX_actual = lengthCellX*scalebar;
lengthCellY_actual = lengthCellY*scalebar;

if lengthCellX > lengthCellY
    lengthCell = lengthCellX;
else
    lengthCell = lengthCellY;
end
lengthCell_actual = lengthCell*scalebar;

close 2
return;
end
function dist = distance(S1, S2)
dist = sqrt((S1.Centroid(1)-S2.Centroid(1))^2 + (S1.Centroid(2)-S2.Centroid(2))^2);
return
end
function [SI1,SI2]=maxdistance(S)
index = 1;
for i=1:1:length(S)
    for j=1:1:length(S)
        distlist(index).l = distance(S(i), S(j));
        distlist(index).i = i;
        distlist(index).j = j;
        index = index + 1;
    end
end
[~,I] = max([distlist(:).l]);
SI1 = distlist(I).i;
SI2 = distlist(I).j;
end



function [cellNum, frame, endFrame, SpeedResult,vx_actual,vy_actual]=SpeedMeasurement(video,frame,scalebar)
vx_actual = [];
vy_actual = [];
currentCells()
cellNum = inputGenerat('input cell number : ');

figure(2)
videoFrame = read(video,frame);
imshow(videoFrame);
title('four points region');
disp('please select the region for speed measurement');
[x,y]= four_point_region();
patch(x,y,'r','FaceAlpha',0.1);
disp('press enter to continue')
pause()

cla;
figure(2)
clc;
PlayDirection = 1;
while true
    i=frame+1;
    disp('this is the start frame for speed measurement')
    
    while true
        clc;
        videoFrame = read(video,i);
        imshow(videoFrame);
        patch(x,y,'r','FaceAlpha',0.1);
        title(['Start Frame: ' ,num2str(frame),  '  Current Frame: ',num2str(i)])
        
        disp('0, play settings')
        disp('1, end frame of speed measurement')
        disp('9, exit')
        disp('input or press enter to go next frame: ');
        %         sel = input('input or press enter to go next frame: ');
        
        keypressed = getkey(1,'non-ascii');
        sel = 2;
        if strcmp(keypressed,'0')
            sel = 0;
        elseif strcmp(keypressed,'1')
            sel = 1;
        elseif strcmp(keypressed,'9')
            sel = 9;
        elseif strcmp('rightarrow',keypressed)
            i = i + PlayDirection;
            
        elseif strcmp('leftarrow',keypressed)
            i = i - PlayDirection;
        else
            
        end
        
        
        
        
        if i<frame+1
            i = frame+1;
        elseif i > video.NumFrames
            i = video.NumFrames;
        end
        
        switch sel
            case 1
                break;
            case 9
                close 2;
                endFrame=-1;
                SpeedResult = [];
                return;
            case 0
                %                 PlayDirection = input('input frame steps: ');
                PlayDirection=inputGenerat('input frame steps: ');
            otherwise
        end
        
        cla;
    end
    endFrame = i;
    clear i
    
    
    if endFrame <= frame +1
        disp('too short, please select again (press enter to continue)')
        pause()
    else
        break;
    end
end

maxX = round(max(x));
minX = round(min(x));
maxY = round(max(y));
minY = round(min(y));

while true % speed measurement
    %     threshold = input('Set threshold (0~0.2): ');
    threshold =inputGenerat('Set threshold (0~0.2): ');
    videoFrames=zeros(video.Height,video.Width,  endFrame - frame +1);
    frameList = frame:1:endFrame;
    for i = 1:1:length(frameList)
        videoFrames(:,:,i) = rgb2gray(read(video,frameList(i)));
    end
    midbackground = uint8(median(videoFrames,3));
    diffIM =  uint8(videoFrames) - midbackground;
    cutDiffIM = diffIM(minY:maxY,minX:maxX,:);
    diffIMBW = imbinarize(cutDiffIM,threshold);
    
    %     filterlevel = input('Set filter level (1~5): ');
    filterlevel =inputGenerat('Set filter level (1~5): ');
    SE = strel('diamond',filterlevel);
    
    centerPositionX = zeros(1,endFrame - frame +1);
    centerPositionY = zeros(1,endFrame - frame +1);
    indexremove = zeros(1,endFrame - frame +1);
    for i = 1:1:endFrame - frame +1
        targetIM = diffIMBW(:,:,i);
        erodedBW = imerode(targetIM, SE);
        BW= imdilate(erodedBW,SE);
        S = regionprops(BW,'Centroid'); %% center of the objects in the tube
        xposition = zeros(1,length(S));
        yposition = zeros(1,length(S));
        
        for j = 1:1:length(S)
            xposition(j) = S(j).Centroid(1);
            yposition(j) = S(j).Centroid(2);
        end
        centerPositionX(i) = mean(xposition);
        centerPositionY(i) = mean(yposition);
        indexremove(i) = length(S);
    end
    
    
    centerPositionX_actual = centerPositionX * scalebar;
    centerPositionY_actual = centerPositionY * scalebar;
    
    SpeedResult.centerPositionX_actual = centerPositionX_actual;
    SpeedResult.centerPositionY_actual = centerPositionY_actual;
    SpeedResult.centerPositionX = centerPositionX;
    SpeedResult.centerPositionY = centerPositionY;
    SpeedResult.frame = frameList;
    SpeedResult.time = frameList/video.FrameRate;
    
    iremove = indexremove == 0;
    for i = length(iremove):-1:1
        if iremove(i) == 1
            SpeedResult.time(i) = [];
            SpeedResult.centerPositionX_actual(i) = [];
            SpeedResult.centerPositionY_actual(i) = [];
            SpeedResult.centerPositionX(i) = [];
            SpeedResult.centerPositionY(i) = [];
        end
    end
    
    if length(SpeedResult.time)<2
        disp('too short to compute, press enter to exit, (try function 6)')
        pause()
        break;
    else
        fitobjectX = fit(SpeedResult.time',SpeedResult.centerPositionX_actual','poly1');
        fitobjectY = fit(SpeedResult.time',SpeedResult.centerPositionY_actual','poly1');
        SpeedResult.speedX = fitobjectX.p1;
        SpeedResult.speedY = fitobjectY.p1;
        
        
        figure(4)
        subplot(2,1,1)
        plot(fitobjectX,SpeedResult.time, SpeedResult.centerPositionX_actual)
        ylabel('position X')
        subplot(2,1,2)
        plot(fitobjectY,SpeedResult.time, SpeedResult.centerPositionY_actual)
        xlabel('time (s)')
        ylabel('position Y')
        
        disp(['speed X : ',num2str(fitobjectX.p1)])
        disp(['speed Y : ',num2str(fitobjectY.p1)])
        IIs = input('is this result acceptable? (press enter to accept, input 1 to decline)');
        if isempty(IIs)
            vx_actual = fitobjectX.p1;
            vy_actual = fitobjectY.p1;
            close 4
            break;
        else
            cla;
        end
        
    end
end


close 2
return;
end



function [cellNum, frame, Xlength, Ylength, Xlength_actual, Ylength_actual]=CellLength(video,frame,scalebar)
clc;
currentCells()
cellNum = inputGenerat('input cell number : ');


figure(2)
videoFrame = read(video,frame);
imshow(videoFrame); hold on
title ('four points region')
[x,y] = four_point_region();
patch(x,y,'r','FaceAlpha',0.1);

figure(3);

imshow(videoFrame);hold on

xlim( [min(x), max(x)])
ylim( [min(y), max(y)])


disp('select two points for cell length measurement')
[x1,y1]= two_point_region();
Xlength = x1(2)-x1(1);
Ylength = y1(2)-y1(1);

Xlength_actual = Xlength * scalebar;
Ylength_actual = Ylength * scalebar;





disp(['the X Length of the cell is ',num2str(Xlength),' pixels'])
disp(['or ',num2str(Xlength_actual)])

disp(['the Y Length of the cell is ',num2str(Ylength),' pixels'])
disp(['or ',num2str(Ylength_actual)])

disp('press any key to continue')
pause();

close 3
close 2
return
end


% function x=inputCellNum()
% a = true;
% while a
%     try
%         x = input('input cell number : ');
%         if isempty(x)
%             a = true;
%         else
%             a = false;
%         end
%     catch
%         warning('incorrect input!, try again');
%         a = true;
%     end
% end
%
% end

function x=inputGenerat(strrr)
a = true;
while a
    try
        x = input(strrr);
        if isempty(x)
            a = true;
        else
            a = false;
        end
    catch
        warning('incorrect input!, try again');
        a = true;
    end
end

end


function [cellNum, frame, endFrame, SpeedResult,vx_actual,vy_actual] = averageSpeed(video,frame,scalebar)
currentCells()
cellNum = inputGenerat('input cell number : ');
SpeedResult = [];
figure(2)
videoFrame = read(video,frame);
imshow(videoFrame);hold on
title('four points region');
disp('please select the cell');
% [x,y]= four_point_region();
% patch(x,y,'r','FaceAlpha',0.1);

[xx,yy]=ginput(1);
plot(xx,yy,'b*')


cla;
figure(2)
clc;
PlayDirection = 1;
while true
    i=frame;
    disp('this is the start frame for speed measurement')
    
    while true
        clc;
        videoFrame = read(video,i);
        imshow(videoFrame);hold on
        plot(xx,yy,'b*')
        title(['Start Frame: ' ,num2str(frame),  '  Current Frame: ',num2str(i)])
        
        disp('0, play settings')
        disp('1, end frame of speed measurement')
        disp('9, exit')
        disp('input or press enter to go next frame: ');
        %         sel = input('input or press enter to go next frame: ');
        
        keypressed = getkey(1,'non-ascii');
        sel = 2;
        if strcmp(keypressed,'0')
            sel = 0;
        elseif strcmp(keypressed,'1')
            sel = 1;
        elseif strcmp(keypressed,'9')
            sel = 9;
        elseif strcmp('rightarrow',keypressed)
            i = i + PlayDirection;
            
        elseif strcmp('leftarrow',keypressed)
            i = i - PlayDirection;
        else
            
        end
        
        
        
        
        if i<frame+1
            i = frame+1;
        elseif i > video.NumFrames
            i = video.NumFrames;
        end
        
        switch sel
            case 1
                break;
            case 9
                close 2;
                endFrame=-1;
                SpeedResult = [];
                return;
            case 0
                %                 PlayDirection = input('input frame steps: ');
                PlayDirection=inputGenerat('input frame steps: ');
            otherwise
        end
        
        cla;
    end
    endFrame = i;
    clear i
    
    
    if endFrame <= frame +0
        disp('too short, please select again (press enter to continue)')
        pause()
    else
        break;
    end
end


disp('please select the cell');
[xx2,yy2]=ginput(1);
plot(xx2,yy2,'b*')

dx=xx2-xx;
dy=yy2-yy;
dt=(endFrame-frame)/video.FrameRate;

vx = (dx/dt);
vy = (dy/dt);
vx_actual = (vx*scalebar);
vy_actual = (vy*scalebar);


clc
disp(['X speed ',num2str(vx_actual)])
disp(['Y speed ',num2str(vy_actual)])
disp('press enter to continue')
pause()
close 2
end