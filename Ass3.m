function PathTestWithAlternates
fig = figure('Name','Advanced Pathfinding System','Position',[100,100,900,600]);

% Axes where the map and paths are drawn
hAxes = axes('Parent',fig,'Units','pixels','Position',[250,100,600,450]);

% Inputs for grid size and obstacle density
uicontrol('Style','text','Position',[20,540,120,20],'String','Grid Size:','HorizontalAlignment','left');
gridSizeEdit = uicontrol('Style','edit','Position',[150,540,80,20],'String','16');
uicontrol('Style','text','Position',[20,500,120,20],'String','Obstacle Density (0-1):','HorizontalAlignment','left');
obstacleDensityEdit = uicontrol('Style','edit','Position',[150,500,80,20],'String','0.2');

% Start/End coordinate inputs
uicontrol('Style','text','Position',[20,420,120,20],'String','Start (x,y):','HorizontalAlignment','left');
startPointEdit = uicontrol('Style','edit','Position',[150,420,80,20],'String','1,1');
uicontrol('Style','text','Position',[20,380,120,20],'String','End (x,y):','HorizontalAlignment','left');
endPointEdit = uicontrol('Style','edit','Position',[150,380,80,20],'String','16,16');

% Buttons and dropdown
uicontrol('Style','pushbutton','Position',[20,460,120,30],'String','Generate Map',...
    'Callback',@(s,e) generateMap(gridSizeEdit,obstacleDensityEdit,hAxes));
uicontrol('Style','pushbutton','Position',[20,340,120,30],'String','Plot Start/End',...
    'Callback',@(s,e) plotStartEnd(startPointEdit,endPointEdit,hAxes));
uicontrol('Style','pushbutton','Position',[20,300,120,30],'String','Find Path',...
    'Callback',@(s,e) findPath(startPointEdit,endPointEdit,hAxes));
uicontrol('Style','text','Position',[20,260,120,20],'String','Select Path:','HorizontalAlignment','left');
pathDropdown = uicontrol('Style','popupmenu','Position',[150,260,120,22],'String',{'Select path'},...
    'Callback',@(s,e) highlightSelectedPath(hAxes));
setappdata(hAxes,'pathDropdown',pathDropdown);

uicontrol('Style','pushbutton','Position',[20,220,150,30],'String','Generate Alternative Path',...
    'Callback',@(s,e) findAlternativePath(startPointEdit,endPointEdit,hAxes));
uicontrol('Style','pushbutton','Position',[20,180,150,30],'String','Start Simulation',...
    'Callback',@(s,e) simulateDronePath(startPointEdit,endPointEdit,hAxes));
end

function generateMap(gridSizeEdit,obstacleDensityEdit,hAxes)
gridSize = clampInt(str2double(get(gridSizeEdit,'String')),2,512);
p = str2double(get(obstacleDensityEdit,'String'));
if ~isfinite(p) || p<0 || p>1, errordlg('Obstacle density must be in [0,1].'); return; end
map = rand(gridSize)<p; % 1=obstacle, 0=free
cla(hAxes);
imshow(1-map,'Parent',hAxes);
colormap(hAxes,gray); axis(hAxes,'equal'); axis(hAxes,'on'); hold(hAxes,'on');
setappdata(hAxes,'map',map);
setappdata(hAxes,'paths',{});
setappdata(hAxes,'startPoint',[]);
setappdata(hAxes,'endPoint',[]);
dd = getappdata(hAxes,'pathDropdown');
if ishghandle(dd), set(dd,'String',{'Select path'},'Value',1); end
end

function plotStartEnd(startPointEdit,endPointEdit,hAxes)
map = needMap(hAxes); if isempty(map), return; end
g = size(map,1);
[okS,s] = parseXY(get(startPointEdit,'String'),g);
[okE,e] = parseXY(get(endPointEdit,'String'),g);
if ~okS || ~okE, return; end
if map(s(2),s(1))==1, errordlg('Start is an obstacle.'); return; end
if map(e(2),e(1))==1, errordlg('End is an obstacle.'); return; end
sM = plot(hAxes,s(1),s(2),'o','MarkerSize',10,'MarkerEdgeColor','blue','MarkerFaceColor','blue');
eM = plot(hAxes,e(1),e(2),'o','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor','red');
setappdata(hAxes,'startMarker',sM);
setappdata(hAxes,'endMarker',eM);
setappdata(hAxes,'startPoint',s);
setappdata(hAxes,'endPoint',e);
end


function findPath(startPointEdit,endPointEdit,hAxes)
map = needMap(hAxes); if isempty(map), return; end
g = size(map,1);
[okS,s] = parseXY(get(startPointEdit,'String'),g);
[okE,e] = parseXY(get(endPointEdit,'String'),g);
if ~okS || ~okE, return; end
if map(s(2),s(1))==1 || map(e(2),e(1))==1, errordlg('Points must be obstacle-free.'); return; end
[found,pathRC] = bfs4(map,[s(2) s(1)],[e(2) e(1)]);
if ~found, errordlg('No path found.'); return; end
pathXY = [pathRC(:,2) pathRC(:,1)];
plot(hAxes,pathXY(:,1),pathXY(:,2),'r-','LineWidth',2);
paths = getappdata(hAxes,'paths'); if isempty(paths), paths={}; end
paths{end+1} = pathXY; setappdata(hAxes,'paths',paths);
updateDropdown(hAxes,numel(paths));
end

function findAlternativePath(startPointEdit,endPointEdit,hAxes)
map = needMap(hAxes); if isempty(map), return; end
g = size(map,1);
s = getappdata(hAxes,'startPoint');
e = getappdata(hAxes,'endPoint');
if isempty(s) || isempty(e)
    [okS,s] = parseXY(get(startPointEdit,'String'),g);
    [okE,e] = parseXY(get(endPointEdit,'String'),g);
    if ~okS || ~okE, return; end
    if map(s(2),s(1))==1 || map(e(2),e(1))==1, errordlg('Points must be obstacle-free.'); return; end
end
for tries=1:5000
    cp=[randi([1 g]) randi([1 g])];
    if map(cp(2),cp(1))==0 && ~isequal(cp,s) && ~isequal(cp,e), break; end
    if tries==5000, errordlg('Failed to pick a free checkpoint.'); return; end
end
[f1,p1] = bfs4(map,[s(2) s(1)],[cp(2) cp(1)]);
[f2,p2] = bfs4(map,[cp(2) cp(1)],[e(2) e(1)]);
if ~(f1 && f2), errordlg('No through-path via checkpoint.'); return; end
altRC = [p1; p2(2:end,:)];
altXY = [altRC(:,2) altRC(:,1)];
plot(hAxes,altXY(:,1),altXY(:,2),'b--','LineWidth',2);
plot(hAxes,cp(1),cp(2),'go','MarkerSize',8,'MarkerFaceColor','g');
paths = getappdata(hAxes,'paths'); if isempty(paths), paths={}; end
paths{end+1} = altXY; setappdata(hAxes,'paths',paths);
writematrix(altXY,'alternativePath.csv');
updateDropdown(hAxes,numel(paths));
end


function highlightSelectedPath(hAxes)
map = needMap(hAxes); if isempty(map), return; end
g = size(map,1);
dd = getappdata(hAxes,'pathDropdown');
paths = getappdata(hAxes,'paths'); if isempty(paths), paths={}; end
idx = get(dd,'Value')-1;
cla(hAxes);
imshow(1-map,'Parent',hAxes); colormap(hAxes,gray); axis(hAxes,'on'); hold(hAxes,'on');
sM = getappdata(hAxes,'startMarker'); eM = getappdata(hAxes,'endMarker');
if ishghandle(sM), plot(hAxes,sM.XData,sM.YData,'bo','MarkerFaceColor','b'); end
if ishghandle(eM), plot(hAxes,eM.XData,eM.YData,'ro','MarkerFaceColor','r'); end
if idx>=1 && idx<=numel(paths)
    sel = paths{idx};
    for i=1:numel(paths)
        if i==idx, continue; end
        p = paths{i}; plot(hAxes,p(:,1),p(:,2),'w-','LineWidth',0.5);
    end
    plot(hAxes,sel(:,1),sel(:,2),'k-','LineWidth',5);
    plot(hAxes,sel(:,1),sel(:,2),'w-','LineWidth',2);
end
axis(hAxes,[0.5 g+0.5 0.5 g+0.5]);
end

function simulateDronePath(startPointEdit,endPointEdit,hAxes)
map = needMap(hAxes); if isempty(map), return; end
g = size(map,1);
dd = getappdata(hAxes,'pathDropdown');
paths = getappdata(hAxes,'paths'); if isempty(paths), errordlg('No paths yet.'); return; end
idx = get(dd,'Value')-1;
if idx<1 || idx>numel(paths), errordlg('Select a valid path.'); return; end
sel = paths{idx};
cla(hAxes);
imshow(1-map,'Parent',hAxes); colormap(hAxes,gray); axis(hAxes,'equal'); axis(hAxes,'on'); hold(hAxes,'on');
plot(hAxes,sel(:,1),sel(:,2),'k-','LineWidth',5);
plot(hAxes,sel(:,1),sel(:,2),'w-','LineWidth',2);
drone = plot(hAxes,sel(1,1),sel(1,2),'bo','MarkerFaceColor','b');
for i=2:size(sel,1)
    set(drone,'XData',sel(i,1),'YData',sel(i,2));
    pause(0.08);
end
end

function [found,path] = bfs4(map,startRC,goalRC)
n = size(map,1); found = false; path = [];
visited = false(n,n);
parent = zeros(n,n,2,'uint16');
q = zeros(n*n,2,'uint16'); head=1; tail=1;
q(tail,:) = uint16(startRC); tail=tail+1;
visited(startRC(1),startRC(2)) = true;
nbr = int16([-1 0; 1 0; 0 -1; 0 1]);
while head<tail
    u = double(q(head,:)); head=head+1;
    if isequal(u,goalRC)
        found=true;
        path=reconstruct_path(parent,startRC,goalRC);
        return;
    end
    for k=1:4
        v = u+double(nbr(k,:)); r=v(1); c=v(2);
        if r<1 || r>n || c<1 || c>n, continue; end
        if visited(r,c) || map(r,c)==1, continue; end
        visited(r,c)=true;
        parent(r,c,1)=uint16(u(1)); parent(r,c,2)=uint16(u(2));
        q(tail,:)=uint16([r c]); tail=tail+1;
    end
end
end

function p = reconstruct_path(parent,startRC,goalRC)
cur=goalRC; p=cur;
while any(cur~=startRC)
    pr=parent(cur(1),cur(2),:); pr=double([pr(1) pr(2)]);
    if all(pr==0), p=[]; return; end
    p=[pr; p]; cur=pr;
end
end

function [ok,xy] = parseXY(str,gridSize)
ok=false; xy=[];
s=char(str); parts=regexp(s,'\s*,\s*','split');
if numel(parts)~=2, errordlg('Use "x,y"'); return; end
x=round(str2double(parts{1})); y=round(str2double(parts{2}));
if ~isfinite(x) || ~isfinite(y), errordlg('Numbers required'); return; end
if x<1 || x>gridSize || y<1 || y>gridSize, errordlg(sprintf('[1..%d]',gridSize)); return; end
ok=true; xy=[x y];
end

function g = clampInt(x,lo,hi)
if ~isfinite(x), x=lo; end
x=round(x); g=max(lo,min(hi,x));
end

function map = needMap(hAxes)
if ~isappdata(hAxes,'map'), errordlg('Generate a map first.'); map=[]; return; end
map=getappdata(hAxes,'map');
end

function updateDropdown(hAxes,nPaths)
dd=getappdata(hAxes,'pathDropdown'); if ~ishghandle(dd), return; end
labels=arrayfun(@(i) sprintf('Path %d',i),1:nPaths,'UniformOutput',false);
set(dd,'String',[{'Select path'} labels],'Value',max(1,min(nPaths+1,get(dd,'Value'))));
end

function mask = buildPathMask(map, paths, g)
mask = map; % keep obstacles
for i = 1:numel(paths)
    p = paths{i};
    idx = sub2ind([g g], p(:,2), p(:,1)); % (row,col) = (y,x)
    mask(idx) = 0; % carve paths open so they remain visible
end
end
