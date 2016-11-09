rosshutdown
rosinit('192.168.11.62')
rostopic list
tbot = turtlebot('192.168.11.62');

laser = rossubscriber('/scan');
%scan = receive(laser);
%plot(scan);
%data = readCartesian(scan);
%x = data(:,1);
%y = data(:,2);
%figure
%plot(y,x,':');

%ranges = double(scan.Ranges);
%angles = double(scan.readScanAngles);
%figure
%plot(angles,ranges,':')

%%

while true
x = [];
y = [];
ydis = [];
%figure(1)
%scatter(x,y)

scan = receive(laser);    
ranges = double(scan.Ranges);
angles = double(scan.readScanAngles);   
    
x = sin(-angles(:,1)).*ranges(:,1);
y = cos(angles(:,1)).*ranges(:,1);
x(any(isnan(x),2),:) = [];
y(any(isnan(y),2),:) = [];
figure(1)
scatter(x,y)
hold on
for i = 1:find(x ==x(end))-1
ydis(i,1) = y(i+1) - y(i);
end
unknown = ydis(abs(ydis)>0.5);
findmat = find(abs(ydis)>0.5);
odometry = getOdometry(tbot);
detect = [(x(findmat+1)+x(findmat))/2 (y(findmat+1)+y(findmat))/2];
scatter(detect(:,1),detect(:,2),'r','p')

globalDetect = [odometry.Position(1)+(x(findmat+1)+x(findmat))/2 odometry.Position(2)+(y(findmat+1)+y(findmat))/2];
hold off
end

%%
%{
x = data(:,1);
y = data(:,2);

dist = sqrt(x.^2 + y.^2);
minDist = min(dist);
%}