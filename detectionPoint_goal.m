%%
%turtlebot recongnition
rosshutdown
rosinit('IP')
rostopic list
tbot = turtlebot('IP');
[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');
odom = getOdometry(tbot);
laser = rossubscriber('/scan');
%%
i = 1;
while true
    odom = getOdometry(tbot);
    odomList(i,:) = [odom.Position(1) odom.Position(2)];
%plot(odomList(:,1),odomList(:,2))

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
unknown = ydis(abs(ydis)>0.4);
findmat = find(abs(ydis)>0.4);
odom = getOdometry(tbot);
detect = [(x(findmat+1)+x(findmat))/2 (y(findmat+1)+y(findmat))/2];
scatter(detect(:,1),detect(:,2),'r','p')

globalDetect = [odom.Position(1)+(x(findmat+1)+x(findmat))/2 odom.Position(2)+(y(findmat+1)+y(findmat))/2];
hold off

%targetmat = find(min(globalDetect(:,2)));
targetmat = find(globalDetect(:,2)==min(globalDetect(:,2)));
target = [globalDetect(targetmat,1) globalDetect(targetmat,2)];


%%

if ~isnan(target(2))
%globalunknown = [odomList(i,1)+unknown(1) odomList(i,2)+unknown(2)];
l = 1;
while true
    figure(3)
    scatter(target(1),target(2));

    velMsg.Linear.X = 0.2;
    velMsg.Angular.Z = 0;
    velPub.send(velMsg);
      
    odom = getOdometry(tbot); 
    odomList2(l,:) = [odom.Position(1) odom.Position(2)];
    figure(4)
    plot(odomList2(:,1),odomList2(:,2))
    if odom.Position(1) > target(2)-0.05 && odom.Position(1) < target(2)+0.05;
        break
    end
    l = l + 1;
end
end


tic;
m = 1;
while toc < 1.3
velMsg.Linear.X = 0.1;
velMsg.Angular.Z = 1;
velPub.send(velMsg);
odom = getOdometry(tbot); 
odomList3(m,:) = [odom.Position(1) odom.Position(2)];
figure(4)
    plot(odomList3(:,1),odomList3(:,2))
m = m + 1;
end

pause(1)
i = i + 1;
end
