%%
%turtlebot recongnition
rosshutdown
rosinit('IP')
rostopic list
tbot = turtlebot('IP');
%%
%laser scan
laserSub = rossubscriber('/scan');
%%
%velocity
[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');
%%
%VFH parameter
vfh = robotics.VectorFieldHistogram;%
vfh.DistanceLimits = [0.05 1];%
vfh.RobotRadius = 0.16;%
vfh.MinTurningRadius = 0.2;%
vfh.SafetyDistance = 0.1;%
targetDir = 0;%
%% VFH
while true
    odom = getOdometry(tbot);
    % Get laser scan data
    laserScan = receive(laserSub);
    ranges = double(laserScan.Ranges);
    angles = double(laserScan.readScanAngles);
    laserScan.Ranges(isnan(laserScan.Ranges(:,1))) = 0;
    S = sum(laserScan.Ranges(:,1));%laser scan data quantity
    % Call VFH step method to computer steering direction
    steerDir = vfh.step(ranges, angles, targetDir);
    % Calculate velocities
    if ~isnan(steerDir) % ‚à‚µsteerDir‚ª1‚¾‚Á‚½‚ç(‚à‚µsteerDir‚ª1‚¾‚Á‚½‚çNaN‚¶‚á‚È‚©‚½‚ç)
        desiredV = 0.2;
        w = exampleHelperComputeAngularVelocity(steerDir, 1);%
        w1= exampleHelperComputeAngularVelocity(1, 1);%;
        Zw = velMsg.Angular.Z;
    else % Stop and search for valid direction
        desiredV = 0.0;
        w = 0.5;
    end
    % Assign and send velocity commands
    velMsg.Linear.X = desiredV;
    velMsg.Angular.Z = w;
    velPub.send(velMsg);
    % Don't received laser data
    if  S == 0
        desiredV = -0.2;
        velMsg.Linear.X = desiredV;
        velMsg.Angular.Z = w;
        velPub.send(velMsg);
        pause(1.5)
        % lotation  desired direction
        while toc < 100        
            if Zw > 0
                w1 = -1;
            elseif Zw < 0
                w1 = 1;
            end
            if Zw == 0
                w1 = 1;
            end
            velMsg.Linear.X = 0;
            velMsg.Angular.Z = w1;
            velPub.send(velMsg);
            laserScan = receive(laserSub);
            laserScan.Ranges(isnan(laserScan.Ranges(:,1))) = 0;
            S = sum(laserScan.Ranges(:,1));
            if  S > 300    %if laser scan data is received, break
                break
            end
        end
        pause(1.5)
    end
    
%% target detection decision
odom = getOdometry(tbot);
laserScan = receive(laserSub);  
    x = [];
    y = [];
    ydis = [];
ranges1 = double(laserScan.Ranges);
angles1 = double(laserScan.readScanAngles);   
    
x = sin(-angles1(:,1)).*ranges1(:,1);
y = cos(angles1(:,1)).*ranges1(:,1);
x(any(isnan(x),2),:) = [];
y(any(isnan(y),2),:) = [];
figure(1)
scatter(x,y)
hold on

for i = 1:find(x ==x(end))-1
ydis(i,1) = y(i+1) - y(i);
end

unknown = ydis(abs(ydis)>1.85);
findmat = find(abs(ydis)>1.85);
odom = getOdometry(tbot);
detect = [(x(findmat+1)+x(findmat))/2 (y(findmat+1)+y(findmat))/2];
figure(1)
scatter(detect(:,1),detect(:,2),'r','p')
hold off

targetmat = find(detect(:,2)==min(detect(:,2)));
target = [detect(targetmat,1) detect(targetmat,2)];

%% forword to target
if ~isempty(target) == 1
l = 1;
preodo = [odom.Position(1) odom.Position(2) odom.Position(3)];
preori = odom.Orientation(1);
while true
    figure(2)
    scatter(target(1),target(2),'r','p');
    hold on
    %scatter(odom.Position(2),odom.Position(1));
    %% VFH 
    laserScan = receive(laserSub);
    ranges = double(laserScan.Ranges);
    angles = double(laserScan.readScanAngles);
    laserScan.Ranges(isnan(laserScan.Ranges(:,1))) = 0;
    S = sum(laserScan.Ranges(:,1));%laser scan data quantity
    % Call VFH step method to computer steering direction
    steerDir = vfh.step(ranges, angles, targetDir);
    % Calculate velocities
    if ~isnan(steerDir) % ‚à‚µsteerDir‚ª1‚¾‚Á‚½‚ç(‚à‚µsteerDir‚ª1‚¾‚Á‚½‚çNaN‚¶‚á‚È‚©‚½‚ç)
        desiredV = 0.2;
        w = exampleHelperComputeAngularVelocity(steerDir, 1);%
        w1= exampleHelperComputeAngularVelocity(1, 1);%;
        Zw = velMsg.Angular.Z;
    else % Stop and search for valid direction
        desiredV = 0.0;
        w = 0.5;
    end
    % Assign and send velocity commands
    velMsg.Linear.X = desiredV;
    velMsg.Angular.Z = w;
    velPub.send(velMsg);
    
     % Don't received laser data
    if  S == 0
        desiredV = -0.2;
        velMsg.Linear.X = desiredV;
        velMsg.Angular.Z = w;
        velPub.send(velMsg);
        pause(1.5)
        % lotation  desired direction
        tic;
        while toc < 100
            if Zw > 0
                w1 = -1;
            elseif Zw < 0
                w1 = 1;
            end
            if Zw == 0
                w1 = 1;
            end
            velMsg.Linear.X = 0;
            velMsg.Angular.Z = w1;
            velPub.send(velMsg);
            
            laserScan = receive(laserSub);
            laserScan.Ranges(isnan(laserScan.Ranges(:,1))) = 0;
            S = sum(laserScan.Ranges(:,1));
            
            if  S > 300    %if laser scan data is received, break
                break
            end
        end
        pause(1.5)
    end    
    %% target dicision
    odom = getOdometry(tbot); 
    nowodo = odom.Position - preodo;
    Rt = [cos(preori) -sin(preori);
         sin(preori) cos(preori)];
    trans = Rt * [nowodo(2); nowodo(1)];
    
    figure(3)
    scatter(trans(1),trans(2));
    hold on
    
    dist = sqrt((odom.Position(2)-preodo(2))^2 + (odom.Position(1)-preodo(1))^2);
    if abs(dist) > target(2)-0.2 && abs(dist) < target(2)+0.2;
    %if abs(trans(2)) > target(2)-0.2 && abs(trans(2)) < target(2)+0.2;
    break
    end
    l = l + 1;
end

tic
while toc < 1.3
    if target(1) < 0
velMsg.Linear.X = 0.1;
velMsg.Angular.Z = 1;
velPub.send(velMsg);
odom = getOdometry(tbot);
    end
    
    if target(1) > 0
velMsg.Linear.X = 0.1;
velMsg.Angular.Z = -1;
velPub.send(velMsg);
odom = getOdometry(tbot);
    end  
end
end
%% exploration
if isempty(target) == 1
   odom = getOdometry(tbot); 
end

end