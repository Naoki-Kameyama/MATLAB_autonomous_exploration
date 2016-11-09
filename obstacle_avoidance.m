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
%VFH param
vfh = robotics.VectorFieldHistogram;%
vfh.DistanceLimits = [0.05 1];%
vfh.RobotRadius = 0.16;%
vfh.MinTurningRadius = 0.2;%
vfh.SafetyDistance = 0.1;%
targetDir = 0;%


%%
i = 1;
%n = 1;
while true
    %tic
    % Get odometry data
    odom = getOdometry(tbot);
    odomList(i,:) = [odom.Position(1) odom.Position(2)];
    figure(1)
    % subplot(1,2,1)
     plot(odomList(:,1),odomList(:,2))
    i=i+1;
    
    % Get laser scan data
    laserScan = receive(laserSub);
    % xlim auto
    %  subplot(1,2,2)
    % plot(laserScan);
    
    ranges = double(laserScan.Ranges);
    angles = double(laserScan.readScanAngles);
    laserScan.Ranges(isnan(laserScan.Ranges(:,1))) = 0;
    S = sum(laserScan.Ranges(:,1));%laser scan data quantity
    
    % Call VFH step method to computer steering direction
    steerDir = vfh.step(ranges, angles, targetDir);
    
    % Calculate velocities
    if ~isnan(steerDir) % もしsteerDirが1だったら(もしsteerDirが1だったらNaNじゃなかたら)
        
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
    % figure(2)
    %show(vfh);
    
    
    
  %  t(n,1) = toc;
   % count(n,1) = n;
    %plot(count(:,1),t(:,1));
    %n = n + 1;
    
    
    
end

