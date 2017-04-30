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
vfh.RobotRadius = 0.16;%%%
%turtlebot recongnition
rosshutdown
rosinit('192.168.11.62')
rostopic list
tbot = turtlebot('192.168.11.62');

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
n = 1;
%t = zeros(1,100);
while true
    tic
    % Get odometry data
    odom = getOdometry(tbot);
    odomList(i,:) = [odom.Position(1) odom.Position(2)];
    %figure(1)
   % subplot(1,2,1)
   % plot(odomList(:,1),odomList(:,2))
    i=i+1;
    
	% Get laser scan data
	laserScan = receive(laserSub);
    %xlim auto
    %subplot(1,2,2)        
    %plot(laserScan);
    
    	ranges = double(laserScan.Ranges);
	angles = double(laserScan.readScanAngles);
	laserScan.Ranges(isnan(laserScan.Ranges(:,1))) = 0;
    S = sum(laserScan.Ranges(:,1));%laser scan data quantity 
    
    
    
    

    pcloud = getPointCloud(tbot);
    ptCloud = pointCloud(pcloud.XYZ);
    
    %gridStep = 0.01;
    %ptCloudA = pcdownsample(ptCloud,'gridAverage',gridStep);        
    
    maxNumPoints = 50;
    ptCloudA = pcdownsample(ptCloud,'nonuniformGridSample',maxNumPoints) ;
    pointtemp1 = ptCloudA.Location(:,1);
    pointtemp3 = ptCloudA.Location(:,3);
    
    for i = 1:ptCloudA.Count
    if ptCloudA.Location(i,2) > 0
    pointtemp3(i,1) = NaN;                %<- I don't know why this is not work.... 
    end
    end
    
    %R =  sqrt(ptCloudA.Location(:,1).^2+ptCloudA.Locat3on(:,3).^2);
    R =  sqrt(pointtemp1(:,1).^2 + pointtemp3(:,1).^2);
    A =  acos(pointtemp1(:,1)./R) - 1.6 ;
    A(isnan(A)) = 0;
    
    

    
	% Call VFH step method to computer steering direction
	%steerDir = vfh.step(ranges, angles, targetDir); 
       	steerDir = vfh.step(R, A, targetDir);     
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
            
            if  S > 500    %if laser scan data is received, break
               break  
            end
            
            end
            
pause(1.5)

    end
    
   % figure(2)
    %show(vfh);

%t(n,1) = toc;
%point(n,1) = ptCloudA.Count
%count(n,1) = n;
%plot(count(:,1),t(:,1));
%n = n + 1;

end
