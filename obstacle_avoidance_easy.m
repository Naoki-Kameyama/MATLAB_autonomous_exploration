rosshutdown
rosinit('IP')
rostopic list
tbot = turtlebot('IP');


 spinVelocity = 0.8;       % Angular velocity (rad/s)
  forwardVelocity = 0.1;    % Linear velocity (m/s)
  backwardVelocity = -0.02; % Linear velocity (reverse) (m/s)
  distanceThreshold = 0.55;  % Distance threshold (m) for turning


  robot = rospublisher('/mobile_base/commands/velocity');
  velmsg = rosmessage(robot);
  laser = rossubscriber('/scan');
  wheel = rossubscriber('/odom');
 
i=1;
  while true
    odom = getOdometry(tbot);
    odomList(i,:) = [odom.Position(1) odom.Position(2)];
    subplot(1,2,1)
    plot(odomList(:,1),odomList(:,2))
    i=i+1;
      
      

      % Collect information from laser scan
      scan = receive(laser);
      subplot(1,2,2)
      plot(scan);
      data = readCartesian(scan);
      x = data(:,1);
      y = data(:,2);
      % Compute distance of the closest obstacle
      dist = sqrt(x.^2 + y.^2);
      minDist = min(dist);
      % Command robot action
      if minDist < distanceThreshold
          % If close to obstacle, back up slightly and spin
          velmsg.Angular.Z = spinVelocity;
          velmsg.Linear.X = backwardVelocity;
      else
          % Continue on forward path
          velmsg.Linear.X = forwardVelocity;
          velmsg.Angular.Z = 0;
      end
      send(robot,velmsg);
  end
