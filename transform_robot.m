%%
%turtlebot recongnition
rosshutdown
rosinit('192.168.11.62')
%rostopic list
tbot = turtlebot('192.168.11.62');
[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');
   odom = getOdometry(tbot);
   
   i = 1;
%    getposition = [1 1]
%    targetposition = [3 5]
%    get2tar = targetposition - getposition;
   preori = odom.Orientation(1);
   preodo = [odom.Position(1) odom.Position(2) odom.Position(3)];
while true
    odom = getOdometry(tbot);
    thita = odom.Orientation(1)*(180/pi);
    nowori = odom.Orientation(1) - preori;
    nowodo = odom.Position - preodo;
    figure(1)
    scatter(cos(nowori),sin(nowori));
    hold on
    figure(2)
    scatter(nowodo(2),nowodo(1));
    hold on
    
     Rt = [cos(odom.Orientation(1)) -sin(odom.Orientation(1));
         sin(odom.Orientation(1)) cos(odom.Orientation(1))]
   
%     Rt = [cos(nowori) -sin(nowori);
%        sin(nowori) cos(nowori)]
%     
       % Rt = [0 0;
       % sin(odom.Orientation(1)) cos(odom.Orientation(1))]
    
%     trans = Rt * [odom.Position(2); odom.Position(1)];
%     yVariation = odom.Position(1) - trans(2);
%     figure(3)
%     scatter(trans(1),trans(2)+yVariation)
    
    trans = Rt * [nowodo(2); nowodo(1)];
    yVariation = nowodo(1) - trans(2);
    %yVariation = 0;

    figure(3)
    scatter(trans(1),trans(2)+yVariation)
   % figure(4)
   % scatter(trans(1),yVariation)
    hold on
    %velMsg.Linear.X = 0;
    %velMsg.Angular.Z = 0.2;
    %velPub.send(velMsg);
    i = i + 1;
   end

    