clear all
rosshutdown
rosinit('ros1-HP-ProBook-450-G1')
tbot = turtlebot('ros1-HP-ProBook-450-G1');

p=1;
while true
    odom = getOdometry(tbot);
    tf = rossubscriber('/tf');
    tfdata = receive(tf,3);  
    name = char(tfdata.Transforms(1).ChildFrameId(1))
    if name == 'o'
        tfx(p,1)=tfdata.Transforms.Transform.Translation.X
        tfy(p,1)=tfdata.Transforms.Transform.Translation.Y   
        tfr(p,1)=tfdata.Transforms.Transform.Rotation.Z
        figure(1)
        scatter(tfx(p,1),tfy(p,1))
        hold on
    end  
    odomx(p,1) = -odom.Position(2);
    odomy(p,1) = odom.Position(1);
    
    %figure(1)
    %scatter(odomx(p,1),odomy(p,1),'r','p')
    %hold on
    
    p=p+1;
end



