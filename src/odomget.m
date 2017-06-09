clc
clear all
close all
%%
rosshutdown
rosinit('ros1-HP-ProBook-450-G1')
tbot = turtlebot('ros1-HP-ProBook-450-G1');

%% VFH
m = 1;
l = 1;
p=1;
q=1;
tfx(1,1)=0
tfy(1,1)=0
tfr(1,1)=0

while true
   
    if mod(m,3) == 0 || m == 1  
        odom = getOdometry(tbot);
        tf = rossubscriber('/tf');
        tfdata = receive(tf,3);  
        name = char(tfdata.Transforms(1).ChildFrameId(1))

        
        if name == 'o' && (abs(tfdata.Transforms.Transform.Translation.X) > 0 || abs(tfdata.Transforms.Transform.Translation.Y) > 0 || abs(tfdata.Transforms.Transform.Translation.Z) > 0);
            q=q+1;
            tfx(q,1)=-tfdata.Transforms.Transform.Translation.Y
            tfy(q,1)=tfdata.Transforms.Transform.Translation.X   
            tfr(q,1)=tfdata.Transforms.Transform.Rotation.Z
            %figure(1)
    
        end     
        odomx(p,1) = -odom.Position(2);
        odomy(p,1) = odom.Position(1);    
        %Rt = [cos(tfr(q,1)) -sin(tfr(q,1));
        %      sin(tfr(q,1)) cos(tfr(q,1))];  
        codomx(p,1) = -odom.Position(2)+tfx(q,1);
        codomy(p,1) = odom.Position(1)+tfy(q,1);
        ccodomx(p,1) = cos(tfr(q,1))*codomx(p,1)-sin(tfr(q,1))*codomy(p,1);
        ccodomy(p,1) = sin(tfr(q,1))*codomx(p,1)+cos(tfr(q,1))*codomy(p,1);
        %ccodomx(p,1) = cos(tfr(q,1))*codomx(p,1)+sin(tfr(q,1))*codomy(p,1);
        %ccodomy(p,1) = -sin(tfr(q,1))*codomx(p,1)+cos(tfr(q,1))*codomy(p,1);
        %figure(1)
        %scatter(codomx(p,1),codomy(p,1))
        %hold on
        figure(1)
        scatter(ccodomx(p,1),ccodomy(p,1))
        hold on
        odom = getOdometry(tbot);
        %accumx(p,1) = -odom.Position(2);
        %accumy(p,1) = odom.Position(1);
        %figure(1);
        %scatter(accumx(p,1),accumy(p,1))
        %hold on
        p = p + 1;
    end
    m = m +1;  
    odom = getOdometry(tbot);
           
end
       
     