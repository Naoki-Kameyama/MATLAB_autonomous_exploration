close all
clear all

rosshutdown
rosinit('ros1-HP-ProBook-450-G1')
tbot = turtlebot('ros1-HP-ProBook-450-G1');

%laser scan
laserSub = rossubscriber('/scan');

%velocity
[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');

%VFH parameter
%   vfh = robotics.VectorFieldHistogram;%
%   vfh.DistanceLimits = [0.05 0.65];%
%   vfh.RobotRadius = 0.3;%
%   vfh.MinTurningRadius = 0.1;%
%   vfh.SafetyDistance = 0.15;%c
%   targetDir = 0;%

%  vfh = robotics.VectorFieldHistogram;%
%  vfh.DistanceLimits = [0.05 0.5];%
%  vfh.RobotRadius = 0.2;%
%  vfh.MinTurningRadius = 0.25;%
%  vfh.SafetyDistance = 0.15;%
%  targetDir = 0;%


%  vfh = robotics.VectorFieldHistogram;%
%  vfh.DistanceLimits = [0.05 0.7];%
%  vfh.RobotRadius = 0.20;%
%  vfh.MinTurningRadius = 0.15;%
%  vfh.SafetyDistance = 0.1;%c
%  targetDir = 0;%


vfh = robotics.VectorFieldHistogram;%
vfh.DistanceLimits = [0.05 1];%
vfh.RobotRadius = 0.1;%
vfh.MinTurningRadius = 0.05;%
vfh.SafetyDistance = 0.25;%c
targetDir = 0;%


% vfh = robotics.VectorFieldHistogram;
% vfh.DistanceLimits = [0.05 1];
% vfh.RobotRadius = 0.1;
% vfh.MinTurningRadius = 0.2;
% vfh.SafetyDistance = 0.1;


%% VFH
m = 1;
p=1;
q=1;
tfx(1,1)=0
tfy(1,1)=0
tfr(1,1)=0

while true
    %odom = getOdometry(tbot);
    % Get laser scan data
    laserScan = receive(laserSub);
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
    
    if mod(m,3) == 0 || m == 1  
        odom = getOdometry(tbot);
        tf = rossubscriber('/tf');
        tfdata = receive(tf,3);  
        name = char(tfdata.Transforms(1).ChildFrameId(1))
        velPub.send(velMsg);  
        
        if name == 'o' && (abs(tfdata.Transforms.Transform.Translation.X) > 0 || abs(tfdata.Transforms.Transform.Translation.Y) > 0 || abs(tfdata.Transforms.Transform.Translation.Z) > 0);
            q=q+1;
            tfx(q,1)=-tfdata.Transforms.Transform.Translation.Y
            tfy(q,1)=tfdata.Transforms.Transform.Translation.X   
            tfr(q,1)=tfdata.Transforms.Transform.Rotation.Z
            %figure(1)
            %scatter(tfx(q,1),tfy(q,1),'r','p')
            %hold on
            %figure(2)
            %scatter(odomx(p,1),odomy(p,1))
            %hold on 
            velPub.send(velMsg);
        end     
        odomx(p,1) = -odom.Position(2);
        odomy(p,1) = odom.Position(1);    
        %Rt = [cos(tfr(q,1)) -sin(tfr(q,1));
        %      sin(tfr(q,1)) cos(tfr(q,1))];  
        codomx(p,1) = -odom.Position(2)+tfx(q,1);
        codomy(p,1) = odom.Position(1)+tfy(q,1);
        %ccodomx(p,1) = cos(tfr(q,1))*codomx(p,1)-sin(tfr(q,1))*codomy(p,1);
        %ccodomy(p,1) = sin(tfr(q,1))*codomx(p,1)+cos(tfr(q,1))*codomy(p,1);
        ccodomx(p,1) = cos(tfr(q,1))*codomx(p,1)+sin(tfr(q,1))*codomy(p,1);
        ccodomy(p,1) = -sin(tfr(q,1))*codomx(p,1)+cos(tfr(q,1))*codomy(p,1);
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
    Rt = [cos(odom.Orientation(1)) -sin(odom.Orientation(1));
          sin(odom.Orientation(1)) cos(odom.Orientation(1))];
    xx = cos(odom.Orientation(1)).*x + (-sin(odom.Orientation(1))).*y;
    yy = sin(odom.Orientation(1)).*x + cos(odom.Orientation(1)).*y;
    xd = sin(-angles1(:,1)).*ranges1(:,1);
    yd = cos(angles1(:,1)).*ranges1(:,1);
    x(any(isnan(x),2),:) = [];
    y(any(isnan(y),2),:) = [];
    figure(2)
    scatter(x,y)
    hold on
    if isempty(x) == 0 && isempty(y) == 0
        for i = 1:find(x ==x(end))-1
            ydis(i,1) = y(i+1) - y(i);
        end
        % nu
        unknown = ydis(abs(ydis)>1.5);
        findmat = find(abs(ydis)>1.5);
        odom = getOdometry(tbot);
        detect = [(x(findmat+1)+x(findmat))/2 (y(findmat+1)+y(findmat))/2];
        figure(2)
        scatter(detect(:,1),detect(:,2),'r','p')
        hold off
        targetmat = find(detect(:,2) == min(detect(:,2)));
        detectxx = cos(odom.Orientation(1)).*detect(:,1) + (-sin(odom.Orientation(1))).*detect(:,2);
        detectyy = sin(odom.Orientation(1)).*detect(:,1) + cos(odom.Orientation(1)).*detect(:,2);
        %detectxx = sin(odom.Orientation(1)).*detect(:,1) + cos(odom.Orientation(1)).*detect(:,2);
        %detectyy = cos(odom.Orientation(1)).*detect(:,1) + (-sin(odom.Orientation(1))).*detect(:,2);
        %detectxx = cos(-(odom.Orientation(1))).*detect(:,1) + (-sin(-(odom.Orientation(1)))).*detect(:,2);
        %detectyy = sin(-(odom.Orientation(1))).*detect(:,1) + cos(-(odom.Orientation(1))).*detect(:,2);
        %transpoint = [-odom.Position(2)+detectxx(targetmat,1) odom.Position(1)+detectyy(targetmat,1)];
        transpoint = [ccodomx(p-1,1)+detectxx(targetmat,1) ccodomy(p-1,1)+detectyy(targetmat,1)];
        %figure(1)
        %scatter(transpoint(:,1),transpoint(:,2),'r','p')
        %hold on
        %figure(6)
        %scatter(transpoint(:,1),transpoint(:,2),'r','p')
        %hold on
        a = x(findmat+1);
        b = x(findmat);
        xa = a(find(detect(:,2) == min(detect(:,2))));
        xb = b(find(detect(:,2) == min(detect(:,2))));
        xab = abs(xa - xb);
            if (isnan(yd(319)) == 1 && isnan(yd(321)) == 1 && isempty(detect) == 0) || (yd(319) > 3 && isempty(detect) == 0) || (yd(321) > 3 && isempty(detect) == 0);
                target = [detect(targetmat,1) detect(targetmat,2)];
                if detect(targetmat,2) > 6.5
                    target = [];
                end
                %kyoyou = min(abs(sqrt((transpoint(1) - accumx(:,1)).^2 + (transpoint(2) - accumy(:,1)).^2)));
                kyoyou = min(abs(sqrt((transpoint(1) - ccodomx(:,1)).^2 + (transpoint(2) - ccodomy(:,1)).^2)));
                if kyoyou < 1.5
                    target = [];
                end
            end
        %if isnan(ranges1(319)) == 0 || isnan(ranges1(321)) == 0 || ranges1(319) < 1 || ranges1(321) < 1 || isempty(xab)==1 || xab > 1.5 ;
        if (isnan(yd(319)) == 0 && yd(319) < 1) || (isnan(yd(319)) == 0 && yd(321) < 1) || isempty(xab)==1 || xab > 0.8 ;
            %targetmat = find(detect(:,2)==min(detect(:,2)));
            target = [];
        end
    end
%% forword to target
    if isempty(target) == 0
        figure(1)
        scatter(transpoint(1),transpoint(2),'r','p')
        hold on
        figure(6)
        scatter(transpoint(1),transpoint(2),'r','p')
        hold on
        l = 1;
        preodo = [odom.Position(1) odom.Position(2) odom.Position(3)];
        preori = odom.Orientation(1);
        velPub.send(velMsg);
        while true
            if mod(m,3) == 0 || m == 1
                odom = getOdometry(tbot);
                tf = rossubscriber('/tf');
                tfdata = receive(tf,3);  
                name = char(tfdata.Transforms(1).ChildFrameId(1))
                velPub.send(velMsg);    
                if name == 'o' && (abs(tfdata.Transforms.Transform.Translation.X) > 0 || abs(tfdata.Transforms.Transform.Translation.Y) > 0 || abs(tfdata.Transforms.Transform.Translation.Z) > 0);
                    q=q+1;
                    tfx(q,1)=-tfdata.Transforms.Transform.Translation.Y
                    tfy(q,1)=tfdata.Transforms.Transform.Translation.X   
                    tfr(q,1)=tfdata.Transforms.Transform.Rotation.Z
                    %figure(1)
                    %scatter(tfx(q,1),tfy(q,1),'r','p')
                    %hold on
                    %figure(2)
                    %scatter(odomx(p,1),odomy(p,1))
                    %hold on 
                    velPub.send(velMsg);
                end     
                odomx(p,1) = -odom.Position(2);
                odomy(p,1) = odom.Position(1);    
                %Rt = [cos(tfr(q,1)) -sin(tfr(q,1));
                      %sin(tfr(q,1)) cos(tfr(q,1))];  
                codomx(p,1) = -odom.Position(2)+tfx(q,1);
                codomy(p,1) = odom.Position(1)+tfy(q,1);
                %ccodomx(p,1) = cos(tfr(q,1))*codomx(p,1)-sin(tfr(q,1))*codomy(p,1);
                %ccodomy(p,1) = sin(tfr(q,1))*codomx(p,1)+cos(tfr(q,1))*codomy(p,1);
                ccodomx(p,1) = cos(tfr(q,1))*codomx(p,1)+sin(tfr(q,1))*codomy(p,1);
                ccodomy(p,1) = -sin(tfr(q,1))*codomx(p,1)+cos(tfr(q,1))*codomy(p,1);
                %figure(1)
                %scatter(codomx(p,1),codomy(p,1))
                %hold on
                figure(1)
                scatter(ccodomx(p,1),ccodomy(p,1))
                hold on
                %accumx(p,1) = -odom.Position(2) ;
                %accumy(p,1) = odom.Position(1);
                %figure(1)
                %scatter(accumx(p,1),accumy(p,1));
                %hold on  
                p = p + 1;
            end
            m = m +1; 
            %figure(3)
            %scatter(target(1),target(2),'r','p');
            %hold on
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
                    if  S > 350    %if laser scan data is received, break
                        break
                    end
                end
                pause(1.5)
            end    
           %% target dicision
            odom = getOdometry(tbot); 
            nowodo = odom.Position - preodo;
            nowori = preori - odom.Orientation(1);
            Rt = [cos(preori) -sin(preori);
                  sin(preori) cos(preori)];
            trans = Rt * [nowodo(2); nowodo(1)];
            %figure(3)
            %scatter(trans(1),trans(2));
            %hold on
            dist = sqrt((odom.Position(2)-preodo(2))^2 + (odom.Position(1)-preodo(1))^2);
                if (abs(dist) > target(2)) && (abs(trans(2)) > target(2));
                %if abs(trans(2)) > target(2)-0.1 && abs(trans(2)) < target(2)+0.5;
                    break
                end
                l = l + 1;
        end
        tic
        while toc < 1.6
            detectxx = cos(odom.Orientation(1)).*detect(:,1) + (-sin(odom.Orientation(1))).*detect(:,2);
            targett = target(1)*cos(nowori) + (-sin(nowori))*target(2);
            %if target(1) > 0
            if targett > 0
                velMsg.Linear.X = 0;
                velMsg.Angular.Z = -1;
                velPub.send(velMsg);
                odom = getOdometry(tbot);
            end 
            %if target(1) < 0
            if targett < 0
                velMsg.Linear.X = 0;
                velMsg.Angular.Z = 1;
                velPub.send(velMsg);
                odom = getOdometry(tbot);
            end
        end
        %回転コマンド書き直し。。
        %もし目の前に壁があったら逆回転せよ！！！！！！！！！！！！！！！！！！！！！！！！！！！！
    end
    %% exploration
    if isempty(target) == 1
        odom = getOdometry(tbot); 
    end
end
