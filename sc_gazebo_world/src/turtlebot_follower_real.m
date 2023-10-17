
classdef turtlebot_follower
    properties
        RobotCmd;
        OdomSub;
        LidarSub;
        CameraRgbSub;
        DepthSub;
        PoseSub;

        MarkerImg;
        Intrinsics;
        MarkerSize = 0.09;
        % for SetGoalPose
        Distance = 0.7;
        % for DetermineCmdVelocity
        endPositionError = 0.5;
        endOrientationError = 20;
        % for SetRefPose
        changeInDistance = 0.01;
        changeInAngle = 10;

    end
    methods
        function obj = turtlebot_follower()
            rosinit()
            
            focalLength    = [554 554]; 
            principalPoint = [320 240];
            imageSize      = [480 640];
            obj.Intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize); % would be better to get from camera_info topic

            obj.MarkerImg = rgb2gray (imread('../meshes/0.png'));

            obj.RobotCmd = rospublisher("/cmd_vel","DataFormat","struct");
            obj.OdomSub = rossubscriber("/odom","DataFormat","struct");
            %obj.PoseSub = rossubscriber("/robot2/odom","DataFormat","struct");
            obj.LidarSub = rossubscriber("/scan","DataFormat","struct");
            obj.CameraRgbSub = rossubscriber("/camera/color/image_raw","DataFormat","struct");
            obj.DepthSub = rossubscriber("/camera/depth/image_raw","DataFormat","struct");
        end

        function FollowTheLeader(obj)
            tic;
            %r = rosrate(100);
            followLeader = true;
            timer = 5;
            readAprilTagTime = timer;
            markerPresent = 0;
            poseType = 0;
            refPose = 0;
            firstPose = 0;
            secondPose = 0;
            %reset(r);
            
            while followLeader
                % get values from robot
                % every ten seconds get new image
%                 if toc > readAprilTagTime
                rgbImgMsg = CameraRgbCallback(obj);
                depthMsg = CameraDepthCallback(obj);
                currentOdom = OdomCallback(obj);
                robotPose = currentOdom.Pose.Pose;

                [markerPresent, pose] = AnalyseImage(obj, rgbImgMsg, depthMsg, robotPose);

                readAprilTagTime = toc+timer;
                disp("Image Read");
%                 else
%                     % do nothing
%                 end

                if markerPresent && ~isnan(pose.Position.X)

                    
                    currentOdom = OdomCallback(obj);
                    robotPose = currentOdom.Pose.Pose;
%                     currentLeaderPose = PoseCallback(obj);
%                     leaderPose = currentLeaderPose.Pose.Pose;

                    % Move towards the marker
                    [refPose, poseType, firstPose, secondPose] = SetRefPose(obj, pose, poseType, refPose, firstPose, secondPose);
                    disp("refPose = ")
                    disp(refPose.Position)
                    disp(refPose.Orientation)
                    goalPose = DetermineGoalPose(obj, refPose, poseType);
                    disp("goalPose = ")
                    disp(goalPose.Position)
                    disp(goalPose.Orientation)
                    [cmdVel, poseType] = DetermineCmdVelocity(obj, refPose, poseType, goalPose, robotPose); 
                    PublishCmdVelocity(obj, cmdVel);
                else
                    velocities = [0,0,0,0,0,0];
                    PublishCmdVelocity(obj, velocities); % stand still if marker not present
                end

                % stop after 5 minutes
                if toc > 5*60
                   followLeader = false;
                   velocities = [0,0,0,0,0,0];
                   PublishCmdVelocity(obj, velocities);
                end
            end
        end

        function [refPose, poseType, firstPose, secondPose] = SetRefPose(obj, pose, poseType, refPose, firstPose, secondPose)
            % map the pose taken from image analysis whenever it moves 0.3m
            % from its last position OR whenever it turns
            
            
            % save the first pose for comparison
            if poseType==0
                firstPose = pose;
                refPose = pose;
                poseType = 1;
                disp("first pose")
            end

            if poseType==1
                
                % difference in angle between first and current pose
                % more than 5 degrees
                quatFirst = firstPose.Orientation;
                anglesFirst = quat2eul([quatFirst.W quatFirst.X quatFirst.Y quatFirst.Z]);
                thetaFirst = rad2deg(anglesFirst(1)); %use roll instead of yaw to determine yaw

                quatPose = pose.Orientation;
                anglesPose = quat2eul([quatPose.W quatPose.X quatPose.Y quatPose.Z]);
                thetaPose = rad2deg(anglesPose(1));

                if (firstPose.Position.X-pose.Position.X)^2+(firstPose.Position.Y-pose.Position.Y)^2 >= obj.changeInDistance
                    refPose = pose;
                    firstPose = firstPose;
                    secondPose = secondPose;
                    poseType = 3;
                    disp("translation")                

                elseif abs(thetaFirst-thetaPose)>obj.changeInAngle % in progress of turning
                    refPose = refPose;
                    firstPose = firstPose;
                    secondPose = pose;
                    poseType = 2;
                    disp("second pose")
                else
                    refPose = pose;
                    firstPose = firstPose;
                    secondPose = secondPose;
                end

            end

            if poseType==2
                % difference in angle between second and third pose
                % less than 2 degrees
                quatSecond = secondPose.Orientation;
                anglesSecond = quat2eul([quatSecond.W quatSecond.X quatSecond.Y quatSecond.Z]);
                thetaSecond = rad2deg(anglesSecond(1));

                thirdPose = pose;
                quatThird = thirdPose.Orientation;
                anglesThird = quat2eul([quatThird.W quatThird.X quatThird.Y quatThird.Z]);
                thetaThird = rad2deg(anglesThird(1));

                if abs(thetaThird-thetaSecond)<2
                    refPose = pose;
                    firstPose = firstPose;
                    secondPose = secondPose;
                    poseType = 4;
                    disp("rotation")
                else
                    refPose = refPose;
                    firstPose = firstPose;
                    secondPose = thirdPose;
                    disp("third pose")
                end
            end
        end

        function [cmdVel, poseType] = DetermineCmdVelocity(obj, pose, poseType, goalPose, currentPose)
            cmdVel = [0 0 0 0 0 0];
            poseType = poseType;

            % display goal transform & pose
            goalPoseTr = quat2rotm([goalPose.Orientation.W goalPose.Orientation.X goalPose.Orientation.Y goalPose.Orientation.Z]);
            goalPoseTr(4,:) = [0 0 0];
            goalPoseTr(:,4) = [goalPose.Position.X; goalPose.Position.Y; goalPose.Position.Z; 1];
            disp("Goal:");
            disp(goalPoseTr);

            currentPoseTr = quat2rotm([currentPose.Orientation.W currentPose.Orientation.X currentPose.Orientation.Y currentPose.Orientation.Z]);
            currentPoseTr(4,:) = [0 0 0];
            currentPoseTr(:,4) = [currentPose.Position.X; currentPose.Position.Y; currentPose.Position.Z; 1];
            disp("currentPose:");
            disp(currentPoseTr);

            quatGoal = goalPose.Orientation;
            angles = quat2eul([quatGoal.W quatGoal.X quatGoal.Y quatGoal.Z]);
            thetaGoal = rad2deg(angles(1));

            quatCurrent = currentPose.Orientation;
            angles = quat2eul([quatCurrent.W quatCurrent.X quatCurrent.Y quatCurrent.Z]);
            thetaCurrent = rad2deg(angles(1));

            xDiff = abs(goalPose.Position.X - currentPose.Position.X)
            yDiff = abs(goalPose.Position.Y - currentPose.Position.Y)
            angularError = rad2deg(atan2(yDiff,xDiff));
            direction1 = (angularError-thetaCurrent)/(abs(angularError-thetaCurrent));
            currentDistance = sqrt((pose.Position.X-currentPose.Position.X)^2+(pose.Position.Y-currentPose.Position.Y)^2)
            if currentDistance >= obj.Distance
                direction2 = 1
            else
                direction2 = -1;
            end
            direction3 = (thetaGoal-thetaCurrent)/(abs(thetaGoal-thetaCurrent));
            
            % do we need this angular error value ?? angular error the angle
            % towards the goalPose if its not directly in front of it
            % if goal is behind it dont spin all the way around
%             if (angularError/thetaCurrent)/(abs(angularError/thetaCurrent))<0 
%                 disp("ALERT")
%                 if angularError>0
%                     angularError = angularError-180;
%                 else
%                     angularError = angularError+180;
%                 end
%             end

            if isnan(currentDistance)
                cmdVel = [0 0 0 0 0 0];
                disp("Invalid data - Stop")              
            elseif abs(xDiff)<obj.endPositionError && abs(yDiff)<obj.endPositionError
                if abs(thetaGoal-thetaCurrent)<obj.endOrientationError
                    % at goal and facing correct direction
                    % do nothing
                    cmdVel = [0 0 0 0 0 0];
                    poseType=0;
                    disp("!!! At goal !!!")
                else
                    % at goal and not facing correct direction
                    % spin to correct direction
                    cmdVel = [0 0 0 0 0 direction3*0.05];
                    disp("Final spin")
                end
            else
                k = VelocityController(obj, currentDistance);
                disp(k)
                if abs(angularError-thetaCurrent)<obj.endOrientationError
                    % facing direction of goal but not there yet
                    % drive towards goal
                    cmdVel = [direction2*k*0.5 0 0 0 0 0];
                    disp("Drive to goal")
                else
                    % not facing direction of goal and not at goal
                    % turn to face goal
                    cmdVel = [0 0 0 0 0 direction1*0.05];
                    disp("Face goal")
                    disp("angularError = " + string(angularError))
                    disp("thetaCurrent = " + string(thetaCurrent))
                    disp("endOrientationError = " + string(obj.endOrientationError))
                end
            end
        end

        function k = VelocityController(obj, currentDistance)
            distanceToGoal = abs(currentDistance-obj.Distance);
            if distanceToGoal>0.7
                k = 0.9;
            elseif distanceToGoal>0.3
                k = 0.8;
            elseif distanceToGoal>0.2
                k = 0.3;
            else
                k = 0.1;
            end
        end

        function goalPose = DetermineGoalPose(obj, pose, poseType)
            % pose = pose from AR Tag
            % translate this pose to be about 0.5m away from leader turtlebot
            
            % find angle of AR tag
            quat = pose.Orientation;
            angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
            theta = angles(1);
            % get x,y distance away based on angle
            translate_x = -obj.Distance*cos(theta);
            translate_y = -obj.Distance*sin(theta);

            % convert from rigid3d to ros Pose
            goalPose = rosmessage("geometry_msgs/Pose","DataFormat","struct");
            if poseType==3 || poseType==1
                    goalPose.Position.X = pose.Position.X+translate_x;
                    goalPose.Position.Y = pose.Position.Y+translate_y;
                    goalPose.Orientation = pose.Orientation;
            elseif poseType==4
                    goalPose.Position.X = pose.Position.X;
                    goalPose.Position.Y = pose.Position.Y;
                    quat = eul2quat([theta 0 0]);
                    goalPose.Orientation = quat;
            end

                goalPose.Position.Z = pose.Position.Z;
    %             goalPose.Orientation = rotm2quat(pose(1:3,1:3));  % Add back
    %             in later when using AnalyseImage
            
            
        end

        function [markerPresent,worldPose] = AnalyseImage(obj, rgbImgMsg, depthMsg, robotPose)
            rgbImg = rosReadImage(rgbImgMsg);
            grayImage = rgb2gray(rgbImg);

            % adjust the image to allow for easier detection
            refinedImage = imadjust(grayImage);
%             refinedImage = imlocalbrighten(refinedImage);
            refinedImage(refinedImage >= 60) = 255; % make grey pixels white to increase contrast
%             imshow(refinedImage);

            % april tag     
            I = undistortImage(rgbImg,obj.Intrinsics,OutputView="same");
            [id,loc,pose] = readAprilTag(refinedImage,"tag36h11", obj.Intrinsics,obj.MarkerSize);
            
            if isempty(id)
                markerPresent = false;
                worldPose = pose;
                disp("Marker was not detected");
            else 
                % rotate axis to coinside with robot frame
                rotation = eul2rotm([0 0 -pi/2]);
                tform = rigid3d(rotation,[0 0 0]);
                updatedR = pose.Rotation * tform.Rotation;
                pose = rigid3d(updatedR, pose.Translation);

                % Swap rotation from x-axis to z-axis
                angles = rotm2eul(pose.Rotation);
                swapAxes = [angles(3) angles(2) angles(1)];
                pose.Rotation = eul2rotm(swapAxes);

                % display tag axis
                worldPoints = [0 0 0; obj.MarkerSize/2 0 0; 0 obj.MarkerSize/2 0; 0 0 obj.MarkerSize/2];

                % Get image coordinates for axes.
                imagePoints = worldToImage(obj.Intrinsics,pose(1),worldPoints);
            
                % Draw colored axes.
                I = insertShape(I,Line=[imagePoints(1,:) imagePoints(2,:); ...
                    imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
                    Color=["red","green","blue"],LineWidth=7);
            
                I = insertText(I,loc(1,:,1),id(1),BoxOpacity=1,FontSize=12);
    
                % find central tag image point
                uMin = min(loc(:,1));
                uMax = max(loc(:,1));
                vMin = min(loc(:,2));
                vMax = max(loc(:,2));
    
                centerPoint = [round(mean([uMax uMin])) round(mean([vMax vMin]))];                 
                I = insertMarker(I,centerPoint,"circle","Size",10,"Color","yellow");
                %figure(1);
                %imshow(I);
       
                % convert image point to 3d points
                depthImg = rosReadImage(depthMsg);

                depthPt = [round((centerPoint(1)/640)*480) round((centerPoint(2)/480)*360)];
                depth = (depthImg(depthPt(2),depthPt(1)))/1000 % get from sensor

                % Draw colored axes.
                depthImg = insertShape(depthImg,Line=[imagePoints(1,:) imagePoints(2,:); ...
                    imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
                    Color=["red","green","blue"],LineWidth=7);
            
                depthImg = insertText(depthImg,loc(1,:,1),id(1),BoxOpacity=1,FontSize=12);
                depthImg = insertMarker(depthImg,centerPoint,"circle","Size",10,"Color","yellow");

                %figure(2);
                %imshow(depthImg);

                translation = [depth ...
                    depth * (centerPoint(1)-obj.Intrinsics.PrincipalPoint(1))/obj.Intrinsics.FocalLength(1) ...
                    depth * (centerPoint(2)-obj.Intrinsics.PrincipalPoint(2))/obj.Intrinsics.FocalLength(2)]

%                 poseM = eul2rotm([0 0 -pi/2]); %eul2rotm([-1.57 0 -1.57]); % from camera - see urdf file
%                 poseM(1:3,4) = translation';
%                 poseM(4,4) = 1;
    
%                 quat = robotPose.Orientation;
%                 worldPoseTr = quat2rotm([quat.W quat.X quat.Y quat.Z]);
%                 worldPoseTr(1:3,4) = [robotPose.Position.X;robotPose.Position.Y;robotPose.Position.Z];
%                 worldPoseTr(4,4) = 1;
    
%                 worldPoseTM = worldPoseTr * poseM;

                quat = robotPose.Orientation;
                poseRM = quat2rotm([quat.W quat.X quat.Y quat.Z]);

                worldPoseTM = pose.Rotation*poseRM;
                % Swap rotation from y-axis to z-axis
                angles = rotm2eul(worldPoseTM);
                swapAxes = [angles(2) 0 0];
                worldPoseTM = eul2rotm(swapAxes);
                worldPoseTM(1:4,4) = [robotPose.Position.X+translation(1); robotPose.Position.Y+translation(2); robotPose.Position.Z+translation(3); 1];

                markerPresent = true;
                disp("Marker detected at");
                disp(worldPoseTM);

                worldPose = rosmessage("geometry_msgs/Pose","DataFormat","struct");
                worldPose.Position.X = worldPoseTM(1,4);
                worldPose.Position.Y = worldPoseTM(2,4);
                worldPose.Position.Z = worldPoseTM(3,4);
                quatWorldPose = rotm2quat(worldPoseTM(1:3,1:3));
                worldPose.Orientation.W = quatWorldPose(1);
                worldPose.Orientation.X = quatWorldPose(2);
                worldPose.Orientation.Y = quatWorldPose(3);
                worldPose.Orientation.Z = quatWorldPose(4);
            end
        end

        function ShutdownRos(obj)
            clear
            rosshutdown
        end

        function velMsg = GenerateVelocityMessage(obj, velocities)
            velMsg = rosmessage(obj.RobotCmd);
            velMsg.Linear.X = velocities(1,1);
            velMsg.Linear.Y = velocities(1,2);
            velMsg.Linear.Z = velocities(1,3);
            velMsg.Angular.X = velocities(1,4);
            velMsg.Angular.Y = velocities(1,5);
            velMsg.Angular.Z = velocities(1,6);
        end

        function PublishCmdVelocity(obj, velocities)
            velMsg = GenerateVelocityMessage(obj, velocities);
            send(obj.RobotCmd,velMsg)
        end

        function rbgImgMsg = CameraRgbCallback(obj)
            rbgImgMsg = receive(obj.CameraRgbSub);
        end

        function depthMsg = CameraDepthCallback(obj)
            depthMsg = receive(obj.DepthSub);
        end

        function odomMsg = OdomCallback(obj)
            odomMsg = receive(obj.OdomSub,3);
%             pose = odomMsg.Pose.Pose;
%             x = pose.Position.X;
%             y = pose.Position.Y;
%             z = pose.Position.Z;
%             
%             % display x, y, z values
%             [x y z]; 
%             
%             quat = pose.Orientation;
%             angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
%             
%             % display orientation
%             theta = rad2deg(angles(1));
        end

        function poseMsg = PoseCallback(obj) %standin for calling pose
            poseMsg = receive(obj.PoseSub,3);
%             pose = poseMsg.Pose.Pose;
%             x = pose.Position.X;
%             y = pose.Position.Y;
%             z = pose.Position.Z;
%             
%             % display x, y, z values
%             [x y z];
%             
%             quat = pose.Orientation;
%             angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
%             
%             % display orientation
%             theta = rad2deg(angles(1));
        end

        function scanMsg = LidarCallback(obj)
            scanMsg = receive(obj.LidarSub);
            figure(1)
            rosPlot(scanMsg)
        end
    end
end

