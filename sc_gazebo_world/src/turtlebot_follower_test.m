
classdef turtlebot_follower_test
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
        endPositionError = 0.1;
        endOrientationError = 5;
        % for SetRefPose
        changeInDistance = 0.01;
        changeInAngle = 20;

    end
    methods
        function obj = turtlebot_follower_test()
            rosinit()

            focalLength    = [554 554];
            principalPoint = [320 240];
            imageSize      = [480 640];
            obj.Intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize); % would be better to get from camera_info topic

            obj.MarkerImg = rgb2gray (imread('../meshes/0.png'));

            obj.RobotCmd = rospublisher("/robot1/cmd_vel","DataFormat","struct");
            obj.OdomSub = rossubscriber("/robot1/odom","DataFormat","struct");
            obj.PoseSub = rossubscriber("/robot2/odom","DataFormat","struct");
            obj.LidarSub = rossubscriber("/robot1/scan","DataFormat","struct");
            obj.CameraRgbSub = rossubscriber("/robot1/camera/rgb/image_raw","DataFormat","struct");
            obj.DepthSub = rossubscriber("/robot1/camera/depth/image_raw","DataFormat","struct");
        end

        function FollowTheLeader(obj)
            tic;
            followLeader = true;
            poseType = 0;
            markerPresent = 0;
            while followLeader == true

                % Get Pose of AR tag
                while poseType == 0
                    currentOdom = OdomCallback(obj);
                    robotPose = currentOdom.Pose.Pose;
                    [markerPresent, pose] = AnalyseImage(obj, robotPose);
                    if markerPresent && ~isnan(pose.Position.X)
                        firstPose = pose;
                        poseType = 1;
                    end
                end

                % Find AR tag Pose again
                while poseType == 1
                    currentOdom = OdomCallback(obj);
                    robotPose = currentOdom.Pose.Pose;
                    [markerPresent, pose] = AnalyseImage(obj, robotPose);
                    if markerPresent && ~isnan(pose.Position.X)
                        secondPose = pose;

                        % Check if its translation/rotation/staying still
                        thetaFirst = GetTheta(obj, firstPose);
                        thetaSecond = GetTheta(obj, secondPose);

                        translationDistance = GetDistance(obj, firstPose, secondPose);

                        if translationDistance >= obj.changeInDistance
                            poseType = 2;
                            disp("translation")

                            thetaFirstSecond = abs(thetaFirst-thetaSecond)

                        elseif abs(thetaFirst-thetaSecond)>obj.changeInAngle % in progress of turning
                            poseType = 3;
                            disp("rotation")
                        else
                            poseType = 1;
                            disp("staying still")
                        end
                    end
                end
                %% Translation
                % close the gap to the goal pose
                while poseType == 2
                    currentOdom = OdomCallback(obj);
                    robotPose = currentOdom.Pose.Pose;

                    % Find distance between AR tag and follower
                    distanceToLeader = GetDistance(obj, robotPose, secondPose)

                    % determine direction
                    if distanceToLeader >= obj.Distance
                        direction = 1;
                    else
                        direction = -1;
                    end

                    % Find how far to the goal pose
                    % goalPose = DetermineGoalPose(obj, secondPose);
                    distanceToGoal = abs(distanceToLeader-obj.Distance);

                    k = VelocityController(obj, distanceToGoal);
                    % close the gap
                    if distanceToGoal>obj.endPositionError
                        PublishCmdVelocity(obj, [direction*k 0 0 0 0 0]);
                        disp("Drive to goal")
                    else
                        PublishCmdVelocity(obj, [0 0 0 0 0 0]);
                        poseType = 0;
                        disp("goal reached")
                        pause(3); %wait for rolling to stop
                    end

                end
                %% Rotation
                while poseType == 3
                    % Find AR tag Pose
                    currentOdom = OdomCallback(obj);
                    robotPose = currentOdom.Pose.Pose;
                    [markerPresent, pose] = AnalyseImage(obj, robotPose);
                    if markerPresent && ~isnan(pose.Position.X)
                        secondPose = pose;

                        pause(1.0);

                        currentOdom = OdomCallback(obj);
                        robotPose = currentOdom.Pose.Pose;
                        [markerPresent, pose] = AnalyseImage(obj, robotPose);
                        thirdPose = pose;

                        % Get angles of secondPose and thirdPose
                        thetaSecond = GetTheta(obj, secondPose);
                        thetaThird = GetTheta(obj, thirdPose);

                        % Check if it stops rotating (if not stay in place and keep checking)
                        while abs(thetaThird-thetaSecond)<=2 && poseType == 3

                            % Find AR tag Pose
                            currentOdom = OdomCallback(obj);
                            robotPose = currentOdom.Pose.Pose;
                            [markerPresent, pose] = AnalyseImage(obj, robotPose);
                            
                            if markerPresent && ~isnan(pose.Position.X)                                
                                startPose = robotPose;
                                fourthPose = pose;
                                poseType = 5;
                            end
                            while poseType==5
                                % Check if its moved away (if not stay in place and keep checking)
                                currentOdom = OdomCallback(obj);
                                robotPose = currentOdom.Pose.Pose;
                                [markerPresent, pose] = AnalyseImage(obj, robotPose);
                                if markerPresent && ~isnan(pose.Position.X) 
                                    fourthPose = pose;
                                end
                                translationDistance = GetDistance(obj, thirdPose, fourthPose);
                                if translationDistance >= obj.changeInDistance
                                    robotPose = currentOdom.Pose.Pose;
                                    thetaRobot = GetTheta(obj, robotPose);
                                    thetaStart = GetTheta(obj, startPose);

                                    % move 0.7m forward and turn 45 degrees
                                    moveDistance = GetDistance(obj, startPose, robotPose);
                                    if moveDistance <= obj.Distance
                                        distanceToGoal = abs(obj.Distance - moveDistance);
                                        k = VelocityController(obj, distanceToGoal);
                                        direction = 1; %forward
                                        PublishCmdVelocity(obj, [direction*k 0 0 0 0 0]);
                                        disp("Drive to turning spot")
                                    end
                                    if abs(thetaStart-thetaRobot)<=45 && moveDistance>obj.Distance
                                        robotPose = currentOdom.Pose.Pose;
                                        thetaRobot = GetTheta(obj, robotPose);
                                        PublishCmdVelocity(obj, [0 0 0 0 0 -0.1]);
                                        disp("turn")
                                        turnedDistance = abs(thetaStart-thetaRobot)
                                    end
                                    if abs(thetaStart-thetaRobot)>45 && moveDistance>obj.Distance
                                    poseType = 0; % Go find next goal
                                    disp("goal reached")
                                    PublishCmdVelocity(obj, [0 0 0 0 0 0]);
                                    end
                                else
                                    PublishCmdVelocity(obj, [0 0 0 0 0 0]);
                                end
                            end
                        end

                    end
                end
                %% Stop after 5 min
                if toc>5*60
                    followLeader = false;
                end
            end
            PublishCmdVelocity(obj, [0 0 0 0 0 0]);
        end

        function k = VelocityController(obj, currentDistance)
            distanceToGoal = abs(currentDistance-obj.Distance);
            if distanceToGoal>0.7
                k = 0.6;
            elseif distanceToGoal>0.3
                k = 0.4;
            elseif distanceToGoal>0.2
                k = 0.3;
            else
                k = 0.1;
            end
        end

        function goalPose = DetermineGoalPose(obj, pose)
            theta = GetTheta(obj, pose);
            % pose = pose from AR Tag
            % get x,y distance away based on angle
            translate_x = -obj.Distance*cos(theta);
            translate_y = -obj.Distance*sin(theta);

            % convert from rigid3d to ros Pose
            goalPose = rosmessage("geometry_msgs/Pose","DataFormat","struct");
            goalPose.Position.X = pose.Position.X+translate_x;
            goalPose.Position.Y = pose.Position.Y+translate_y;
            goalPose.Position.Z = pose.Position.Z;
            goalPose.Orientation = pose.Orientation;
        end

        function theta = GetTheta(obj, pose)
            quat = pose.Orientation;
            angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
            theta = rad2deg(angles(1));
        end

        function distance = GetDistance(obj, firstPose, secondPose)
            distance = sqrt((secondPose.Position.X-firstPose.Position.X)^2+(secondPose.Position.Y-firstPose.Position.Y)^2);
        end

        function [markerPresent,worldPose] = AnalyseImage(obj, robotPose)
            rgbImgMsg = CameraRgbCallback(obj);
            depthMsg = CameraDepthCallback(obj);
            rgbImg = rosReadImage(rgbImgMsg);
            grayImage = rgb2gray(rgbImg);

            % adjust the image to allow for easier detection
            refinedImage = imadjust(grayImage);
%             refinedImage = imlocalbrighten(refinedImage);
            refinedImage(refinedImage >= 5) = 255; % make grey pixels white to increase contrast
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
                depth = depthImg(centerPoint(2),centerPoint(1)); % get from sensor

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
                    depth * (centerPoint(2)-obj.Intrinsics.PrincipalPoint(2))/obj.Intrinsics.FocalLength(2)];

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
                disp(worldPoseTM)

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

