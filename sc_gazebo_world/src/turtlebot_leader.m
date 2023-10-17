
classdef turtlebot_leader
    properties
        RobotCmd;
        OdomSub;
    end
    methods
        function obj = turtlebot_leader()
            rosinit()
            obj.RobotCmd = rospublisher("/robot2/cmd_vel","DataFormat","struct");
            obj.OdomSub = rossubscriber("/robot2/odom","DataFormat","struct");
        end

        function cmdVel = MoveLeaderOctagon(obj)
            %Leader at Starting Position
            cmdVel = [0 0 0 0 0 0];
            %For loop to move in octagonal shape
            for i = 1:8
                %move forward distance in m
                distance = 0.5;
                fwddelay = 3;
                fvelocity = distance/fwddelay;
                if fvelocity > 0.26
                    disp("Velocity limit surpassed")
                    fvelocity = 0.26;
                end
                cmdVel = [fvelocity 0 0 0 0 0];
                PublishCmdVelocity(obj, cmdVel);
                pause(fwddelay);

                %stop moving
                stopdelay = 1;
                cmdVel = zeros(1,6);
                PublishCmdVelocity(obj, cmdVel);
                pause(stopdelay); %wait for follower to catch up

                %rotate angle in degrees
                angle = 45;
                rotdelay = 3;
                rvelocity = 2*deg2rad(angle)/rotdelay;
                if rvelocity > 1.82
                    disp("Angular Velocity limit surpassed")
                    rvelocity = 1.82;
                end
                direction = -1; %(negative = clockwise)
                cmdVel = [0 0 0 0 0 direction*rvelocity];
                PublishCmdVelocity(obj, cmdVel);
                pause(rotdelay)

                %stop rotating
                cmdVel = zeros(1,6);
                PublishCmdVelocity(obj, cmdVel);
                pause(stopdelay);

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

        function odomMsg = OdomCallback(obj)
            odomMsg = receive(obj.OdomSub,3);
            pose = odomMsg.Pose.Pose;
            x = pose.Position.X;
            y = pose.Position.Y;
            z = pose.Position.Z;
            
            % display x, y, z values
            [x y z] 
            
            quat = pose.Orientation;
            angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
            
            % display orientation
            theta = rad2deg(angles(1));
        end
    end
end


