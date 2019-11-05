classdef LetsPlayCatch < handle
    
    properties
        robot = UR3; % create UR3 class object
        
        int8Msg = rosmessage('std_msgs/Int8'); %The message type is initialised
        chatPub = rospublisher('/Colour','std_msgs/Int8'); %These variables are initialised
        
        jointStateSub = rossubscriber('/joint_states');
        
        depth = rossubscriber('/kinect2/sd/image_depth_rect');
        location = rossubscriber('/BalloonCentre');
        
        ur_move_joint_client = rossvcclient('/ur_move');
        
    end
    
    methods
        
        function self = LetsPlayCatch
            
            PlaceObject('environment.ply',[0,0,0]);
            PlaceObject('kinect.ply',[-1.38,0,0]);
            PlaceObject('robot platform.ply',[0,-0.82,0.07]); % robot 1
            PlaceObject('robot platform.ply',[0,0.82,0.07]); % robot 2
            
            drawnow();
            axis equal;
        end
        
        function [balloonPose,plotBalloon] = HitBalloon(self,robotNumber,balloonPose,plotBalloon,firstHit,balloonClose,plotGreen)
            
            balloonPose(3,4) = balloonPose(3,4) + 0.075; % 0.075 offset so that origin is at the bottom instead of centre
            balloonFallRate = 0.007;
            
            steps = 15;
            s = lspb(0,1,steps); % scalar for Trapezoidal Velocity Profile
            robotQMatrix = nan(steps,6); % memory allocation
            
            if robotNumber == 1 % UR3_1 hits balloon
                % For the first hit
                if firstHit == true
                    robotCurrentConfig = self.robot.model1.getpos(); %get current joint config
                    
                    balloonOffsetPose = balloonPose;
                    balloonOffsetPose(3,4) = balloonPose(3,4) - 0.1; %offset pose so that robot accounts for balloon falling
                    
                    % Calculate target joint configuration using inverse kinematics
                    robotTargetConfig = self.robot.model1.ikcon(balloonOffsetPose,robotCurrentConfig);
                    
                    % Adjust joint 2 & 4 angles to simulate building the momentum required for hitting balloon
                    robotTargetConfig(2) = robotTargetConfig(2)-deg2rad(10);
                    robotTargetConfig(4) = robotTargetConfig(4)+deg2rad(60);
                    
                    % Create a matrix of interpolated joint configs to reach target config
                    for i = 1:1:steps
                        robotQMatrix(i,:) = (1-s(i))*robotCurrentConfig + s(i)*robotTargetConfig;
                    end
                    
                    % Simulate balloon falling and plot calculated joint configs at every iteration
                    for i = 1:1:steps
                        balloonPose(3,4) = balloonPose(3,4) - balloonFallRate;
                        try delete(plotBalloon);end
                        
                        if plotGreen == true
                            plotBalloon = PlaceObject('balloonGreen.ply',(balloonPose(1:3,4))');
                        elseif plotGreen == false
                            plotBalloon = PlaceObject('balloonBlue.ply',(balloonPose(1:3,4))');
                        end
                        
                        self.robot.model1.animate(robotQMatrix(i,:));
                        drawnow();
                    end
                    
                    % Adjust joint 4 angles to simulate building the momentum required for hitting balloon
                    robotCurrentConfig = self.robot.model1.getpos();
                    robotTargetConfig(4) = robotTargetConfig(4)-deg2rad(50);
                    
                    % Create a matrix of interpolated joint configs to reach target config
                    for i = 1:1:steps
                        robotQMatrix(i,:) = (1-s(i))*robotCurrentConfig + s(i)*robotTargetConfig;
                    end
                    
                    % Simulate balloon falling and plot calculated joint configs
                    for i = 1:1:steps
                        robotCurrentPose = self.robot.model1.fkine(self.robot.model1.getpos());
                        
                        balloonPose(3,4) = balloonPose(3,4) - balloonFallRate;
                        try delete(plotBalloon);end
                        
                        if plotGreen == true
                            plotBalloon = PlaceObject('balloonGreen.ply',(balloonPose(1:3,4))');
                        elseif plotGreen == false
                            plotBalloon = PlaceObject('balloonBlue.ply',(balloonPose(1:3,4))');
                        end
                        
                        % Check when the paddle hits the balloon -- use LinePlaneIntersection here instead
                        if (balloonPose(3,4)+(balloonFallRate*i)) <= robotCurrentPose(3,4)
                            self.robot.model1.animate(robotQMatrix(i,:));
                            drawnow();
                            break;
                        end
                        
                        self.robot.model1.animate(robotQMatrix(i,:));
                        drawnow();
                    end
                    
                    % For all other hits
                elseif firstHit == false
                    % self.GetRobotJointConfig(2); % copy UR3_2 joint configuration
                    
                    robotCurrentConfig = self.robot.model1.getpos();
                    % Calculate inverse kinematics masking all except the Y to adjust robot based on balloon pose
                    robotTargetConfig = self.robot.model1.ikcon(balloonPose,robotCurrentConfig);
                    
                    % If the balloon is close to the paddle adjust joint 4 angle
                    %to simulate building the momentum required for hitting balloon
                    if balloonClose == true
                        robotTargetConfig(4) = robotTargetConfig(4)+deg2rad(60);
                    end
                    
                    % Create a matrix of interpolated joint configs to reach target config
                    for i = 1:1:steps
                        robotQMatrix(i,:) = (1-s(i))*robotCurrentConfig + s(i)*robotTargetConfig;
                    end
                    
                    % Simulate balloon falling and plot calculated joint config
                    for i = 1:1:steps
                        self.robot.model1.animate(robotQMatrix(i,:));
                        drawnow();
                    end
                    
                    % Adjust joint 4 angle to simulate building the momentum required for hitting balloon
                    robotCurrentConfig = self.robot.model1.getpos();
                    
                    if balloonClose == true
                        robotTargetConfig(4) = robotTargetConfig(4)-deg2rad(60);
                    end
                    
                    % Create a matrix of interpolated joint configs to hit balloon
                    for i = 1:1:steps
                        robotQMatrix(i,:) = (1-s(i))*robotCurrentConfig + s(i)*robotTargetConfig;
                    end
                    
                    % Plot calculated joint configs until the paddle hits the balloon
                    for i = 1:1:steps
                        robotCurrentPose = self.robot.model1.fkine(self.robot.model1.getpos());
                        
                        if balloonPose(3,4) <= robotCurrentPose(3,4)
                            self.robot.model1.animate(robotQMatrix(i,:));
                            drawnow();
                            break;
                        end
                        
                        self.robot.model1.animate(robotQMatrix(i,:));
                        drawnow();
                    end
                end
                
            elseif robotNumber == 2 % UR3_1 hits balloon
                % For the first hit
                if firstHit == true
                    robotCurrentConfig = self.robot.model2.getpos(); %get current joint config
                    
                    balloonOffsetPose = balloonPose;
                    balloonOffsetPose(3,4) = balloonPose(3,4) - 0.1; %offset pose so that robot accounts for balloon falling
                    
                    % Calculate target joint configuration using inverse kinematics
                    robotTargetConfig = self.robot.model2.ikcon(balloonOffsetPose,robotCurrentConfig);
                    
                    % Adjust joint 2 & 4 angles to simulate building the momentum required for hitting balloon
                    robotTargetConfig(2) = robotTargetConfig(2)+deg2rad(10);
                    robotTargetConfig(4) = robotTargetConfig(4)-deg2rad(60);
                    
                    % Create a matrix of interpolated joint configs to reach target config
                    for i = 1:1:steps
                        robotQMatrix(i,:) = (1-s(i))*robotCurrentConfig + s(i)*robotTargetConfig;
                    end
                    
                    % Simulate balloon falling and plot calculated joint config
                    for i = 1:1:steps
                        balloonPose(3,4) = balloonPose(3,4) - balloonFallRate;
                        try delete(plotBalloon);end
                        
                        if plotGreen == true
                            plotBalloon = PlaceObject('balloonGreen.ply',(balloonPose(1:3,4))');
                        elseif plotGreen == false
                            plotBalloon = PlaceObject('balloonBlue.ply',(balloonPose(1:3,4))');
                        end
                        
                        self.robot.model2.animate(robotQMatrix(i,:));
                        drawnow();
                    end
                    
                    % Adjust joint 4 angles to simulate building the momentum required for hitting balloon
                    robotCurrentConfig = self.robot.model2.getpos();
                    robotTargetConfig(4) = robotTargetConfig(4)+deg2rad(50);
                    
                    % Create a matrix of interpolated joint configs to reach target config
                    for i = 1:1:steps
                        robotQMatrix(i,:) = (1-s(i))*robotCurrentConfig + s(i)*robotTargetConfig;
                    end
                    
                    % Simulate balloon falling and plot calculated joint config
                    for i = 1:1:steps
                        robotCurrentPose = self.robot.model2.fkine(self.robot.model2.getpos());
                        
                        balloonPose(3,4) = balloonPose(3,4) - balloonFallRate;
                        try delete(plotBalloon);end
                        
                        if plotGreen == true
                            plotBalloon = PlaceObject('balloonGreen.ply',(balloonPose(1:3,4))');
                        elseif plotGreen == false
                            plotBalloon = PlaceObject('balloonBlue.ply',(balloonPose(1:3,4))');
                        end
                        
                        % Check when the paddle hits the balloon -- use LinePlaneIntersection here instead
                        if (balloonPose(3,4)+(balloonFallRate*i)) <= robotCurrentPose(3,4)
                            self.robot.model2.animate(robotQMatrix(i,:));
                            drawnow();
                            break;
                        end
                        
                        self.robot.model2.animate(robotQMatrix(i,:));
                        drawnow();
                    end
                    
                    % For all other hits
                elseif firstHit == false
                    % self.GetRobotJointConfig(1); % copy UR3_1 joint configuration
                    
                    robotCurrentConfig = self.robot.model2.getpos();
                    % Calculate inverse kinematics masking all except the Y to adjust robot based on balloon pose
                    robotTargetConfig = self.robot.model2.ikcon(balloonPose,robotCurrentConfig);
                    
                    % If the balloon is close to the paddle adjust joint 4 angle
                    %to simulate building the momentum required for hitting balloon
                    if balloonClose == true
                        robotTargetConfig(4) = robotTargetConfig(4)+deg2rad(60);
                    end
                    
                    % Create a matrix of interpolated joint configs to reach target config
                    for i = 1:1:steps
                        robotQMatrix(i,:) = (1-s(i))*robotCurrentConfig + s(i)*robotTargetConfig;
                    end
                    
                    % Simulate balloon falling and plot calculated joint config
                    for i = 1:1:steps
                        self.robot.model2.animate(robotQMatrix(i,:));
                        drawnow();
                    end
                    
                    % Adjust joint 4 angle to simulate building the momentum required for hitting balloon
                    robotCurrentConfig = self.robot.model2.getpos();
                    
                    if balloonClose == true
                        robotTargetConfig(4) = robotTargetConfig(4)-deg2rad(60);
                    end
                    
                    % Create a matrix of interpolated joint configs to hit balloon
                    for i = 1:1:steps
                        robotQMatrix(i,:) = (1-s(i))*robotCurrentConfig + s(i)*robotTargetConfig;
                    end
                    
                    % Plot calculated joint configs until the paddle hits the balloon
                    for i = 1:1:steps
                        robotCurrentPose = self.robot.model2.fkine(self.robot.model2.getpos());
                        
                        if balloonPose(3,4) <= robotCurrentPose(3,4)
                            self.robot.model2.animate(robotQMatrix(i,:));
                            drawnow();
                            break;
                        end
                        
                        self.robot.model2.animate(robotQMatrix(i,:));
                        drawnow();
                    end
                end
            end
        end
        
        function [balloonPose,plotBalloon] = PassBalloon(self,hittingRobot,balloonPose,plotBalloon,plotGreen)
            
            % UR3_1 passing the balloon
            if hittingRobot == 1
                % Get the X and Z coordinates of the balloon (Y stays the same)
                balloonCurrentX = balloonPose(1,4);
                balloonCurrentY = balloonPose(2,4);
                balloonCurrentZ = balloonPose(3,4);
                
                increment = (abs(balloonCurrentY)*2)/20; %20 steps
                
                % Solve quadratic equation to determine a,b and c values (for balloon parabolic trajectory)
                x = balloonCurrentY:-increment:-balloonCurrentY;
                c = 0.7;
                b = 0;
                a = (balloonCurrentZ-c)/(balloonCurrentY^2);
                y = a.*(x.^2)+b.*x+c;
                
                % Plot a parabolic trajectory of the balloon from current X to opposite X coordinate
                for i = 1:size(y,2)
                    try delete(plotBalloon);end
                    
                    if plotGreen == true
                        plotBalloon = PlaceObject('balloonGreen.ply',(balloonPose(1:3,4))');
                    elseif plotGreen == false
                        plotBalloon = PlaceObject('balloonBlue.ply',(balloonPose(1:3,4))');
                    end
                    
                    drawnow();
                    
                    balloonPose(1:3,4) = [balloonCurrentX,x(i),y(i)];
                    
                    % When the balloon is close to the paddle, prepare the robot to hit it
                    if size(y,2) == i
                        firstHit = false;
                        balloonClose = true;
                        self.HitBalloon(2,balloonPose,plotBalloon,firstHit,balloonClose);
                        continue;
                        
                        % When the balloon Z starts decreasing, adjust robot base angle
                    elseif size(y,2)-10 <= i
                        firstHit = false;
                        balloonClose = false;
                        self.HitBalloon(2,balloonPose,plotBalloon,firstHit,balloonClose);
                    end
                end
                
                % UR3_2 passing the balloon
            elseif hittingRobot == 2
                % Get the X and Z coordinates of the balloon (Y stays the same)
                balloonCurrentX = balloonPose(1,4);
                balloonCurrentY = balloonPose(2,4);
                balloonCurrentZ = balloonPose(3,4);
                
                increment = (abs(balloonCurrentY)*2)/20; %20 steps in between
                
                % Solve quadratic equation to determine a,b and c values (for balloon parabolic trajectory)
                x = balloonCurrentY:increment:-balloonCurrentY;
                c = 0.7;
                b = 0;
                a = (balloonCurrentZ-c)/(balloonCurrentY^2);
                y = a.*(x.^2)+b.*x+c;
                
                % Plot a parabolic trajectory of the balloon from current X to opposite X coordinate
                for i = 1:size(y,2)
                    try delete(plotBalloon);end
                    
                    if plotGreen == true
                        plotBalloon = PlaceObject('balloonGreen.ply',(balloonPose(1:3,4))');
                    elseif plotGreen == false
                        plotBalloon = PlaceObject('balloonBlue.ply',(balloonPose(1:3,4))');
                    end
                    
                    drawnow();
                    
                    balloonPose(1:3,4) = [balloonCurrentX,x(i),y(i)];
                    % When the balloon is close to the paddle, prepare the robot to hit it
                    if size(y,2) == i
                        firstHit = false;
                        balloonClose = true;
                        self.HitBalloon(1,balloonPose,plotBalloon,firstHit,balloonClose);
                        continue;
                        
                        % When the balloon Z starts decreasing, adjust robot base angle
                    elseif size(y,2)-10 <= i
                        firstHit = false;
                        balloonClose = false;
                        self.HitBalloon(1,balloonPose,plotBalloon,firstHit,balloonClose);
                    end
                end
            end
        end
        
        function ColourDetectionPublisher(self,colour)
            BLUE = 0;
            GREEN = 1;
            
            if(colour == BLUE)
                self.int8Msg.Data = 0;      %number for a blue detection
                
            elseif(colour == GREEN)
                self.int8Msg.Data = 1;       %number for a green detection
            end
            send(self.chatPub,self.int8Msg); %this is to actually publish the message
        end
        
        function RMRC(self,balloonPose)
            mainObj  = Main();
            counter = 0;
            % This is a modified RMRC for our purposes we run through the
            % process twice
            while counter < 2
                counter = counter +1;
                
                robotCurrentConfig = self.GetJointStates();
                
                if counter == 1
                    t = 1;             % Total time (s)
                elseif counter == 2
                    t = t*scalar*1.1; %The time is multiplied by scalar to make sure joint vel limits are not reached also added 1.1 to make sure that joint limits would not be reached
                end
                deltaT = 0.008;      % Control frequency which matches real robot rate
                steps = round(t/deltaT);   % No. of steps rounded
                epsilon = 0.3;      % Threshold value for manipulability/Damped Least Squares
                W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
                W2 = [1 1 1 0.5 0.5 0.5];           %Weighting matrix for the joint velocity limits specific to the UR3
                
                % Allocate array data
                m = zeros(steps,1);             % Array for Measure of Manipulability
                qMatrix = zeros(steps,6);       % Array for joint anglesR
                qdot = zeros(steps,6);          % Array for joint velocities
                theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
                x = zeros(3,steps);             % Array for x-y-z trajectory
                
                % Set up trajectory, initial pose
                s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
                
                robotCurrentCartConfig = self.robot.model1.fkine(robotCurrentConfig);
                
                robotTargetCartConfig = balloonPose(1:3,4);
                
                for i=1:1:steps
                    x(1,i) = (1-s(i))*robotCurrentCartConfig(1,4) + s(i)*robotTargetCartConfig(1); % Points in x
                    x(2,i) = (1-s(i))*robotCurrentCartConfig(2,4) + s(i)*robotTargetCartConfig(2); % Points in y
                    x(3,i) = (1-s(i))*robotCurrentCartConfig(3,4) + s(i)*robotTargetCartConfig(3); % Points in z
                    
                    theta(1,i) = 0;            % Roll angle
                    theta(2,i) = 0;            % Pitch angle
                    theta(3,i) = 0;            % Yaw angle
                end
                
                T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];    % Create transformation of first point and angle
                q0 = robotCurrentConfig;                                              % Initial guess for joint angles
                qMatrix(1,:) = self.robot.model1.ikcon(T,q0);                         % Solve joint angles to achieve first waypoint
                
                % Track the trajectory with RMRC
                for i = 1:1:steps-1
                    T = self.robot.model1.fkine(qMatrix(i,:));                              % Get forward transformation at current joint state
                    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                    Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
                    S = Rdot*Ra';                                                           % Skew symmetric
                    linear_velocity = (1/deltaT)*deltaX;
                    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                    J = self.robot.model1.jacob0(qMatrix(i,:));                             % Get Jacobian at current joint state
                    m(i) = sqrt(det(J*J'));
                    if m(i) < epsilon                                                       % If manipulability is less than given threshold
                        lambda = (1 - m(i)/epsilon)*5E-2;
                    else
                        lambda = 0;
                    end
                    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                    qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the vector)
                    
                    for j = 1:6                                                             % Loop through joints 1 to 6
                        if qMatrix(i,j) + deltaT*qdot(i,j) < self.robot.model1.qlim(j,1)    % If next joint angle is lower than joint limit...
                            qdot(i,j) = 0; % Stop the motor
                        elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.robot.model1.qlim(j,2)% If next joint angle is greater than joint limit ...
                            qdot(i,j) = 0; % Stop the motor
                        end
                    end
                end
                
                if counter == 1
                    qdotTemp = qdot;                    %using a tempVariable for safety
                    qdotTemp = qdotTemp.*W2;            %multiply by joint vel lim weightings
                    maxQdotValue = max(abs(qdotTemp));  %calculate largest Velocity
                    maxVal = max(maxQdotValue);         %Create a scalar based on this
                    maxJointVel = pi;                   %Max Joint vel of UR3 after weighting
                    scalar = maxJointVel/maxVal;        %creates a scalar to manipulate time of second RMRC run
                end
            end
            
            %removed python pubisher code
            if counter == 2  % redundant but a double check
                mainObj.JointSpeedPublisher(qdot);
            end
        end
        
        function balloonPose = BalloonDetectionSubscriber(self)
            receive(self.depth,1);
            receive(self.location,1);
            X = self.location.LatestMessage.X;
            Y = self.location.LatestMessage.Y;
            imageDep = readImage(self.depth.LatestMessage);
            
            %get and process depth value
            dis = imageDep(Y,X);
            
            %if error value (0) take new spot
            tempX = X;
            while (dis == 0)
                tempX = tempX+1;
                dis = imageDep(Y,tempX);
            end
            
            dis = double(dis);           %from out other calculations/calibration
            
            % Turn pixel value to real world points
            pixelNumY = 1:190;                      %Create a matrix
            pixelNumY = rescale(pixelNumY,0,725);   %rescale to real distance essentially creating a readtable
            testCamY = pixelNumY(Y-16)-725;         %shift it to return values in correct frame
            
            pixelNumX = 1:206;                      %same thing for ex
            pixelNumX = rescale(pixelNumX,0,820);
            testCamX = pixelNumX(X-50)-820;
            
            testCamZ = dis;
            
            cameraPos = transl([-1.38,0,0])*trotx(-pi/2)*troty(pi/2);   %this is the real life cameraPosition
            
            testCamX2 = testCamZ*0.001+cameraPos(1,4);  %Move to the global frame and change to metres
            testCamY2 = -testCamX*0.001;
            testCamZ2 = -testCamY*0.001;
            
            
            balloonPose = transl(testCamX2,testCamY2,testCamZ2); %single variable for position
        end
        
        function jointStates = GetJointStates(self)
            jointStateMessage = receive(self.jointStateSub,5);
            jointStates = jointStateMessage.Position;     
        end
        
        function LiveRobotDemo(self)
            counter = 0;
            while 1
                balloonPose = self.BalloonDetectionSubscriber;
                counter = counter +1;
                self.RMRC(balloonPose);
            end
        end
        
        function JoystickControl(self,robotNumber,duration)
            
            ur_move_joint_msg = rosmessage(self.ur_move_joint_client);
            
            % setup joystick
            id = 1; % one joystick being used
            joy = vrjoystick(id);
            caps(joy) % display joystick information
            
            % Start "real-time" simulation
            % Set initial robot configuration 'q'
            
            if robotNumber == 1
                self.robot.model1.tool = transl(0,0,0);   % Define tool frame on end-effector
                q = self.robot.model1.getpos();
                
            elseif robotNumber == 2
                self.robot.model2.tool = transl(0,0,0);   % Define tool frame on end-effector
                q = self.robot.model2.getpos();
            end
            
            dt = 0.3;      % Set time step for simulation (seconds)
            
            n = 0;  % Initialise step count to zero
            tic;    % recording simulation start time
            while( toc < duration && duration ~=0)
                
                n=n+1; % increment step count
                
                % read joystick
                [axes, buttons, povs] = read(joy);
                
                Kv = 0.2; % linear velocity gain
                Kw = 1.0; % angular velocity gain
                
                vx = Kv*(axes(1)); %up down
                vy = Kv*(axes(2)); %side
                vz = Kv*(buttons(3)-buttons(4)); % y and b
                
                wx = Kw*(buttons(1)-buttons(2)); %roll a and b
                wy = Kw*(buttons(5)-buttons(6)); %pitch lb and rb
                wz = Kw*(buttons(9)-buttons(10)); %yaw select and start
                
                dx = [vx;vy;vz;wx;wy;wz]; % combined velocity vector
                
                % use J inverse to calculate joint velocity
                lambda = 0.5;
                
                if robotNumber == 1
                    J = self.robot.model1.jacob0(q);
                    
                elseif robotNumber == 2
                    J = self.robot.model2.jacob0(q);
                end
                
                Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
                dq = Jinv_dls*dx;
                
                % apply joint velocity to step robot joint angles
                q = q + dq'*dt;
                
                % Update plot
                if robotNumber == 1
                    
                    self.robot.model1.animate(q);
                    ur_move_joint_msg.Q = q; % Set joint state values
                    self.ur_move_joint_client.call(ur_move_joint_msg); % Send values to robot
                    
                elseif robotNumber == 2
                    self.robot.model2.animate(q);
                    ur_move_joint_msg.Q = q; % Set joint state values
                    self.ur_move_joint_client.call(ur_move_joint_msg); % Send values to robot
                end
                
                drawnow();
                
                % wait until loop time elapsed
                if (toc > dt*n)
                    warning('Loop %i took too much time - consider increating dt',n);
                end
                while (toc < dt*n) % wait until loop time (dt) has elapsed
                end
            end
        end
    end
end

