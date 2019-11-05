classdef Main < handle
    
    properties
        traj_pub = rospublisher('/ur_driver/joint_speed','trajectory_msgs/JointTrajectory');
    end
    
    methods
        function JointSpeedPublisher(self,qDot)
            
            ur_joint_speed = rosmessage(self.traj_pub);
            ur_joint_speed.JointNames = {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
            
            
            p1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            ur_joint_speed.Points(1) = p1;
            
            
            r = rosrate(125);
            [row,col] = size(qDot);
            
            for i = 1:row
                
                p1.Velocities = qDot(i,:);
                waitfor(r);
                send(self.traj_pub,ur_joint_speed);
            end
        end
    end
end