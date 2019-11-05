classdef UR3 < handle
    
    properties
        model1;
        model2;
        
        workspace = [-1 1 -1 1 0 1];
        
        robot1BaseLocation = transl(0,0.82,0.1);%*trotz(pi);
        robot2BaseLocation = transl(0,-0.82,0.1)*trotz(pi);
    end
    
    methods
        
        function self = UR3()
            set(0,'DefaultFigureWindowStyle','docked'); %docks figure
            
            self.GetUR3(); % create two UR3 models
            self.PlotAndColourUR3(); % plot and colour robots based on data from ply files
            hold on;
        end
        
        function GetUR3(self)
            %DH Parameters to create UR3
            % NB: qlim is expressed in radians (values obtained from UR3 Datasheet)
            L1 = Link('d',0.1519,'a',0,'alpha',(pi/2),'qlim', [-2*pi 2*pi]);
            L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim',[-2*pi 2*pi]);
            L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim',[-2*pi 2*pi]);
            L4 = Link('d',0.11235,'a',0,'alpha',(pi/2),'qlim',[-2*pi 2*pi]);
            L5 = Link('d',0.08535,'a',0,'alpha',(-pi/2),'qlim',[-2*pi 2*pi]);
            L6 = Link('d',0.0819,'a',0,'alpha',0);
            
            self.model1 = SerialLink([L1 L2 L3 L4 L5 L6],'name','UR3_1','base',self.robot1BaseLocation);
            self.model2 = SerialLink([L1 L2 L3 L4 L5 L6],'name','UR3_2','base',self.robot2BaseLocation);
        end

        function PlotAndColourUR3(self)
            %Get data from each robot joint plyfile
            for linkIndex = 0:self.model1.n %robots have the same number of links
                [ robot1FaceData, robot1VertexData, robot1PlyData{linkIndex + 1} ] = plyread(['j',num2str(linkIndex),'.ply'],'tri');
                self.model1.faces{linkIndex + 1} = robot1FaceData;
                self.model1.points{linkIndex + 1} = robot1VertexData;
                
                [ robot2FaceData, robot2VertexData, robot2PlyData{linkIndex + 1} ] = plyread(['j',num2str(linkIndex),'.ply'],'tri');
                self.model2.faces{linkIndex + 1} = robot2FaceData;
                self.model2.points{linkIndex + 1} = robot2VertexData;
            end
            
            %Plot robot models
            self.model1.plot3d(zeros(1,self.model1.n),'workspace',self.workspace);
            self.model1.delay = 0;
            self.model2.plot3d(zeros(1,self.model2.n),'workspace',self.workspace);
            self.model2.delay = 0;
            
            hold on;
            
            % Get colour data from ply file and assign to robot arm
            for linkIndex = 0:self.model1.n
                robot1Handles = findobj('Tag',self.model1.name); %locate graphics object
                robot2Handles = findobj('Tag',self.model2.name);
                robot1_h = get(robot1Handles,'UserData');
                robot2_h = get(robot2Handles,'UserData');
                try
                    robot1_h.link(linkIndex+1).Children.FaceVertexCData = [robot1PlyData{linkIndex+1}.vertex.red ...
                                                                      , robot1PlyData{linkIndex+1}.vertex.green ...
                                                                      , robot1PlyData{linkIndex+1}.vertex.blue]/255;
                    robot1_h.link(linkIndex+1).Children.FaceColor = 'interp';

                    robot2_h.link(linkIndex+1).Children.FaceVertexCData = [robot2PlyData{linkIndex+1}.vertex.red ...
                                                                      , robot2PlyData{linkIndex+1}.vertex.green ...
                                                                      , robot2PlyData{linkIndex+1}.vertex.blue]/255;
                    robot2_h.link(linkIndex+1).Children.FaceColor = 'interp';    

                end
            end
        end
    end
end