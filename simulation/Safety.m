classdef Safety < handle
    
    methods
        
        function checkCollision = CollisionDetection(self,point1OnLine,point2OnLine)
            % Plane 1 (Normal to X axis right)
            plane1Normal = [0.5,0,0.8];
            pointOnPlane1 = [0.98,0,0.8]; % centre of plane
            
            % Plane 2 (Normal to X axis left)
            plane2Normal = [-0.5,0,0.8];
            pointOnPlane2 = [-0.98,0,0.8]; % centre of plane
            
            % Plane 3 (Normal to Y axis +ve)
            plane3Normal = [0,0.5,0.8];
            pointOnPlane3 = [0,1.1,0.8]; % centre of plane

            % Plane 4 (Normal to Y axis -ve)
            plane4Normal = [0,-0.95,0.8];
            pointOnPlane4 = [0,-1.14,0.8]; % centre of plane

            % Plane 5 (Top Plane)
            plane5Normal = [0,0,0.8];
            pointOnPlane5 = [0,0,1.04]; % centre of plane
           
            
            % Check for intersection in all planes (function LinePlaneIntersection was provided)
            [intersectionPoint,checkPlane1] = self.LinePlaneIntersection(plane1Normal,pointOnPlane1,point1OnLine,point2OnLine);
            [intersectionPoint,checkPlane2] = self.LinePlaneIntersection(plane2Normal,pointOnPlane2,point1OnLine,point2OnLine);
            [intersectionPoint,checkPlane3] = self.LinePlaneIntersection(plane3Normal,pointOnPlane3,point1OnLine,point2OnLine);
            [intersectionPoint,checkPlane4] = self.LinePlaneIntersection(plane4Normal,pointOnPlane4,point1OnLine,point2OnLine);
            [intersectionPoint,checkPlane5] = self.LinePlaneIntersection(plane5Normal,pointOnPlane5,point1OnLine,point2OnLine);
            
            if checkPlane1 == 1 || checkPlane2 == 1 || checkPlane3 == 1 || checkPlane4 == 1 || checkPlane5 == 1
                checkCollision = true;               
            else
                checkCollision = false;
            end
        end

        function [intersectionPoint,check] = LinePlaneIntersection(self,planeNormal,pointOnPlane,point1OnLine,point2OnLine)
        % Given a plane (normal and point) and two points that make up another line, get the intersection
        % Check == 0 if there is no intersection
        % Check == 1 if there is a line plane intersection between the two points
        % Check == 2 if the segment lies in the plane (always intersecting)
        % Check == 3 if there is intersection point which lies outside line segment
        
            intersectionPoint = [0 0 0];
            u = point2OnLine - point1OnLine;
            w = point1OnLine - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);
            check = 0; %#ok<NASGU>
            if abs(D) < 10^-7        % The segment is parallel to plane
                if N == 0           % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;       %no intersection
                    return
                end
            end

            %compute the intersection parameter
            sI = N / D;
            intersectionPoint = point1OnLine + sI.*u;

            if (sI < 0 || sI > 1)
                check= 3;          %The intersection point  lies outside the segment, so there is no intersection
            else
                check=1;
            end
        end
        
        function [result,plotCube] = CollisionAvoidance(self,robot)
            
            % Change q1 and q2
            q1 = robot.getpos();
            q2 = q1;
            q2(1) = q1(1) - deg2rad(90);

            % Plot Cube
            centerpnt = [0,0.4,0.4];
            side = 0.1;
            plotOptions.plotFaces = true;
            [vertex,faces,faceNormals,plotCube] = self.RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
            
            robot.animate(q1);
            qWaypoints = [q1;q2];   
            isCollision = true;
            checkedTillWaypoint = 1;
            qMatrix = [];
            
            while (isCollision)     %is there a collision between q1 and q2?
                startWaypoint = checkedTillWaypoint;
                for i = startWaypoint:size(qWaypoints,1)-1
                    qMatrixJoin = self.InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));  %checks to see if it can move from the i to i+1 without collision
                    if ~self.IsCollision(robot,qMatrixJoin,faces,vertex,faceNormals) %if not a collision then then qmatrix updates
                        qMatrix = [qMatrix; qMatrixJoin];
                        for i = 1:(size(qMatrixJoin,1))
                            robot.animate(qMatrixJoin(i,:));     %animates the old q to new q
                            drawnow()
                        end
                        isCollision = false;
                        checkedTillWaypoint = i+1;
           
                        qMatrixJoin = self.InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));     %checking between last step and q2 
                        if ~self.IsCollision(robot,qMatrixJoin,faces,vertex,faceNormals)
                            qMatrix = [qMatrix;qMatrixJoin];

                            break;
                        end
            
                    else 
                        isCollision = true;
                        break;
                    end
                end
                result = isCollision;
                if result == true
                    break;
                end
            end
        end

        % IsIntersectionPointInsideTriangle
        function result = IsIntersectionPointInsideTriangle(self,intersectP,triangleVerts)

            u = triangleVerts(2,:) - triangleVerts(1,:);
            v = triangleVerts(3,:) - triangleVerts(1,:);

            uu = dot(u,u);
            uv = dot(u,v);
            vv = dot(v,v);

            w = intersectP - triangleVerts(1,:);
            wu = dot(w,u);
            wv = dot(w,v);

            D = uv * uv - uu * vv;

            s = (uv * wv - vv * wu) / D;
            if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
                result = 0;
                return;
            end
            t = (uv * wu - uu * wv) / D;
            if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
                result = 0;
                return;
            end
            result = 1;                      % intersectP is in Triangle
        end

        % IsCollision
        function result = IsCollision(self,robot,qMatrix,faces,vertex,faceNormals)
            result = false;

            for qIndex = 1:size(qMatrix,1)
                tr = self.GetLinkPoses(qMatrix(qIndex,:),robot);
                for i = 1 : size(tr,3)-1    
                    for faceIndex = 1:size(faces,1)
                        vertOnPlane = vertex(faces(faceIndex,1)',:);
                        [intersectP,check] = self.LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                        if check == 1 && self.IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                            result = true;
                        end
                    end    
                end
            end
        end

        % GetLinkPoses
        function [ transforms ] = GetLinkPoses(self,q,robot)

            links = robot.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = robot.base;

            for i = 1:length(links)
                L = links(1,i);

                current_transform = transforms(:,:, i);

                current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                transforms(:,:,i + 1) = current_transform;
            end
        end

        % FineInterpolation
        function qMatrix = FineInterpolation(self,q1,q2,maxStepRadians)    
            steps = 2;
            while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
                steps = steps + 1;
            end
            qMatrix = jtraj(q1,q2,steps);
        end

        % InterpolateWaypointRadians
        function qMatrix = InterpolateWaypointRadians(self,waypointRadians,maxStepRadians)
            qMatrix = [];
            for i = 1: size(waypointRadians,1)-1
                qMatrix = [qMatrix ; self.FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
            end
        end
        
        function [vertex,face,faceNormals,plotCube] = RectangularPrism(self,lower,upper,plotOptions,axis_h)

            vertex(1,:)=lower;
            vertex(2,:)=[upper(1),lower(2:3)];
            vertex(3,:)=[upper(1:2),lower(3)];
            vertex(4,:)=[upper(1),lower(2),upper(3)];
            vertex(5,:)=[lower(1),upper(2:3)];
            vertex(6,:)=[lower(1:2),upper(3)];
            vertex(7,:)=[lower(1),upper(2),lower(3)];
            vertex(8,:)=upper;

            face=[1,2,3;1,3,7;
                 1,6,5;1,7,5;
                 1,6,4;1,4,2;
                 6,4,8;6,5,8;
                 2,4,8;2,3,8;
                 3,7,5;3,8,5];

            if 2 < nargout    
                faceNormals = zeros(size(face,1),3);
                for faceIndex = 1:size(face,1)
                    v1 = vertex(face(faceIndex,1)',:);
                    v2 = vertex(face(faceIndex,2)',:);
                    v3 = vertex(face(faceIndex,3)',:);
                    faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
                end
            end
            %% If plot verticies
            if isfield(plotOptions,'plotVerts') && plotOptions.plotVerts
                for i=1:size(vertex,1)
                    plotCube = plot3(vertex(i,1),vertex(i,2),vertex(i,3),'r*');
                    text(vertex(i,1),vertex(i,2),vertex(i,3),num2str(i));
                end
            end

            %% If you want to plot the edges
            if isfield(plotOptions,'plotEdges') && plotOptions.plotEdges
                links=[1,2;
                    2,3;
                    3,7;
                    7,1;
                    1,6;
                    5,6;
                    5,7;
                    4,8;
                    5,8;
                    6,4;
                    4,2;
                    8,3];

                for i=1:size(links,1)
                   plotCube =  plot3(axis_h,[vertex(links(i,1),1),vertex(links(i,2),1)],...
                        [vertex(links(i,1),2),vertex(links(i,2),2)],...
                        [vertex(links(i,1),3),vertex(links(i,2),3)],'k');
                end
            end

            %% If you want to plot the edges
            if isfield(plotOptions,'plotFaces') && plotOptions.plotFaces
                tcolor = [.2 .2 .8];

                plotCube = patch('Faces',face,'Vertices',vertex,'FaceVertexCData',tcolor,'FaceColor','flat','lineStyle','none');
            end
        end
    end
end
