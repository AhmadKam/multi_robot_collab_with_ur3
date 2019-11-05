function varargout = SimulationGUI(varargin)

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SimulationGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @SimulationGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before SimulationGUI is made visible.
function SimulationGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)5
% varargin   command line arguments to SimulationGUI (see VARARGIN)

    % Choose default command line output for SimulationGUI
    handles.output = hObject;

    % Create Simulation and Safety class objects
    simObj = LetsPlayCatch;
    view(-36,20);
    camlight('left');
    robotCurrentConfig = deg2rad([90 -75 145 -115 90 0]);
    simObj.robot.model1.animate(robotCurrentConfig);
    simObj.robot.model2.animate(robotCurrentConfig);

    safetyObj = Safety;

    % Save the object as a field within handles
    handles.simObj = simObj;
    handles.safetyObj = safetyObj;

    % Update handles structure
    guidata(hObject, handles);

    % Update joint sliders/input boxes and XYZ/RPY input boxes
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');

    if controlRobot1 == true
        ControlUR3_1_Callback(handles.ControlUR3_1, eventdata,handles);

    elseif controlRobot2 == true
        ControlUR3_2_Callback(handles.ControlUR3_2, eventdata,handles);
    end

    
% UIWAIT makes SimulationGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SimulationGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
    varargout{1} = handles.output;


% --- Executes on selection change in ViewType
function ViewType_Callback(hObject, eventdata, handles)
% hObject    handle to ViewType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    % Determine selected menu option
    contents = cellstr(get(hObject,'String'));
    viewAngle = contents{get(hObject,'Value')};
    
    %Compares chosen value to menu options and sets view angle
    if (strcmp(viewAngle,'3D View')) 
        view(-36,20);
    elseif (strcmp(viewAngle,'Top View'))
        view(0,90);
    elseif (strcmp(viewAngle,'Front View'))
        view(0,0);
    elseif (strcmp(viewAngle,'Side View'))
        view(90,0);
    end

% --- Executes during object creation, after setting all properties.
function ViewType_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ViewType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defautUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in Home -- robot goes back to Home position
function Home_Callback(hObject, eventdata, handles)
% hObject    handle to Home (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)   
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    steps = 30;
    
    if controlRobot1 == true % if UR3_1 is selected
        robotCurrentConfig = simObj.robot.model1.getpos(); % get current joint configuration
        robotTargetConfig = [0 -pi/2 0 -pi/2 0 0];
        
    elseif controlRobot2 == true % if UR3_2 is selected
        robotCurrentConfig = simObj.robot.model2.getpos();
        robotTargetConfig = [0 -pi/2 0 -pi/2 0 0];
    end
    
    % Creates a matrix of interpolated joint configurations
    s = lspb(0,1,steps);
    qMatrix = nan(steps,6);
    for i = 1:1:steps
        qMatrix(i,:) = (1-s(i))*robotCurrentConfig + s(i)*robotTargetConfig;
    end
    
    % Plots the calculated matrix
    for i = 1:1:steps
        eStopStatus = get(handles.EStop,'String'); % check if Estop is pressed
        checkCollision = get(handles.SafetyStatus,'String'); % check for light curtain collision
        
        % if UR3_1 is selected -- checks if Estop is pressed and for collision with light curtain
        if controlRobot1 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
                simObj.robot.model1.animate(qMatrix(i,:));
                drawnow();
                
        % if UR3_2 is selected -- checks if Estop is pressed and for collision with light curtain
        elseif controlRobot2 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
            simObj.robot.model2.animate(qMatrix(i,:));
            drawnow();
            
        % If Estop is pressed or light curtain is triggered then stop robot movement
        elseif eStopStatus == "Press to Release" || checkCollision == "Safety Status: Unsafe! Unexpected object in workspace"...
                                                 || checkCollision == "Safety Status: Unexpected object in robot trajectory!"
            break;
        end
    end
    
    % Update joint sliders/input boxes and XYZ/RPY input boxes
    if controlRobot1 == true
        ControlUR3_1_Callback(handles.ControlUR3_1, eventdata,handles);
        
    elseif controlRobot2 == true
        ControlUR3_2_Callback(handles.ControlUR3_2, eventdata,handles);
    end


% --- Executes on button press in SimulateDemo
function SimulateDemo_Callback(hObject, eventdata, handles)
% hObject    handle to SimulateDemo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    
    if (get(handles.CheckForRealRobot,'Value')) == 0
        
        balloonPose = eye(4);
        balloonPose(1,4) = (-0.1 +(0.1+0.1)*rand(1,1))+balloonPose(1,4); %x boundary within box
        balloonPose(2,4) = (0.35 +(0.4-0.35)*rand(1,1))+balloonPose(2,4); %y boundary within box
        balloonPose(3,4) = (0.5 +(0.6-0.5)*rand(1,1))+balloonPose(3,4); %z boundary within box

        green = false;

        if (get(handles.DetectBlue,'Value')) == 1
            plotBalloon = PlaceObject('balloonBlue.ply',(balloonPose(1:3,4))');
            green = false;

        elseif (get(handles.DetectGreen,'Value')) == 1
            plotBalloon = PlaceObject('balloonGreen.ply',(balloonPose(1:3,4))');
            green = true;
        end

        % UR3_1 hits balloon after first balloon plot
        firstHit = true; % this is the first balloon hit in the simulation
        balloonClose = false; % the balloon is not close to the paddle

        eStopStatus = get(handles.EStop,'String'); % check if Estop is pressed
        checkCollision = get(handles.SafetyStatus,'String'); % check for light curtain collision

        % checks if Estop is pressed and for collision with light curtain
        if eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
            [balloonPose,plotBalloon] = simObj.HitBalloon(1,balloonPose,plotBalloon,firstHit,balloonClose,green);
        end

        % Simulate ten total balloon passes between both robots
        for i = 1:1:5
            eStopStatus = get(handles.EStop,'String'); % check if Estop is pressed
            checkCollision = get(handles.SafetyStatus,'String'); % check for light curtain collision

            % checks if Estop is pressed and for collision with light curtain
            if eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
                [balloonPose,plotBalloon] = simObj.PassBalloon(1,balloonPose,plotBalloon,green);

                [balloonPose,plotBalloon] = simObj.PassBalloon(2,balloonPose,plotBalloon,green);

            % If Estop is pressed or light curtain is triggered then stop robot movement
            elseif eStopStatus == "Press to Release" || checkCollision == "Safety Status: Unsafe! Unexpected object in workspace"...
                                                     || checkCollision == "Safety Status: Unexpected object in robot trajectory!"
                try delete(plotBalloon); end
                break;
            end 
        end
        
    elseif (get(handles.CheckForRealRobot,'Value')) == 1
        simObj.LiveRobotDemo()
    end


% --- Executes on button press in EStop.
function EStop_Callback(hObject, eventdata, handles)
% hObject    handle to EStop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    if (strcmp((get(hObject,'String')),'Press to Release')) == 1
        set(hObject,'String','Emergency Stop'); % set button title
        safetyStatus = 'Safety Status: Safe'; % change safety status
        set(handles.SafetyStatus,'String',safetyStatus,'ForegroundColor',[0.392 0.831 0.075]);
        try set(handles.LightCurtainSensor,'Enable','on');end % reenables 'Moving Object' slider if needed
        uiresume;
        
    elseif (strcmp((get(hObject,'String')),'Emergency Stop'))
        set(hObject,'String','Press to Release'); % set button title
        safetyStatus = 'Safety Status: E-Stop Pressed!';
        set(handles.SafetyStatus,'String',safetyStatus,'ForegroundColor','red');
        if get(handles.SimulateDemo,'Value') == true
            uiwait;
        end
        
    end 
    
    % Update joint sliders/input boxes and XYZ/RPY input boxes
    if controlRobot1 == true
        ControlUR3_1_Callback(handles.ControlUR3_1, eventdata,handles);
        
    elseif controlRobot2 == true
        ControlUR3_2_Callback(handles.ControlUR3_2, eventdata,handles);
    end
    
   
function XEditBox_Callback(hObject, eventdata, handles)
% hObject    handle to XEditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    % Get XYZ and RPY values from edit boxes
    xValue = str2double(get(hObject,'String')); 
    yValue = str2double(get(handles.YEditBox,'String'));
    zValue = str2double(get(handles.ZEditBox,'String'));
    
    rollValue = str2double(get(handles.RollEditBox,'String'));
    pitchValue = str2double(get(handles.PitchEditBox,'String'));
    yawValue = str2double(get(handles.YawEditBox,'String'));
    
    % Determine inverse kinemeatics based on values obtained
    rotationTr = rpy2tr(rollValue,pitchValue,yawValue);
    robotTargetPose = transl(xValue,yValue,zValue) * rotationTr;
    
    if controlRobot1 == true %if UR3_1 is selected
        robotTargetConfig = simObj.robot.model1.ikcon(robotTargetPose,simObj.robot.model1.getpos());
        
    elseif controlRobot2 == true %if UR3_2 is selected
        robotTargetConfig = simObj.robot.model2.ikcon(robotTargetPose,simObj.robot.model2.getpos());
    end
    
    % Checks that joint configuration is within joint limits
    for i = 1:6
        if 2*pi < robotTargetConfig(i) || robotTargetConfig(i) <= -2*pi
            error("Angle for joint %d is not within the joint limits!",i);
        end
    end
    
    % Set joint sliders to correct value
    set(handles.J1Slider,'Value',rad2deg(robotTargetConfig(1)));
    set(handles.J2Slider,'Value',rad2deg(robotTargetConfig(2)));
    set(handles.J3Slider,'Value',rad2deg(robotTargetConfig(3)));
    set(handles.J4Slider,'Value',rad2deg(robotTargetConfig(4)));
    set(handles.J5Slider,'Value',rad2deg(robotTargetConfig(5)));
    set(handles.J6Slider,'Value',rad2deg(robotTargetConfig(6)));
    
    % Set joint edit boxes to correct value
    set(handles.J1EditBox,'String',rad2deg(robotTargetConfig(1)));
    set(handles.J2EditBox,'String',rad2deg(robotTargetConfig(2)));
    set(handles.J3EditBox,'String',rad2deg(robotTargetConfig(3)));
    set(handles.J4EditBox,'String',rad2deg(robotTargetConfig(4)));
    set(handles.J5EditBox,'String',rad2deg(robotTargetConfig(5)));
    set(handles.J6EditBox,'String',rad2deg(robotTargetConfig(6)));
    
% --- Executes during object creation, after setting all properties.
function XEditBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to XEditBox (sgee GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

    
function YEditBox_Callback(hObject, eventdata, handles)
% hObject    handle to YEditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    % Get XYZ and RPY values from edit boxes
    xValue = str2double(get(handles.XEditBox,'String')); 
    yValue = str2double(get(hObject,'String'));
    zValue = str2double(get(handles.ZEditBox,'String'));
    
    rollValue = str2double(get(handles.RollEditBox,'String'));
    pitchValue = str2double(get(handles.PitchEditBox,'String'));
    yawValue = str2double(get(handles.YawEditBox,'String'));
    
    % Determine inverse kinemeatics based on values obtained
    rotationTr = rpy2tr(rollValue,pitchValue,yawValue);
    robotTargetPose = transl(xValue,yValue,zValue) * rotationTr;
    
    if controlRobot1 == true %if UR3_1 is selected
        robotTargetConfig = simObj.robot.model1.ikcon(robotTargetPose,simObj.robot.model1.getpos());
        
    elseif controlRobot2 == true %if UR3_2 is selected
        robotTargetConfig = simObj.robot.model2.ikcon(robotTargetPose,simObj.robot.model2.getpos());
    end
    
    % Checks that joint configuration is within joint limits
    for i = 1:6
        if 2*pi < robotTargetConfig(i) || robotTargetConfig(i) <= -2*pi
            error("Angle for joint %d is not within the joint limits!",i);
        end
    end
    
    % Set joint sliders to correct value
    set(handles.J1Slider,'Value',rad2deg(robotTargetConfig(1)));
    set(handles.J2Slider,'Value',rad2deg(robotTargetConfig(2)));
    set(handles.J3Slider,'Value',rad2deg(robotTargetConfig(3)));
    set(handles.J4Slider,'Value',rad2deg(robotTargetConfig(4)));
    set(handles.J5Slider,'Value',rad2deg(robotTargetConfig(5)));
    set(handles.J6Slider,'Value',rad2deg(robotTargetConfig(6)));
    
    % Set joint edit boxes to correct value
    set(handles.J1EditBox,'String',rad2deg(robotTargetConfig(1)));
    set(handles.J2EditBox,'String',rad2deg(robotTargetConfig(2)));
    set(handles.J3EditBox,'String',rad2deg(robotTargetConfig(3)));
    set(handles.J4EditBox,'String',rad2deg(robotTargetConfig(4)));
    set(handles.J5EditBox,'String',rad2deg(robotTargetConfig(5)));
    set(handles.J6EditBox,'String',rad2deg(robotTargetConfig(6)));


% --- Executes during object creation, after setting all properties.
function YEditBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to YEditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


function ZEditBox_Callback(hObject, eventdata, handles)
% hObject    handle to ZEditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    % Get XYZ and RPY values from edit boxes
    xValue = str2double(get(handles.XEditBox,'String')); 
    yValue = str2double(get(handles.YEditBox,'String'));
    zValue = str2double(get(hObject,'String'));
    
    rollValue = str2double(get(handles.RollEditBox,'String'));
    pitchValue = str2double(get(handles.PitchEditBox,'String'));
    yawValue = str2double(get(handles.YawEditBox,'String'));
    
    % Determine inverse kinemeatics based on values obtained
    rotationTr = rpy2tr(rollValue,pitchValue,yawValue);
    robotTargetPose = transl(xValue,yValue,zValue) * rotationTr;
    
    if controlRobot1 == true %if UR3_1 is selected
        robotTargetConfig = simObj.robot.model1.ikcon(robotTargetPose,simObj.robot.model1.getpos());
        
    elseif controlRobot2 == true %if UR3_2 is selected
        robotTargetConfig = simObj.robot.model2.ikcon(robotTargetPose,simObj.robot.model2.getpos());
    end
    
    % Checks that joint configuration is within joint limits
    for i = 1:6
        if 2*pi < robotTargetConfig(i) || robotTargetConfig(i) <= -2*pi
            error("Angle for joint %d is not within the joint limits!",i);
        end
    end
    
    % Set joint sliders to correct value
    set(handles.J1Slider,'Value',rad2deg(robotTargetConfig(1)));
    set(handles.J2Slider,'Value',rad2deg(robotTargetConfig(2)));
    set(handles.J3Slider,'Value',rad2deg(robotTargetConfig(3)));
    set(handles.J4Slider,'Value',rad2deg(robotTargetConfig(4)));
    set(handles.J5Slider,'Value',rad2deg(robotTargetConfig(5)));
    set(handles.J6Slider,'Value',rad2deg(robotTargetConfig(6)));
    
    % Set joint edit boxes to correct value
    set(handles.J1EditBox,'String',rad2deg(robotTargetConfig(1)));
    set(handles.J2EditBox,'String',rad2deg(robotTargetConfig(2)));
    set(handles.J3EditBox,'String',rad2deg(robotTargetConfig(3)));
    set(handles.J4EditBox,'String',rad2deg(robotTargetConfig(4)));
    set(handles.J5EditBox,'String',rad2deg(robotTargetConfig(5)));
    set(handles.J6EditBox,'String',rad2deg(robotTargetConfig(6)));

% --- Executes during object creation, after setting all properties.
function ZEditBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ZEditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


function RollEditBox_Callback(hObject, eventdata, handles)
% hObject    handle to RollEditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    % Get XYZ and RPY values from edit boxes
    xValue = str2double(get(handles.XEditBox,'String')); 
    yValue = str2double(get(handles.YEditBox,'String'));
    zValue = str2double(get(handles.ZEditBox,'String'));
    
    rollValue = str2double(get(hObject,'String'));
    pitchValue = str2double(get(handles.PitchEditBox,'String'));
    yawValue = str2double(get(handles.YawEditBox,'String'));
    
    % Determine inverse kinemeatics based on values obtained
    rotationTr = rpy2tr(rollValue,pitchValue,yawValue);
    robotTargetPose = transl(xValue,yValue,zValue) * rotationTr;
       
    if controlRobot1 == true %if UR3_1 is selected
        robotTargetConfig = simObj.robot.model1.ikcon(robotTargetPose,simObj.robot.model1.getpos());
        
    elseif controlRobot2 == true %if UR3_2 is selected
        robotTargetConfig = simObj.robot.model2.ikcon(robotTargetPose,simObj.robot.model2.getpos());
    end
    
    % Checks that joint configuration is within joint limits
    for i = 1:6
        if 2*pi < robotTargetConfig(i) || robotTargetConfig(i) <= -2*pi
            error("Angle for joint %d is not within the joint limits!",i);
        end
    end
    
    % Set joint sliders to correct value
    set(handles.J1Slider,'Value',rad2deg(robotTargetConfig(1)));
    set(handles.J2Slider,'Value',rad2deg(robotTargetConfig(2)));
    set(handles.J3Slider,'Value',rad2deg(robotTargetConfig(3)));
    set(handles.J4Slider,'Value',rad2deg(robotTargetConfig(4)));
    set(handles.J5Slider,'Value',rad2deg(robotTargetConfig(5)));
    set(handles.J6Slider,'Value',rad2deg(robotTargetConfig(6)));
    
    % Set joint edit boxes to correct value
    set(handles.J1EditBox,'String',rad2deg(robotTargetConfig(1)));
    set(handles.J2EditBox,'String',rad2deg(robotTargetConfig(2)));
    set(handles.J3EditBox,'String',rad2deg(robotTargetConfig(3)));
    set(handles.J4EditBox,'String',rad2deg(robotTargetConfig(4)));
    set(handles.J5EditBox,'String',rad2deg(robotTargetConfig(5)));
    set(handles.J6EditBox,'String',rad2deg(robotTargetConfig(6)));

% --- Executes during object creation, after setting all properties.
function RollEditBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to RollEditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


function PitchEditBox_Callback(hObject, eventdata, handles)
% hObject    handle to PitchEditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    % Get XYZ and RPY values from edit boxes
    xValue = str2double(get(handles.XEditBox,'String')); 
    yValue = str2double(get(handles.YEditBox,'String'));
    zValue = str2double(get(handles.ZEditBox,'String'));
    
    rollValue = str2double(get(handles.RollEditBox,'String'));
    pitchValue = str2double(get(hObject,'String'));
    yawValue = str2double(get(handles.YawEditBox,'String'));
    
    % Determine inverse kinemeatics based on values obtained
    rotationTr = rpy2tr(rollValue,pitchValue,yawValue);
    robotTargetPose = transl(xValue,yValue,zValue) * rotationTr;
       
    if controlRobot1 == true %if UR3_1 is selected
        robotTargetConfig = simObj.robot.model1.ikcon(robotTargetPose,simObj.robot.model1.getpos());
        
    elseif controlRobot2 == true %if UR3_2 is selected
        robotTargetConfig = simObj.robot.model2.ikcon(robotTargetPose,simObj.robot.model2.getpos());
    end
    
     % Checks that joint configuration is within joint limits
    for i = 1:6
        if 2*pi < robotTargetConfig(i) || robotTargetConfig(i) <= -2*pi
            error("Angle for joint %d is not within the joint limits!",i);
        end
    end
    
    % Set joint sliders to correct value
    set(handles.J1Slider,'Value',rad2deg(robotTargetConfig(1)));
    set(handles.J2Slider,'Value',rad2deg(robotTargetConfig(2)));
    set(handles.J3Slider,'Value',rad2deg(robotTargetConfig(3)));
    set(handles.J4Slider,'Value',rad2deg(robotTargetConfig(4)));
    set(handles.J5Slider,'Value',rad2deg(robotTargetConfig(5)));
    set(handles.J6Slider,'Value',rad2deg(robotTargetConfig(6)));
    
    % Set joint edit boxes to correct value
    set(handles.J1EditBox,'String',rad2deg(robotTargetConfig(1)));
    set(handles.J2EditBox,'String',rad2deg(robotTargetConfig(2)));
    set(handles.J3EditBox,'String',rad2deg(robotTargetConfig(3)));
    set(handles.J4EditBox,'String',rad2deg(robotTargetConfig(4)));
    set(handles.J5EditBox,'String',rad2deg(robotTargetConfig(5)));
    set(handles.J6EditBox,'String',rad2deg(robotTargetConfig(6)));

% --- Executes during object creation, after setting all properties.
function PitchEditBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PitchEditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


function YawEditBox_Callback(hObject, eventdata, handles)
% hObject    handle to YawEditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    % Get XYZ and RPY values from edit boxes
    xValue = str2double(get(handles.XEditBox,'String')); 
    yValue = str2double(get(handles.YEditBox,'String'));
    zValue = str2double(get(handles.ZEditBox,'String'));
    
    rollValue = str2double(get(handles.RollEditBox,'String'));
    pitchValue = str2double(get(handles.PitchEditBox,'String'));
    yawValue = str2double(get(hObject,'String'));
    
    % Determine inverse kinemeatics based on values obtained
    rotationTr = rpy2tr(rollValue,pitchValue,yawValue);
    robotTargetPose = transl(xValue,yValue,zValue) * rotationTr;
       
    if controlRobot1 == true %if UR3_1 is selected
        robotTargetConfig = simObj.robot.model1.ikcon(robotTargetPose,simObj.robot.model1.getpos());
        
    elseif controlRobot2 == true %if UR3_2 is selected
        robotTargetConfig = simObj.robot.model2.ikcon(robotTargetPose,simObj.robot.model2.getpos());
    end
    
    % Checks that joint configuration is within joint limits
    for i = 1:6
        if 2*pi < robotTargetConfig(i) || robotTargetConfig(i) <= -2*pi
            error("Angle for joint %d is not within the joint limits!",i);
        end
    end
    
    % Set joint sliders to correct value
    set(handles.J1Slider,'Value',rad2deg(robotTargetConfig(1)));
    set(handles.J2Slider,'Value',rad2deg(robotTargetConfig(2)));
    set(handles.J3Slider,'Value',rad2deg(robotTargetConfig(3)));
    set(handles.J4Slider,'Value',rad2deg(robotTargetConfig(4)));
    set(handles.J5Slider,'Value',rad2deg(robotTargetConfig(5)));
    set(handles.J6Slider,'Value',rad2deg(robotTargetConfig(6)));
    
    % Set joint edit boxes to correct value
    set(handles.J1EditBox,'String',rad2deg(robotTargetConfig(1)));
    set(handles.J2EditBox,'String',rad2deg(robotTargetConfig(2)));
    set(handles.J3EditBox,'String',rad2deg(robotTargetConfig(3)));
    set(handles.J4EditBox,'String',rad2deg(robotTargetConfig(4)));
    set(handles.J5EditBox,'String',rad2deg(robotTargetConfig(5)));
    set(handles.J6EditBox,'String',rad2deg(robotTargetConfig(6)));

% --- Executes during object creation, after setting all properties.
function YawEditBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to YawEditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on slider movement.
function J1Slider_Callback(hObject, eventdata, handles)
% hObject    handle to J6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    % Get the current slider value and display it in respective edit box
    sliderValue = get(hObject,'Value');
    set(handles.J1EditBox,'String',sliderValue);
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    eStopStatus = get(handles.EStop,'String'); % check if Estop is pressed
    checkCollision = get(handles.SafetyStatus,'String'); % check for light curtain collision
    
    % Based on slider value update robot joint 1 angle and plot
    % if UR3_1 is selected -- checks if Estop is pressed and for collision with light curtain
    if controlRobot1 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
        robotCurrentConfig = simObj.robot.model1.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(1) = deg2rad(sliderValue);
        
        % Update edit boxes for XYZ and RPY
        robotCurrentPose = simObj.robot.model1.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
        simObj.robot.model1.animate(robotTargetConfig);
    
    % if UR3_2 is selected -- checks if Estop is pressed and for collision with light curtain
    elseif controlRobot2 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
        robotCurrentConfig = simObj.robot.model2.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(1) = deg2rad(sliderValue);
        
        % Update edit boxes for XYZ and RPY
        robotCurrentPose = simObj.robot.model2.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
        simObj.robot.model2.animate(robotTargetConfig);
    end
    
% --- Executes during object creation, after setting all properties.
function J1Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to J6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
    sliderValue = rad2deg(pi/2); %assigns initial joint 1 angle
    set(hObject,'Value',sliderValue);

    
% --- Executes on slider movement.
function J2Slider_Callback(hObject, eventdata, handles)
% hObject    handle to J6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    % Get the current slider value and display it in respective edit box
    sliderValue = get(hObject,'Value'); 
    set(handles.J2EditBox,'String',sliderValue);
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    eStopStatus = get(handles.EStop,'String'); % check if Estop is pressed
    checkCollision = get(handles.SafetyStatus,'String'); % check for light curtain collision
    
    % Based on slider value update robot joint 2 angle and plot
    % if UR3_1 is selected -- checks if Estop is pressed and for collision with light curtain
    if controlRobot1 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
        robotCurrentConfig = simObj.robot.model1.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(2) = deg2rad(sliderValue);
        
        % Update edit boxes for XYZ and RPY
        robotCurrentPose = simObj.robot.model1.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
        simObj.robot.model1.animate(robotTargetConfig);
        
    % if UR3_2 is selected -- checks if Estop is pressed and for collision with light curtain
    elseif controlRobot2 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
        robotCurrentConfig = simObj.robot.model2.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(2) = deg2rad(sliderValue);
        
        % Update edit boxes for XYZ and RPY
        robotCurrentPose = simObj.robot.model2.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
        simObj.robot.model2.animate(robotTargetConfig);
    end
    
% --- Executes during object creation, after setting all properties.
function J2Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to J6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
    sliderValue = rad2deg(-pi/2); %set initial joint 2 angle
    set(hObject,'Value',sliderValue);

    
% --- Executes on slider movement.
function J3Slider_Callback(hObject, eventdata, handles)
% hObject    handle to J6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    % Get the current slider value and display it in respective edit box
    sliderValue = get(hObject,'Value');
    set(handles.J3EditBox,'String',sliderValue);
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    eStopStatus = get(handles.EStop,'String'); % check if Estop is pressed
        checkCollision = get(handles.SafetyStatus,'String'); % check for light curtain collision
    
    % Based on slider value update robot joint 3 angle and plot
    % if UR3_1 is selected -- checks if Estop is pressed and for collision with light curtain
    if controlRobot1 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
        robotCurrentConfig = simObj.robot.model1.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(3) = deg2rad(sliderValue);
        
        % Update edit boxes for XYZ and RPY
        robotCurrentPose = simObj.robot.model1.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
        simObj.robot.model1.animate(robotTargetConfig);
        
    % if UR3_2 is selected -- checks if Estop is pressed and for collision with light curtain
    elseif controlRobot2 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
        robotCurrentConfig = simObj.robot.model2.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(3) = deg2rad(sliderValue);
        
        % Update edit boxes for XYZ and RPY
        robotCurrentPose = simObj.robot.model2.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
        simObj.robot.model2.animate(robotTargetConfig);
    end

% --- Executes during object creation, after setting all properties.
function J3Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to J6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
    sliderValue = 0; %set initial joint 3 angle
    set(hObject,'Value',sliderValue);

    
% --- Executes on slider movement.
function J4Slider_Callback(hObject, eventdata, handles)
% hObject    handle to J6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    % Get the current slider value and display it in respective edit box
    sliderValue = get(hObject,'Value');   
    set(handles.J4EditBox,'String',sliderValue);
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    eStopStatus = get(handles.EStop,'String'); % check if Estop is pressed
    checkCollision = get(handles.SafetyStatus,'String'); % check for light curtain collision
        
    % Based on slider value update robot joint 4 angle and plot
    % if UR3_1 is selected -- checks if Estop is pressed and for collision with light curtain
    if controlRobot1 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
        robotCurrentConfig = simObj.robot.model1.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(4) = deg2rad(sliderValue);
        
        % Update edit boxes for XYZ and RPY
        robotCurrentPose = simObj.robot.model1.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
        simObj.robot.model1.animate(robotTargetConfig);
        
    % if UR3_2 is selected -- checks if Estop is pressed and for collision with light curtain
    elseif controlRobot2 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
        robotCurrentConfig = simObj.robot.model2.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(4) = deg2rad(sliderValue);
        
        % Update edit boxes for XYZ and RPY
        robotCurrentPose = simObj.robot.model2.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
        simObj.robot.model2.animate(robotTargetConfig);
    end

% --- Executes during object creation, after setting all properties.
function J4Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to J6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
    sliderValue = rad2deg(-pi/2); % sets initial joint 4 angle
    set(hObject,'Value',sliderValue);

    
% --- Executes on slider movement.
function J5Slider_Callback(hObject, eventdata, handles)
% hObject    handle to J6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    % Get the current slider value and display it in respective edit box
    sliderValue = get(hObject,'Value');   
    set(handles.J5EditBox,'String',sliderValue);
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    eStopStatus = get(handles.EStop,'String'); % check if Estop is pressed
    checkCollision = get(handles.SafetyStatus,'String'); % check for light curtain collision
    
    % Based on slider value update robot joint 5 angle and plot
    % if UR3_1 is selected -- checks if Estop is pressed and for collision with light curtain
    if controlRobot1 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
        robotCurrentConfig = simObj.robot.model1.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(5) = deg2rad(sliderValue);
        
        % Update edit boxes for XYZ and RPY
        robotCurrentPose = simObj.robot.model1.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
        simObj.robot.model1.animate(robotTargetConfig);
        
    % if UR3_2 is selected -- checks if Estop is pressed and for collision with light curtain
    elseif controlRobot2 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
        robotCurrentConfig = simObj.robot.model2.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(5) = deg2rad(sliderValue);
        
        % Update edit boxes for XYZ and RPY
        robotCurrentPose = simObj.robot.model2.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
        simObj.robot.model2.animate(robotTargetConfig);
    end
    
% --- Executes during object creation, after setting all properties.
function J5Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to J6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
    sliderValue = 0; %sets initial joint 5 angle
    set(hObject,'Value',sliderValue);

    
% --- Executes on slider movement.
function J6Slider_Callback(hObject, eventdata, handles)
% hObject    handle to J6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    % Get the current slider value and display it in respective edit box
    sliderValue = get(hObject,'Value');
    set(handles.J6EditBox,'String',sliderValue);
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    eStopStatus = get(handles.EStop,'String'); % check if Estop is pressed
    checkCollision = get(handles.SafetyStatus,'String'); % check for light curtain collision
    
    % Based on slider value update robot joint 6 angle and plot
    % if UR3_1 is selected -- checks if Estop is pressed and for collision with light curtain
    if controlRobot1 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
        robotCurrentConfig = simObj.robot.model1.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(6) = deg2rad(sliderValue);
        
        % Update edit boxes for XYZ and RPY
        robotCurrentPose = simObj.robot.model1.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
        simObj.robot.model1.animate(robotTargetConfig);
        
    % if UR3_2 is selected -- checks if Estop is pressed and for collision with light curtain
    elseif controlRobot2 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
        robotCurrentConfig = simObj.robot.model2.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(6) = deg2rad(sliderValue);
        
        % Update edit boxes for XYZ and RPY
        robotCurrentPose = simObj.robot.model2.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
        simObj.robot.model2.animate(robotTargetConfig);
    end

% --- Executes during object creation, after setting all properties.
function J6Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to J6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
    sliderValue = 0; % sets initial joint 6 angle
    set(hObject,'Value',sliderValue);


% --- Executes on button press in LiveTeach.
function LiveTeach_Callback(hObject, eventdata, handles)
% hObject    handle to LiveTeach (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    eStopStatus = get(handles.EStop,'String'); % check if Estop is pressed
    checkCollision = get(handles.SafetyStatus,'String'); % check for light curtain collision
        
    % checks if Estop is pressed and for collision with light curtain
    if eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
        
        %Determine the joint angles input by the user in the edit boxes
        j1Value = deg2rad(str2double(get(handles.J1EditBox,'String')));
        j2Value = deg2rad(str2double(get(handles.J2EditBox,'String')));
        j3Value = deg2rad(str2double(get(handles.J3EditBox,'String')));
        j4Value = deg2rad(str2double(get(handles.J4EditBox,'String')));
        j5Value = deg2rad(str2double(get(handles.J5EditBox,'String')));
        j6Value = deg2rad(str2double(get(handles.J6EditBox,'String')));

        % Assigns target configuration based on user input
        robotTargetConfig = [j1Value,j2Value,j3Value,j4Value,j5Value,j6Value];

        ur_move_joint_client = rossvcclient('/ur_move');
        ur_move_joint_msg = rosmessage(ur_move_joint_client);
        ur_move_joint_msg.Q = robotTargetConfig; % Set joint state values
        ur_move_joint_client.call(ur_move_joint_msg); % Send values to robot
    end
    
% --- Executes on button press in SimulateTeach.
function SimulateTeach_Callback(hObject, eventdata, handles)
% hObject    handle to SimulateTeach (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    steps = 50;
    
    %Determine the joint angles input by the user in the edit boxes
    j1Value = deg2rad(str2double(get(handles.J1EditBox,'String')));
    j2Value = deg2rad(str2double(get(handles.J2EditBox,'String')));
    j3Value = deg2rad(str2double(get(handles.J3EditBox,'String')));
    j4Value = deg2rad(str2double(get(handles.J4EditBox,'String')));
    j5Value = deg2rad(str2double(get(handles.J5EditBox,'String')));
    j6Value = deg2rad(str2double(get(handles.J6EditBox,'String')));
    
    % Determine the selected robot's current joint configuration
    if controlRobot1 == true %if UR3_1 is selected
        robotCurrentConfig = simObj.robot.model1.getpos();
        
    elseif controlRobot2 == true %if UR3_2 is selected
        robotCurrentConfig = simObj.robot.model2.getpos();
    end
    
    % Assigns target configuration based on user input
    robotTargetConfig = [j1Value,j2Value,j3Value,j4Value,j5Value,j6Value];
    
    % Creates a matrix of interpolated joint configurations
    s = lspb(0,1,steps);
    qMatrix = nan(steps,6);
    for i = 1:1:steps
        qMatrix(i,:) = (1-s(i))*robotCurrentConfig + s(i)*robotTargetConfig;
    end
    
    % Plots the calculated matrix
    for i = 1:1:steps
        eStopStatus = get(handles.EStop,'String'); % check if Estop is pressed
        checkCollision = get(handles.SafetyStatus,'String'); % check for light curtain collision
        
        % if UR3_1 is selected -- checks if Estop is pressed and for collision with light curtain
        if controlRobot1 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
            simObj.robot.model1.animate(qMatrix(i,:));
            drawnow();

        % if UR3_2 is selected -- checks if Estop is pressed and for collision with light curtain
        elseif controlRobot2 == true && eStopStatus == "Emergency Stop" && checkCollision == "Safety Status: Safe"
            simObj.robot.model2.animate(qMatrix(i,:));
            drawnow();
            
        % If Estop is pressed or light curtain is triggered then stop robot movement
        elseif eStopStatus == "Press to Release" || checkCollision == "Safety Status: Unsafe! Unexpected object in workspace"...
                                                 || checkCollision == "Safety Status: Unexpected object in robot trajectory!"
            break;
        end
    end
    
    % Update joint sliders/input boxes and XYZ/RPY input boxes
    if controlRobot1 == true
        ControlUR3_1_Callback(handles.ControlUR3_1, eventdata,handles);
        
    elseif controlRobot2 == true
        ControlUR3_2_Callback(handles.ControlUR3_2, eventdata,handles);
    end

function J1EditBox_Callback(hObject, eventdata, handles)
% hObject    handle to J6EditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    % Get the current edit box value and adjusts slider accordingly
    editBoxValue = str2double(get(hObject,'String'));
    set(handles.J1Slider,'Value',editBoxValue);
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    % Based on edit box value update XYZ and RPY values
    if controlRobot1 == true % if UR3_1 is selected
        robotCurrentConfig = simObj.robot.model1.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(1) = deg2rad(editBoxValue);
        
        robotCurrentPose = simObj.robot.model1.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
    elseif controlRobot2 == true % if UR3_2 is selected
        robotCurrentConfig = simObj.robot.model2.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(1) = deg2rad(editBoxValue);
        
        robotCurrentPose = simObj.robot.model2.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
    end
    
% --- Executes during object creation, after setting all properties.
function J1EditBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to J6EditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


function J2EditBox_Callback(hObject, eventdata, handles)
% hObject    handle to J6EditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    % Get the current edit box value and adjusts slider accordingly
    editBoxValue = str2double(get(hObject,'String'));
    set(handles.J2Slider,'Value',editBoxValue);
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    % Based on edit box value update XYZ and RPY values
    if controlRobot1 == true % if UR3_1 is selected
        robotCurrentConfig = simObj.robot.model1.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(2) = deg2rad(editBoxValue);
        
        robotCurrentPose = simObj.robot.model1.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
    elseif controlRobot2 == true % if UR3_2 is selected
        robotCurrentConfig = simObj.robot.model2.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(2) = deg2rad(editBoxValue);
        
        robotCurrentPose = simObj.robot.model2.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
    end

% --- Executes during object creation, after setting all properties.
function J2EditBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to J6EditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


function J3EditBox_Callback(hObject, eventdata, handles)
% hObject    handle to J6EditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    % Get the current edit box value and adjusts slider accordingly
    editBoxValue = str2double(get(hObject,'String'));
    set(handles.J3Slider,'Value',editBoxValue);
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    % Based on edit box value update XYZ and RPY values
    if controlRobot1 == true % if UR3_1 is selected
        robotCurrentConfig = simObj.robot.model1.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(3) = deg2rad(editBoxValue);
        
        robotCurrentPose = simObj.robot.model1.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
    elseif controlRobot2 == true % if UR3_2 is selected
        robotCurrentConfig = simObj.robot.model2.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(3) = deg2rad(editBoxValue);
        
        robotCurrentPose = simObj.robot.model2.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
    end
    
% --- Executes during object creation, after setting all properties.
function J3EditBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to J6EditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


function J4EditBox_Callback(hObject, eventdata, handles)
% hObject    handle to J6EditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    % Get the current edit box value and adjusts slider accordingly
    editBoxValue = str2double(get(hObject,'String'));  
    set(handles.J4Slider,'Value',editBoxValue);
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    % Based on edit box value update XYZ and RPY values
    if controlRobot1 == true % if UR3_1 is selected
        robotCurrentConfig = simObj.robot.model1.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(4) = deg2rad(editBoxValue);
        
        robotCurrentPose = simObj.robot.model1.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
    elseif controlRobot2 == true % if UR3_2 is selected
        robotCurrentConfig = simObj.robot.model2.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(4) = deg2rad(editBoxValue);
        
        robotCurrentPose = simObj.robot.model2.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
    end
    
% --- Executes during object creation, after setting all properties.
function J4EditBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to J6EditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
   
    
function J5EditBox_Callback(hObject, eventdata, handles)
% hObject    handle to J6EditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    % Get the current edit box value and adjusts slider accordingly
    editBoxValue = str2double(get(hObject,'String'));
    set(handles.J5Slider,'Value',editBoxValue);
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    % Based on edit box value update XYZ and RPY values
    if controlRobot1 == true  % if UR3_1 is selected
        robotCurrentConfig = simObj.robot.model1.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(5) = deg2rad(editBoxValue);
        
        robotCurrentPose = simObj.robot.model1.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
    elseif controlRobot2 == true  % if UR3_2 is selected
        robotCurrentConfig = simObj.robot.model2.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(5) = deg2rad(editBoxValue);
        
        robotCurrentPose = simObj.robot.model2.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
    end
    
% --- Executes during object creation, after setting all properties.
function J5EditBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to J6EditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    
    
function J6EditBox_Callback(hObject, eventdata, handles)
% hObject    handle to J6EditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    % Get the current edit box value and adjusts slider accordingly
    editBoxValue = str2double(get(hObject,'String'));
    set(handles.J6Slider,'Value',editBoxValue);
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    controlRobot1 = get(handles.ControlUR3_1,'Value'); 
    controlRobot2 = get(handles.ControlUR3_2,'Value');
    
    % Based on edit box value update XYZ and RPY values
    if controlRobot1 == true  % if UR3_1 is selected
        robotCurrentConfig = simObj.robot.model1.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(6) = deg2rad(editBoxValue);
        
        robotCurrentPose = simObj.robot.model1.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
        
    elseif controlRobot2 == true  % if UR3_2 is selected
        robotCurrentConfig = simObj.robot.model2.getpos();
        robotTargetConfig = robotCurrentConfig;
        robotTargetConfig(6) = deg2rad(editBoxValue);
        
        robotCurrentPose = simObj.robot.model2.fkine(robotTargetConfig);
        rpyAngles = tr2rpy(robotCurrentPose);
        
        set(handles.XEditBox,'String',num2str(robotCurrentPose(1,4)));
        set(handles.YEditBox,'String',num2str(robotCurrentPose(2,4)));
        set(handles.ZEditBox,'String',num2str(robotCurrentPose(3,4)));
        set(handles.RollEditBox,'String',num2str(rad2deg(rpyAngles(1))));
        set(handles.PitchEditBox,'String',num2str(rad2deg(rpyAngles(2))));
        set(handles.YawEditBox,'String',num2str(rad2deg(rpyAngles(3))));
    end
    
% --- Executes during object creation, after setting all properties.
function J6EditBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to J6EditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in ControlUR3_1.
function ControlUR3_1_Callback(hObject, eventdata, handles)
% hObject    handle to ControlUR3_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    
    % Get UR3_1 current joint configuration and calculates forward kinematics
    robotCurrentConfig = simObj.robot.model1.getpos();
    robotCurrentPose = simObj.robot.model1.fkine(robotCurrentConfig);
    rpyAngles = tr2rpy(robotCurrentPose);
    
    % Assign slider value for each joint based on current joint configuration
    set(handles.J1Slider,'Value',rad2deg(robotCurrentConfig(1)));
    set(handles.J2Slider,'Value',rad2deg(robotCurrentConfig(2)));
    set(handles.J3Slider,'Value',rad2deg(robotCurrentConfig(3)));
    set(handles.J4Slider,'Value',rad2deg(robotCurrentConfig(4)));
    set(handles.J5Slider,'Value',rad2deg(robotCurrentConfig(5)));
    set(handles.J6Slider,'Value',rad2deg(robotCurrentConfig(6)));
    
    % Assign edit box value for each joint based on current joint configuration
    set(handles.J1EditBox,'String',rad2deg(robotCurrentConfig(1)));
    set(handles.J2EditBox,'String',rad2deg(robotCurrentConfig(2)));
    set(handles.J3EditBox,'String',rad2deg(robotCurrentConfig(3)));
    set(handles.J4EditBox,'String',rad2deg(robotCurrentConfig(4)));
    set(handles.J5EditBox,'String',rad2deg(robotCurrentConfig(5)));
    set(handles.J6EditBox,'String',rad2deg(robotCurrentConfig(6)));
    
    % Assign Cartesian coordinates for each joint based forward kinematics calcs
    set(handles.XEditBox,'String',robotCurrentPose(1,4));
    set(handles.YEditBox,'String',robotCurrentPose(2,4));
    set(handles.ZEditBox,'String',robotCurrentPose(3,4));
    set(handles.RollEditBox,'String',rad2deg(rpyAngles(1)));
    set(handles.PitchEditBox,'String',rad2deg(rpyAngles(2)));
    set(handles.YawEditBox,'String',rad2deg(rpyAngles(3)));
    
    if (get(handles.CheckForJoystickControl,'Value')) == 1
        duration = 0; 
        robotNumber = 2;
        simObj.JoystickControl(robotNumber,duration);
    end


% --- Executes on button press in ControlUR3_2.
function ControlUR3_2_Callback(hObject, eventdata, handles)
% hObject    handle to ControlUR3_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    
    % Get UR3_2 current joint configuration and calculates forward kinematics  
    robotCurrentConfig = simObj.robot.model2.getpos();
    robotCurrentPose = simObj.robot.model2.fkine(robotCurrentConfig);
    rpyAngles = tr2rpy(robotCurrentPose);
    
    % Assign slider value for each joint based on current joint configuration
    set(handles.J1Slider,'Value',rad2deg(robotCurrentConfig(1)));
    set(handles.J2Slider,'Value',rad2deg(robotCurrentConfig(2)));
    set(handles.J3Slider,'Value',rad2deg(robotCurrentConfig(3)));
    set(handles.J4Slider,'Value',rad2deg(robotCurrentConfig(4)));
    set(handles.J5Slider,'Value',rad2deg(robotCurrentConfig(5)));
    set(handles.J6Slider,'Value',rad2deg(robotCurrentConfig(6)));
    
    % Assign edit box value for each joint based on current joint configuration
    set(handles.J1EditBox,'String',rad2deg(robotCurrentConfig(1)));
    set(handles.J2EditBox,'String',rad2deg(robotCurrentConfig(2)));
    set(handles.J3EditBox,'String',rad2deg(robotCurrentConfig(3)));
    set(handles.J4EditBox,'String',rad2deg(robotCurrentConfig(4)));
    set(handles.J5EditBox,'String',rad2deg(robotCurrentConfig(5)));
    set(handles.J6EditBox,'String',rad2deg(robotCurrentConfig(6)));
    
    % Assign Cartesian coordinates for each joint based forward kinematics calcs
    set(handles.XEditBox,'String',robotCurrentPose(1,4));
    set(handles.YEditBox,'String',robotCurrentPose(2,4));
    set(handles.ZEditBox,'String',robotCurrentPose(3,4));
    set(handles.RollEditBox,'String',rad2deg(rpyAngles(1)));
    set(handles.PitchEditBox,'String',rad2deg(rpyAngles(2)));
    set(handles.YawEditBox,'String',rad2deg(rpyAngles(3)));
    
    if (get(handles.CheckForJoystickControl,'Value')) == 1
        duration = 0; 
        robotNumber = 1;
        simObj.JoystickControl(robotNumber,duration);
    end


% --- Executes on button press in DetectBlue.
function DetectBlue_Callback(hObject, eventdata, handles)
% hObject    handle to DetectBlue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    
    if (get(hObject,'Value')) == 1
        simObj.ColourDetectionPublisher(0); % 0 is assigned to blue in DetectColour function
    end
        

% --- Executes on button press in DetectGreen.
function DetectGreen_Callback(hObject, eventdata, handles)
% hObject    handle to DetectGreen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    
    if (get(hObject,'Value')) == 1
        simObj.ColourDetectionPublisher(1); %  1 is assigned to green in DetectColour function
    end

% --- Executes on slider movement -- for moving object
function LightCurtainSensor_Callback(hObject, eventdata, handles)
% hObject    handle to LightCurtainSensor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    safetyObj = handles.safetyObj; % Save the Safety class object as a field within handles
    
    collidingObjectInitialPos = [-1.5,0,0.8]; % origin: cube centre -- cube dim: 0.2x0.2x0.2
    
    checkCollision = false;
    
    eStopStatus = get(handles.EStop,'String');
    
    if checkCollision == false && eStopStatus == "Emergency Stop"

        % In this case, the light curtain normal to the Y axis on the +ve side is checked for collision
        sliderValue = get(hObject,'Value'); %object only moving along y axis (x and z stay the same)

        collidingObjectPos = collidingObjectInitialPos;
        collidingObjectPos(1) = collidingObjectInitialPos(1) + sliderValue; %update object y coordinate
        collidingObject = PlaceObject('collidingobject.ply',collidingObjectPos);

        %Line extends between two parallel faces
        point1OnLine = [(collidingObjectPos(1)-0.1),collidingObjectPos(2),collidingObjectPos(3)];
        point2OnLine = [(collidingObjectPos(1)+0.1),collidingObjectPos(2),collidingObjectPos(3)];

        %check for collision on any part of the light curtain and return message
        checkCollision = safetyObj.CollisionDetection(point1OnLine,point2OnLine);
    
        if checkCollision == true
            safetyStatus = 'Safety Status: Unsafe! Unexpected object in workspace';
            set(handles.SafetyStatus,'String',safetyStatus,'ForegroundColor','red');
            
        else
            safetyStatus = 'Safety Status: Safe';
            set(handles.SafetyStatus,'String',safetyStatus,'ForegroundColor',[0.392 0.831 0.075]);    
        end
        pause(2);
        try delete(collidingObject); end % delete object to prevent trail
        
    elseif  checkCollision == false && eStopStatus == "Press to Release"
        safetyStatus = 'Safety Status: E-Stop Pressed!';
        set(handles.SafetyStatus,'String',safetyStatus,'ForegroundColor','red');
        set(hObject,'Enable','off'); % turn off 'Moving Object' slider (reenabled in EStop callback)
    end
    
    
% --- Executes during object creation, after setting all properties.
function LightCurtainSensor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LightCurtainSensor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
    end


% --- Executes on button press in CheckForRealRobot.
function CheckForRealRobot_Callback(hObject, eventdata, handles)
% hObject    handle to CheckForRealRobot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in CheckForJoystickControl.
function CheckForJoystickControl_Callback(hObject, eventdata, handles)
% hObject    handle to CheckForJoystickControl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    simObj = handles.simObj;
    
    if (get(hObject,'Value')) == 1
        duration = 90; % assigns joystick control runtime

        % Checks which robot is selected to control
        if (get(handles.ControlUR3_1,'Value')) == 1
            robotNumber = 1;
            simObj.JoystickControl(robotNumber,duration);

        elseif (get(handles.ControlUR3_2,'Value')) == 1
            robotNumber = 2;
            simObj.JoystickControl(robotNumber,duration);
        
        end
    
    %if joystick control is turned off or safety feature is triggered stop
    %control
    elseif (get(hObject,'Value')) == 0 || eStopStatus == "Press to Release" ...
           || checkCollision == "Safety Status: Unsafe! Unexpected object in workspace"...
           || checkCollision == "Safety Status: Unexpected object in robot trajectory!"
       
        duration = 0;

        if (get(handles.ControlUR3_1,'Value')) == 1  
            robotNumber = 1;
            simObj.JoystickControl(robotNumber,duration);

        elseif (get(handles.ControlUR3_2,'Value')) == 1
            robotNumber = 2;
            simObj.JoystickControl(robotNumber,duration);
        end
    end


% --- Executes on button press in CheckForObjectInTrajectory.
function CheckForObjectInTrajectory_Callback(hObject, eventdata, handles)
% hObject    handle to CheckForObjectInTrajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    simObj = handles.simObj; % Save the Lets Play Catch class object as a field within handles
    safetyObj = handles.safetyObj;
    
    robot = simObj.robot.model1;
    
    
    [objectInTrajectory,plotCube] = safetyObj.CollisionAvoidance(robot);
    
    if objectInTrajectory == true
        safetyStatus = 'Safety Status: Unexpected object in robot trajectory!';
        set(handles.SafetyStatus,'String',safetyStatus,'ForegroundColor','red');
    end
    
    % waits for 5secs and deletes cube
    pause(5);
    try delete(plotCube); end 
    
    safetyStatus = 'Safety Status: Safe';
    set(handles.SafetyStatus,'String',safetyStatus,'ForegroundColor',[0.392 0.831 0.075]);
    
        
        
