function varargout = Fin170(varargin)
% FIN170 MATLAB code for Fin170.fig
%      FIN170, by itself, creates a new FIN170 or raises the existing
%      singleton*.
%
%      H = FIN170 returns the handle to a new FIN170 or the handle to
%      the existing singleton*.
%
%      FIN170('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FIN170.M with the given input arguments.
%
%      FIN170('Property','Value',...) creates a new FIN170 or raises the
%      existing singleton*.  Starting from the camleft, property value pairs are
%      applied to the GUI before Fin170_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      thruster.  All inputs are passed to Fin170_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Fin170

% Last Modified by GUIDE v2.5 03-Mar-2021 11:30:52

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Fin170_OpeningFcn, ...
                   'gui_OutputFcn',  @Fin170_OutputFcn, ...
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


% --- Executes just before Fin170 is made visible.
function Fin170_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Fin170 (see VARARGIN)

axes(handles.Camera);
vid=videoinput('winvideo', 1);
hImage=image(zeros(720, 1280, 3), 'Parent', handles.Camera);
preview(vid,hImage);
camorbit(180,180);
axis off;

axes(handles.capturedpic);
axis off;
axes(handles.boatpic);
image(imread('boat.png'));
axis off;
axes(handles.IMU1axes);
axis off;
axes(handles.IMU2axes);
image(imread('rpw.png'));
axis off;
axes(handles.gpsmap);
image(imread('gpss.png'));
axis off;
axes(handles.batpercent);
image(imread('0.png'));
axis off;

datern = datestr(now, 'mm-dd-yyyy');
myString = sprintf('%s', datern);
handles.date.String = myString;
        
% 
% s = sprintf('RIGHT');
% set(handles.camright, 'TooltipString', s)
% s = sprintf('LEFT');
% set(handles.camleft, 'TooltipString', s)
% s = sprintf('CENTER');
% set(handles.center, 'TooltipString', s)
% s = sprintf('FORWARD/STOP');
% set(handles.thruster, 'TooltipString', s)


% Choose default command line output for Fin170
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Fin170 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Fin170_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in arduino.
function arduino_Callback(hObject, eventdata, handles)
% hObject    handle to arduino (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of arduino
ispushed= get(hObject, 'Value');
if ispushed 
    
    set(hObject, 'String', 'STOP');
    set(handles.sysstatus, 'String', 'ON');
    set(handles.arduino,'BackgroundColor',[1 0 0]);
%     set(handles.sysstatus, 'backgroundcolor', 'g');
    if ~isempty(instrfind)
        fclose(instrfind);
        delete(instrfind);
    end
    
    global GPS_data
    global IMU_data
    global FEEDBACK_data
    global VSCS_data
    global xx;
    ports = instrhwinfo('serial');
    port = ports.AvailableSerialPorts;
    comPort = char(port);
    xx = serial(comPort,'BAUD', 9600);
%     disp(gg)
%     disp(ggg)
%% Initializing for 3D plot for IMU Visualization
 hold on
A = [0 0 0];
B = [1 0 0];
C = [0 1 0];
D = [0 0 1];
E = [0 1 1];
F = [1 0 1];
G = [1 1 0];
H = [1 1 1];
D(3)=D(3)*0.5;
F(3)=F(3)*0.5;
E(3)=E(3)*0.5;
H(3)=H(3)*0.5;
A(1)=A(1)+0.2;
D(1)=D(1)+0.2;
F(1)=F(1)-0.2;
B(1)=B(1)-0.2;
Center=H/2;
Displacement=[0 0 1];
axes(handles.IMU2axes);
plotGraph3d = plot3(0,0,0,'-r')

%% Initialization for the graph for IMU
    plotTitle = 'Arduino Data Log';  % plot title
    xLabel = 'Elapsed Time (s)';     % x-axis label
    yLabel = 'Rad (C)';      % y-axis label
    legend1 = 'Roll';
    legend2 = 'Pitch';
    legend3 = 'Yaw';
    x_max  = 30;                           %y Maximum Value
    x_min  = 0;                       %y minimum Value
    plotGrid = 'on';                 % 'off' to turn off grid
    y_min = -200;                         % set y-min
    y_max = 200;               
%%    
    fopen(xx);

    cnt1 = 1;
    cnt2 = 1;
    cnt3 = 1;
    cnt4 = 1;

   
    GPS_data = [];
    IMU_data = [];
    VSCS_data = [];
    FEEDBACK_data = [];
    IMU_data(cnt2,2) = cnt2;
    IMU_data(cnt2,3) = 0;
    IMU_data(cnt2,4) = 0;
    IMU_data(cnt2,5) = 0;
    
    axes(handles.IMU1axes);
    plotGraph = plot(IMU_data(cnt2,2),IMU_data(cnt2,3),'-r')  % every AnalogRead needs to be on its own Plotgraph
    hold on
    plotGraph1 = plot(IMU_data(cnt2,2),IMU_data(cnt2,4),'-b')  % every AnalogRead needs to be on its own Plotgraph
    plotGraph2 = plot(IMU_data(cnt2,2),IMU_data(cnt2,5),'-g')  % every AnalogRead needs to be on its own Plotgraph
    title(plotTitle,'FontSize',15);
    xlabel(xLabel,'FontSize',15);
    ylabel(yLabel,'FontSize',15);
    legend(legend1,legend2,legend3);
    axis([x_min x_max y_min y_max]);
    grid(plotGrid);

    gx = geoaxes(handles.GPS); %display map at panel
    
    while ispushed     
        serial_data = fscanf(xx);
        flushinput(xx);
        [tag, remain] = strtok(serial_data,',');
     disp(serial_data)
    
        switch tag

            case '$GPS'
                GPS = strsplit(remain , ',');
                o = datestr(now,'HHMMSSFFF');
                GPS_data(cnt1,1) = str2num(o);
                GPS_data(cnt1,2) = cnt1;
                GPS_data(cnt1,3) =str2double(GPS(1,2)); %Latitude
                GPS_data(cnt1,4) =str2double(GPS(1,3)); %Longitude
                GPS_data(cnt1,5) =str2double(GPS(1,4)); %Speed

                set(handles.longitude,'string',GPS_data(cnt1,3));
                set(handles.latitude,'string',GPS_data(cnt1,4));
                set(handles.gpsspeed,'string',GPS_data(cnt1,5));
                geoplot(gx, GPS_data(:,3), GPS_data(:,4),'g->');
                geobasemap(gx, 'satellite');
                cnt1= cnt1+1;

            case '$IMU'
                IMU = strsplit(remain , ',');
                n = datestr(now,'HHMMSSFFF');
                IMU_data(cnt2,1) = str2num(n);
                IMU_data(cnt2,2) = cnt2;
                IMU_data(cnt2,3) = str2double(IMU(1,2));%roll
                IMU_data(cnt2,4) = str2double(IMU(1,3));%pitch
                IMU_data(cnt2,5) = str2double(IMU(1,4));%yaw
                IMU_data(cnt2,6) = str2double(IMU(1,5));%heading
                IMU_data(cnt2,7) = str2double(IMU(1,6));%vs
                IMU_data(cnt2,8) = str2double(IMU(1,7));%cs
%                 IMU_data(cnt2,9) = str2double(IMU(1,8));%gyrox
%                 IMU_data(cnt2,10) = str2double(IMU(1,9));%gyroy
%                 IMU_data(cnt2,11) = str2double(IMU(1,10));%gyroz

                set(handles.pitch,'string',IMU_data(cnt2,3));
                set(handles.roll,'string',IMU_data(cnt2,4));
                set(handles.yaw,'string',IMU_data(cnt2,5));
                
               
                set(plotGraph,'XData',IMU_data(:,2),'YData',IMU_data(:,3));
                set(plotGraph1,'XData',IMU_data(:,2),'YData',IMU_data(:,4));
                set(plotGraph2,'XData',IMU_data(:,2),'YData',IMU_data(:,5));

                
                %% For moving axis   
                if mod(cnt2, 30) == 0 
                    x_min = x_min + 30;
                    x_max = x_max + 30;
                    
                    axes(handles.IMU1axes);
                    axis([x_min x_max y_min y_max]);
%                     flushinput(xx);
                end
                

                %%
                set(handles.longitude,'string',IMU_data(cnt2,7));
                set(handles.latitude,'string',IMU_data(cnt2,8));
                %plot(VSCS_data(:,2),VSCS_data(:,3),'g',VSCS_data(:,2),VSCS_data(:,4),'r')

                %%Battery Percentage approximation
                vl = IMU_data(cnt2,7);
                cu = IMU_data(cnt2,8);
                vi = 0.50*cu + vl; %0.5 the value of internal resistance
                vr = (12.60-vi)/(vi-9.0);
                cp = 100/(vr+1);

                if cu <= 0 && vl <= 0
                    cp = 0;
                end

                aa = cp;
                val = round(aa); %percentage of the battery


                set(handles.batper,'String',num2str(val));
                if val >= 75
                    batper = imread('100.png');
                    axes(handles.batpercent);
                    imshow(batper);
                elseif val < 75 && val >= 50
                    batper = imread('75.png');
                    axes(handles.batpercent);
                    imshow(batper);
                elseif val < 50 && val >= 20
                    batper = imread('50.png');
                    axes(handles.batpercent);
                    imshow(batper);
                elseif val < 20 && val >= 1
                    batper = imread('25.png');
                    axes(handles.batpercent);
                    imshow(batper);
                else
                    batper = imread('0.png');
                    axes(handles.batpercent);
                    imshow(batper);
%                     cnt3= cnt3+1;
                end
        

                dcm = angle2dcm(IMU_data(cnt2,5)/57.3,IMU_data(cnt2,3)/-57.3,IMU_data(cnt2,4)/57.3);
                P = [A;B;F;H;G;C;A;D;E;H;F;D;E;C;G;B];
                for i=1:length(P)
                   P(i,:)=P(i,:)-Center;
                end
                P = P*dcm;
                for i=1:length(P)
                   P(i,:)=P(i,:)+Center;
                end
                axes(handles.IMU2axes); %3D representaion
%                 plot3(P(:,1),P(:,2),P(:,3),'Color','r');
                set(plotGraph3d, 'XData', P(:,1), 'YData',P(:,2),'ZData',P(:,3));
%                 hold on;
                %%
%                 hold off;
                axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]);
% 
                view([1.5,1.5,1.5]) ;
                xlabel('X');
                ylabel('Y');
                zlabel('Z');
                
                %% draw plot
                drawnow;
                pause(0.6)
                cnt2 = cnt2+1;

        end

%          pause(0.5);
    end

 
else
    global GPS_data
    global IMU_data
    global FEEDBACK_data
    global VSCS_data
    set(hObject, 'String', 'START');
    set(handles.sysstatus, 'String', 'OFF');
    set(handles.arduino,'BackgroundColor',[0 1 0]);
%     set(handles.sysstatus, 'backgroundcolor', 'r');
    if ~isempty(instrfind)
         fclose(instrfind);
         delete(instrfind);
    end
         
    nodata1 = imread('nodata.png');
    axes(handles.IMU1axes);
    imshow(nodata1);
    nodata2 = imread('nodata.png');
    axes(handles.IMU2axes);
    imshow(nodata2);
    f = errordlg('Click START button to operate again.', 'System off!'); 
    y = datestr(now,'yyyymmddTHHMMSS');
    if ~isempty(instrfind)
        fclose(instrfind);
        delete(instrfind);
    end
   
    filename = 'GPS1.xlsx' ;  
    writematrix(GPS_data,filename,'Sheet',y);
    filename2 = 'IMU.xlsx' ;
    writematrix(IMU_data,filename2,'Sheet',y);
%     filename3 = 'FEEDBACK.xlsx'  ; 
%     writetable(FEEDBACK_data,filename3,'Sheet',1);
    filename4 = 'VoltageCurrent.xlsx'  ; 
    writematrix(VSCS_data,filename4,'Sheet',y);
    
end
guidata(hObject, handles);


% --- Executes on button press in Capture.
function Capture_Callback(hObject, eventdata, handles)
    % hObject    handle to Capture (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    fulldate = datestr(now, 'yyyymmddTHHMMSS');
    F = getframe(handles.Camera);
    Image = frame2im(F);
    savename= strcat ('C:\Users\Kate Augusto\Documents\EE\5th Year 1\EE 170\Final_EE170\Pictures\', num2str(fulldate), '.jpg');
    imwrite(Image, savename);
    imshow(Image, 'Parent', handles.capturedpic);


% --- Executes on button press in IMU1push.
function IMU1push_Callback(hObject, eventdata, handles)
    % hObject    handle to IMU1push (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    uistack(handles.IMU1,'top')
    set(handles.IMU1, 'Visible', 'on');
    set(handles.IMU2, 'Visible', 'off');

% --- Executes on button press in IMU2push.
function IMU2push_Callback(hObject, eventdata, handles)
    % hObject    handle to IMU2push (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    uistack(handles.IMU2,'top')
    set(handles.IMU2, 'Visible', 'on');
    set(handles.IMU1, 'Visible', 'off');


% --- Executes on slider movement.
function speed_Callback(hObject, eventdata, handles)
    % hObject    handle to speed (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    global xx;
    val = get(hObject,'Value');
    dval= num2str(round(val));
    set(handles.speedstat,'String',num2str(dval));
    disp(dval)
    
   if dval == '0'
       set(handles.thruster,'string','MOVE');
       
   else
       set(handles.thruster,'string','STOP');
   end
   
    switch dval 
        case '0'
            send = "0SS";
        case '1'
            send = "1WW";
        case '2'
            send = "2WW";
        case '3'
            send = "3WW";
        case '4' 
            send = '4WW';
        case '5'
            send = '5WW';
        case '6'
            send = '6WW';
        case '7'
            send = '7WW';
        case '8'
            send = '8WW';
        case '9' 
            send = '9WW';
          
    end

    fprintf(xx, send);
disp(send)




% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function speed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



% --- Executes on button press in camright.
function right_Callback(hObject, eventdata, handles)
% hObject    handle to camright (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xx;
send ='D';
fprintf(xx, send);

% --- Executes on button press in camleft.
function left_Callback(hObject, eventdata, handles)
% hObject    handle to camleft (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xx;
send ='A';
fprintf(xx, send);

% --- Executes on button press in center.
function center_Callback(hObject, eventdata, handles)
% hObject    handle to center (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global xx;
send ='Q';
fprintf(xx, send);


% --- Executes on button press in thruster.
function thruster_Callback(hObject, eventdata, handles)
% hObject    handle to thruster (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xx;
ispushed= get(hObject, 'Value');

if ispushed
    set(hObject, 'String', 'STOP');
%     x = 444;
%     y = 255;
%     set(handles.speed,'Value',255/2);
%     set(handles.speedstat,'String',50);
%     x = num2str(x);
%     y = num2str(y);
%     send =[ x,',',y];
    send = 'W';
    fprintf(xx, send);

else
    set(hObject, 'String', 'MOVE');
%     x = 444;
%     y = 0;
%     set(handles.speed,'Value',0);
%     set(handles.speedstat,'String',0);
%     x = num2str(x);
%     y = num2str(y);
%     send =[ x,',',y];
    send = 'S';
    fprintf(xx, send);   
end


% --- Executes on button press in conveyor.
function conveyor_Callback(hObject, eventdata, handles)
% hObject    handle to conveyor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global xx;
ispushed= get(hObject, 'Value');

if ispushed
   set(hObject, 'String', 'ON');
    send = 'C' ;
    fprintf(xx, send);
else
   set(hObject, 'String', 'OFF');
    send = 'V' ;
    fprintf(xx, send);
 
end
% Hint: get(hObject,'Value') returns toggle state of conveyor





% --- Executes on button press in pushbutton23.
function pushbutton23_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
f= msgbox({'For keyboard shortcuts:' ; '' ; 'Rudder Left = leftarrow'; 
  'Rudder Right = rightarrow' ; 'Rudder Center = uparrow' ; 'Move/Stop = downarrow'
  ; 'Camera Left = a'  ; 'Camera Center = s'  ; 'Camera Right = d'});



function keyshort_Callback(hObject, eventdata, handles)
% hObject    handle to keyshort (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of keyshort as text
%        str2double(get(hObject,'String')) returns contents of keyshort as a double


% --- Executes during object creation, after setting all properties.
function keyshort_CreateFcn(hObject, eventdata, handles)
% hObject    handle to keyshort (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key press with focus on keyshort and none of its controls.
function keyshort_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to keyshort (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

keyPressed = eventdata.Key;

switch keyPressed
    case 'uparrow'
        pushbutton1_Callback(handles.center,[],handles);
    case 'leftarrow'
        left_Callback(handles.left,[],handles); 
    case 'rightarrow'
        pushbutton1_Callback(handles.right,[],handles); 
    case 'downarrow'
        thruster_Callback(handles.thruster,[],handles);
    case 'a'
        pushbutton1_Callback(handles.camleft,[],handles); 
    case 's'
        pushbutton1_Callback(handles.camcenter,[],handles); 
    case 'd'
         pushbutton1_Callback(handles.camright,[],handles);
         
end       
 

% --- Executes on button press in IMUB.
function IMUB_Callback(hObject, eventdata, handles)
% hObject    handle to IMUB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
uistack(handles.IMU,'top')
set(handles.IMU, 'Visible', 'on');
set(handles.GPS, 'Visible', 'off');
set(handles.VSCS, 'Visible', 'off');
global xx;
send ='BMK';
fprintf(xx, send);
flushinput(xx);
% --- Executes on button press in GPSB.
function GPSB_Callback(hObject, eventdata, handles)
% hObject    handle to GPSB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
uistack(handles.GPS,'top')
set(handles.GPS, 'Visible', 'on');
set(handles.IMU, 'Visible', 'off');
set(handles.VSCS, 'Visible', 'off');
global xx;
send ='BL';
fprintf(xx, send);
flushinput(xx);
% --- Executes on button press in CSVSB.
function CSVSB_Callback(hObject, eventdata, handles)
% hObject    handle to CSVSB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
uistack(handles.VSCS,'top')
set(handles.VSCS, 'Visible', 'on');
set(handles.GPS, 'Visible', 'off');
set(handles.IMU, 'Visible', 'off');
global xx;
send ='MJ';
fprintf(xx, send);
flushinput(xx);
