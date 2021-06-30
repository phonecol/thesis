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
%      stop.  All inputs are passed to Fin170_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Fin170

% Last Modified by GUIDE v2.5 10-Dec-2019 15:54:48

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
vid=videoinput('winvideo', 2);
hImage=image(zeros(480, 640, 3), 'Parent', handles.Camera);
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
axis off;
axes(handles.gpsmap);
axis off;
axes(handles.batpercent);
axis off;

datern = datestr(now, 'mm-dd-yyyy');
myString = sprintf('%s', datern);
handles.date.String = myString;
        

s = sprintf('RIGHT');
set(handles.camright, 'TooltipString', s)
s = sprintf('LEFT');
set(handles.camleft, 'TooltipString', s)
s = sprintf('CENTER');
set(handles.forward, 'TooltipString', s)
s = sprintf('FORWARD/STOP');
set(handles.stop, 'TooltipString', s)


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
      set(handles.sysstatus, 'backgroundcolor', 'g');
         if ~isempty(instrfind)
             fclose(instrfind);
             delete(instrfind);
         end
        global xx;
        g = instrhwinfo('serial');
        gg = g.AvailableSerialPorts;
        ggg = char(gg);
        xx = serial(ggg,'BAUD', 9600);
   
        hold on
        A = [0 0 0];
        B = [1 0 0];
        C = [0 1 0];
        D = [0 0 1];
        E = [0 1 1];
        F = [1 0 1];
        G = [1 1 0];
        H = [1 1 1];
        D(3)=D(3)*0.3;
        F(3)=F(3)*0.3;
        E(3)=E(3)*0.3;
        H(3)=H(3)*0.3;
        A(1)=A(1)+0.2;
        D(1)=D(1)+0.2;
        F(1)=F(1)-0.2;
        B(1)=B(1)-0.2;
        Center=H/2;
        Displacement=[0 0 1];
        fopen(xx);
        cnt = 1;

    while 1
    
    d = fscanf(xx);
    
    %Multiple read
    [t1,remain] = strtok(d, ','); %voltage
    r1 = remain;
    [t2,remain] = strtok(r1, ',');%current
    r2 = remain;
    [t3,remain] = strtok(r2, ',');%pitch 
    r3 = remain;
    [t4,remain] = strtok(r3, ',');%roll
     r4 = remain;
    [t5,remain] = strtok(r4, ',');%yaw

    data(1,cnt) = cnt;
    data(2,cnt) = str2double(t1);
    data(3,cnt) = str2double(t2);
    data(4,cnt) = str2double(t3);
    data(5,cnt) = str2double(t4);
    data(6,cnt) = str2double(t5);
    
%     set(handles.text6,'string',t4);
%     set(handles.text7,'string',t3);
%     set(handles.text8,'string',t5);
%     set(handles.v1,'string',t1);
%     set(handles.c1,'string',t2);
       
   dcm = angle2dcm(data(6,cnt),data(4,cnt),data(5,cnt));
   
   P = [A;B;F;H;G;C;A;D;E;H;F;D;E;C;G;B];
   for i=1:length(P)
       P(i,:)=P(i,:)-Center;
   end

   P = P*dcm;
   for i=1:length(P)
       P(i,:)=P(i,:)+Center;
   end
   
   axes(handles.IMU1axes);
   plot(data(1,:),data(5,:),'g',data(1,:),data(4,:),'b',data(1,:),data(6,:),'r');
   axes(handles.IMU2axes);
   plot3(P(:,1),P(:,2),P(:,3),'Color','r');hold on;
   
   hold off;
   axis([-2 2 -2 2 -2 2]);

   view([1,1,1]) ;
   xlabel('X');
   ylabel('Y');
   zlabel('Z');
   
    drawnow;
   [f,ff] = size(data);
    if ff > 2
        data(:,2) = [];
        cnt = 2;
    end
    cnt = cnt + 1;
    end
    guidata(hObject, handles);
    
%     %LET PERCENT BE THE VALUE SA SOC
%     set(handles.speedstat,'String',num2str(percent));
%     if val >= 75
%     batper = imread('100.png');
%     axes(handles.IMU1axes);
%     imshow(batper);
%     elseif val < 75 && val >= 50
%     batper = imread('75.png');
%     axes(handles.IMU1axes);
%     imshow(batper);
%     elseif val < 50 && val >= 20
%     batper = imread('50.png');
%     axes(handles.IMU1axes);
%     imshow(batper);
%     elseif val < 20 && val >= 1
%     batper = imread('25.png');
%     axes(handles.IMU1axes);
%     imshow(batper);.
%     else
%     batper = imread('0.png');
%     axes(handles.IMU1axes);
%     imshow(batper);
%     end
  
    

else
    set(hObject, 'String', 'START');
    set(handles.sysstatus, 'String', 'OFF');
    set(handles.sysstatus, 'backgroundcolor', 'r');
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
         f= errordlg('Click START button to operate again.', 'System off!'); 
end


% --- Executes on button press in Capture.
function Capture_Callback(hObject, eventdata, handles)
% hObject    handle to Capture (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fulldate = datestr(now, 'yyyy-mm-ddTHHMMSSZ');
F = getframe(handles.Camera);
Image = frame2im(F);
savename= strcat ('C:\Users\User1\Documents\EE\5th Year 1\EE 170\Final_EE170\Pictures\', num2str(fulldate), '.jpg');
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
set(handles.speedstat,'String',num2str(val));
iamspeed = val + 1190; 
x = 1;
y = iamspeed;
x = num2str(x);
y = num2str(y);
send =[ x,',',y];
fprintf(xx, send);

% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xx;
ispushed= get(hObject, 'Value');

if ispushed
    set(hObject, 'String', 'STOP');
    x = 1;
    y = 1220;
    set(handles.speed,'Value',20);
    set(handles.speedstat,'String',20);
    x = num2str(x);
    y = num2str(y);
    send =[ x,',',y];
    fprintf(xx, send);

else
    set(hObject, 'String', 'MOVE');
    x = 1;
    y = 1100;
    set(handles.speed,'Value',0);
    x = num2str(x);
    y = num2str(y);
    send =[ x,',',y];
     fprintf(xx, send);   
end


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
x = 2;
y = 130;
x = num2str(x);
y = num2str(y);
send =[ x,',',y];
fprintf(xx, send);

% --- Executes on button press in camleft.
function left_Callback(hObject, eventdata, handles)
% hObject    handle to camleft (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xx;
x = 2;
y = 40;
x = num2str(x);
y = num2str(y);
send =[ x,',',y];
fprintf(xx, send);

% --- Executes on button press in forward.
function forward_Callback(hObject, eventdata, handles)
% hObject    handle to forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global xx;
x = 2;
y = 90;
x = num2str(x);
y = num2str(y);
send =[ x,',',y];
fprintf(xx, send);



% --- Executes on button press in conveyor.
function conveyor_Callback(hObject, eventdata, handles)
% hObject    handle to conveyor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global xx;
ispushed= get(hObject, 'Value');

if ispushed
   set(hObject, 'String', 'ON');
   x = 4;
    y = 400;
    x = num2str(x);
    y = num2str(y);
    send =[ x,',',y];
    fprintf(xx, send);
else
   set(hObject, 'String', 'OFF');
   x = 4;
    y = 0;
    x = num2str(x);
    y = num2str(y);
    send =[ x,',',y];
    fprintf(xx, send);
 
end
% Hint: get(hObject,'Value') returns toggle state of conveyor


% --- Executes on button press in center.
function center_Callback(hObject, eventdata, handles)
% hObject    handle to center (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xx;
x = 3;
y = 90;
x = num2str(x);
y = num2str(y);
send =[ x,',',y];
fprintf(xx, send);



% --- Executes on button press in camright.
function camright_Callback(hObject, eventdata, handles)
% hObject    handle to camright (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xx;
x = 3;
y = 89;
x = num2str(x);
y = num2str(y);
send =[ x,',',y];
fprintf(xx, send);



% --- Executes on button press in camleft.
function camleft_Callback(hObject, eventdata, handles)
% hObject    handle to camleft (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xx;
x = 3;
y = 91;
x = num2str(x);
y = num2str(y);
send =[ x,',',y];
fprintf(xx, send);


% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

H = datestr(now, 'HH');
set(handles.hour, 'String',H);
M = datestr(now, 'MM');
set(handles.minute, 'String',M);
S = datestr(now, 'SS');
set(handles.second, 'String',S);


while M<60
    while S<60
    set(handles.second, 'String',S);
    pause(1);
    S = S+1;
    end
    H = datestr(now, 'HH');
    set(handles.hour, 'String',H);
    M = datestr(now, 'MM');
    set(handles.minute, 'String',M);
%while M<
    
end   
return
