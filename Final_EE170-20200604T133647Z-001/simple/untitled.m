function varargout = ReadFromSerialPort(varargin)
% READFROMSERIALPORT MATLAB code for ReadFromSerialPort.fig
%      READFROMSERIALPORT, by itself, creates a new READFROMSERIALPORT or raises the existing
%      singleton*.
%
%      H = READFROMSERIALPORT returns the handle to a new READFROMSERIALPORT or the handle to
%      the existing singleton*.
%
%      READFROMSERIALPORT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in READFROMSERIALPORT.M with the given input arguments.
%
%      READFROMSERIALPORT('Property','Value',...) creates a new READFROMSERIALPORT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ReadFromSerialPort_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ReadFromSerialPort_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES
% Edit the above text to modify the response to help ReadFromSerialPort
% Last Modified by GUIDE v2.5 21-May-2015 23:42:38
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ReadFromSerialPort_OpeningFcn, ...
                   'gui_OutputFcn',  @ReadFromSerialPort_OutputFcn, ...
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
% --- Executes just before ReadFromSerialPort is made visible.
function ReadFromSerialPort_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ReadFromSerialPort (see VARARGIN)
% Choose default command line output for ReadFromSerialPort
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
% UIWAIT makes ReadFromSerialPort wait for user response (see UIRESUME)
% uiwait(handles.figure1);
global s1;  
s1=serial('COM6','Baudrate',9600);
% --- Outputs from this function are returned to the command line.
function varargout = ReadFromSerialPort_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
varargout{1} = handles.output;
% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global s1;
fopen(s1);
str='';
reading=0;
j=1;
x=0;
accX_offset=0;
accY_offset=0;
accZ_offset=0;
gyroX_offset=65450;
gyroY_offset=65500;
gyroZ_offset=65500;
delta_t=0.04;%
alpha=0.85;%filter parameter.Range from 0 to 1
GyroscopeAngle=zeros(3,5000);
FilteredAngle=zeros(3,5000);
accX=zeros(1,5000);
accY=zeros(1,5000);
accZ=zeros(1,5000);
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
tic
while(j<5000)
    str=fscanf(s1);
    reading=str2num(str);
    
    %get signed values for all 6-DOF
    accX(j)=reading(1);
    accY(j)=reading(2);
    accZ(j)=reading(3);
    gyroX(j)=reading(4);
    gyroY(j)=reading(5);
    gyroZ(j)=reading(6);
    x(j)=j;
    
    %Calculate orientation    
    rho(j)=atan2(accX(j),(accY(j)^2+accZ(j)^2)^0.5);
    phi(j)=atan2(-accY(j),(accX(j)^2+accZ(j)^2)^0.5);
    %theta is different from Yaw
    theta(j)=atan2((accX(j)^2+accY(j)^2)^0.5,-accZ(j));
    
    %use atan2 so that rho&phi range[-180,180], theta range[0,180]
    gyroX(j)=gyroX(j)/131/180*pi;%radians per second
    gyroY(j)=gyroY(j)/131/180*pi;
    gyroZ(j)=gyroZ(j)/131/180*pi;
    
%%
%complementary filter
    if(toc>delta_t)
        if(j==1)
            GyroscopeAngle(1,j)=rho(j);
            GyroscopeAngle(2,j)=phi(j);
            GyroscopeAngle(3,j)=0;
        else
            GyroscopeAngle(1,j)=FilteredAngle(1,j-1)+gyroX(j)*toc;
            GyroscopeAngle(2,j)=FilteredAngle(2,j-1)+gyroY(j)*toc;
            GyroscopeAngle(3,j)=FilteredAngle(3,j-1)+gyroZ(j)*toc;
        end
        tic
    else
    end
    FilteredAngle(1,j) = alpha*GyroscopeAngle(1,j)+(1-alpha)*rho(j);
    FilteredAngle(2,j) = alpha*GyroscopeAngle(2,j)+(1-alpha)*phi(j);
    FilteredAngle(3,j) = alpha*GyroscopeAngle(3,j)+(1-alpha)*GyroscopeAngle(3,j);
    plot(handles.axes1,x,FilteredAngle(1,1:length(x)),x,FilteredAngle(2,1:length(x)),x,FilteredAngle(3,1:length(x))...
                    ,x,rho,x,phi,x,theta);
    
   
%%
%Drawing red box and blue box
   yaw=0;
   pitch=rho(j);
   roll=phi(j);
   
   dcm = angle2dcm(yaw, pitch, roll);
   dcm_filtered = angle2dcm(FilteredAngle(3,j) , FilteredAngle(1,j), FilteredAngle(2,j));
   P = [A;B;F;H;G;C;A;D;E;H;F;D;E;C;G;B];
   P_filtered = [A;B;F;H;G;C;A;D;E;H;F;D;E;C;G;B];
   
   %rotate about Center, but this is not good way to rotate about a point
   for i=1:length(P)
       P(i,:)=P(i,:)-Center;
       P_filtered(i,:)=P_filtered(i,:)-Center; 
   end
   
   P = P*dcm;
   P_filtered = P_filtered*dcm_filtered;
   for i=1:length(P)
       P(i,:)=P(i,:)+Center;
       P_filtered(i,:)=P_filtered(i,:)+Center+Displacement;
   end
   
   plot3(handles.axes2,P(:,1),P(:,2),P(:,3),'Color','r');hold on;
   plot3(handles.axes2,P_filtered(:,1),P_filtered(:,2),P_filtered(:,3));
   hold off;
   axis([-2 2 -2 2 -2 2]);
   grid on;
   view([1,1,1]) ;
   xlabel('X');
   ylabel('Y');
   zlabel('Z');
   
   
    drawnow;
    j=j+1;
end;
% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global s1;
fclose(s1);
% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla;
