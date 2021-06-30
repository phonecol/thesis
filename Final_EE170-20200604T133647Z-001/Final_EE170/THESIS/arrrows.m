function varargout = arrrows(varargin)
% ARRROWS MATLAB code for arrrows.fig
%      ARRROWS, by itself, creates a new ARRROWS or raises the existing
%      singleton*.
%
%      H = ARRROWS returns the handle to a new ARRROWS or the handle to
%      the existing singleton*.
%
%      ARRROWS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ARRROWS.M with the given input arguments.
%
%      ARRROWS('Property','Value',...) creates a new ARRROWS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before arrrows_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to arrrows_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help arrrows

% Last Modified by GUIDE v2.5 23-Jan-2020 15:30:56

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @arrrows_OpeningFcn, ...
                   'gui_OutputFcn',  @arrrows_OutputFcn, ...
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


% --- Executes just before arrrows is made visible.
function arrrows_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to arrrows (see VARARGIN)



% Choose default command line output for arrrows
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes arrrows wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = arrrows_OutputFcn(hObject, eventdata, handles) 
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

set(handles.hng,'String',111111);
set(handles.hyyy,'String',000000);
      


% --- Executes on key press with focus on pushbutton1 and none of its controls.
function pushbutton1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
% determine the key that was pressed 
 



% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.hyyy,'String',111111);
set(handles.hng,'String',000000);


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key press with focus on edit1 and none of its controls.
function edit1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

keyPressed = eventdata.Key;

%  if strcmpi(keyPressed,'uparrow')
%      pushbutton1_Callback(handles.pushbutton1,[],handles);
%  elseif strcmpi(keyPressed,'downarrow')
%      pushbutton2_Callback(handles.pushbutton2,[],handles);
%  end

switch keyPressed
    case 'uparrow'
        pushbutton1_Callback(handles.pushbutton1,[],handles);
    case 'downarrow'
        pushbutton2_Callback(handles.pushbutton2,[],handles);
        
end


% --- Executes on key press with focus on pushbutton2 and none of its controls.
function pushbutton2_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

 global xx;
 g = instrhwinfo('serial');
 gg = g.AvailableSerialPorts;
 ggg = char(gg);
 xx = serial(ggg,'BAUD', 9600);
   
     

    while 1
    %tic
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
    
   
    set(handles.yaw,'string',t5);
    
    end

