function varargout = capabilityMapControls(varargin)
% CAPABILITYMAPCONTROLS MATLAB code for capabilityMapControls.fig
%      CAPABILITYMAPCONTROLS, by itself, creates a new CAPABILITYMAPCONTROLS or raises the existing
%      singleton*.
%
%      H = CAPABILITYMAPCONTROLS returns the handle to a new CAPABILITYMAPCONTROLS or the handle to
%      the existing singleton*.
%
%      CAPABILITYMAPCONTROLS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CAPABILITYMAPCONTROLS.M with the given input arguments.
%
%      CAPABILITYMAPCONTROLS('Property','Value',...) creates a new CAPABILITYMAPCONTROLS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before capabilityMapControls_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to capabilityMapControls_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help capabilityMapControls

% Last Modified by GUIDE v2.5 16-Jul-2015 14:43:25

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @capabilityMapControls_OpeningFcn, ...
                   'gui_OutputFcn',  @capabilityMapControls_OutputFcn, ...
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


% --- Executes just before capabilityMapControls is made visible.
function capabilityMapControls_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to capabilityMapControls (see VARARGIN)

set(handles.slider1, 'Min', -pi/2);
set(handles.slider1, 'Max', pi/2);
set(handles.slider1, 'SliderStep', [pi/180, pi/18]);
set(handles.slider1, 'Value', varargin{1});
set(handles.slider2, 'Min', -pi);
set(handles.slider2, 'Max', pi);
set(handles.slider2, 'SliderStep', [pi/180, pi/18]);
set(handles.slider2, 'Value', varargin{2});
set(handles.slider3, 'Min', 0);
set(handles.slider3, 'Max', 10);
set(handles.slider3, 'Value', varargin{3});
set(handles.slider4, 'Min', 0);
set(handles.slider4, 'Max', 10);
set(handles.slider4, 'Value', varargin{4});
set(handles.slider5, 'Min', 0);
set(handles.slider5, 'Max', 10);
set(handles.slider5, 'Value', varargin{5});
set(handles.text6, 'String', varargin{1}/pi*180);
set(handles.text7, 'String', varargin{2}/pi*180);
set(handles.text8, 'String', varargin{3});
set(handles.text9, 'String', varargin{4});
set(handles.text10, 'String', varargin{5});
set(hObject, 'UserData', {varargin{6}, varargin{7}});

% Choose default command line output for capabilityMapControls
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes capabilityMapControls wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = capabilityMapControls_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text6, 'String', get(hObject, 'Value')/pi*180);
drawMap(handles, get(gcbf, 'UserData'))


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text7, 'String', get(hObject, 'Value')/pi*180);
drawMap(handles, get(gcbf, 'UserData'))


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text8, 'String', get(hObject, 'Value'));
drawMap(handles, get(gcbf, 'UserData'))


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text9, 'String', get(hObject, 'Value'));
drawMap(handles, get(gcbf, 'UserData'))


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text10, 'String', get(hObject, 'Value'));
drawMap(handles, get(gcbf, 'UserData'))


% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function drawMap(handles, UserData)
  mapFile = UserData{1};
  center = UserData{2};
  load(mapFile, 'options')
  sa = get(handles.slider1, 'Value');
  ta = get(handles.slider2, 'Value');
  sc = get(handles.slider3, 'Value');
  tc = get(handles.slider4, 'Value');
  rc = get(handles.slider5, 'Value');
  [map, reachabilityIndex, sphCenters] = pruneCapabilityMap(mapFile, sa, ta, sc, tc, rc);
  sphCentersShoulder = sphCenters + repmat(center, 1, size(sphCenters, 2));
  diameter = options.sphDiameter;
  
  nSph = size(map, 1);
  set(handles.text11, 'String', sprintf('n Spheres: %d', nSph))
  lcmClient = LCMGLClient('CapabilityMap');
  for sph = 1:nSph
    color = hsv2rgb([reachabilityIndex(sph) 1 1]);
    lcmClient.glColor3f(color(1), color(2), color(3))
    lcmClient.sphere(sphCentersShoulder(:,sph), diameter/2, 20, 20);   
  end
  lcmClient.switchBuffers();
