clc;

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

global xx

popStrings = get(handles.popupmenu2, 'String');
selectedIndex = get(handles.popupmenu2,'Value');

com = popStrings(selectedIndex,:);

port = com(~isspace(com));
set(handles.value1,'string',com);

xx = serial(port,'BAUD',9600);

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

fopen(xx); %communicate with the selected COM
    cnt1 = 1;
    cnt2 = 1;
    cnt3 = 1;
    cnt4 = 1;
    
global GPS_data
global IMU_data
global FEEDBACK_data
global VSCS_data
GPS_data = [];
IMU_data = [];
VSCS_data = [];
FEEDBACK_data = [];

while 1
    
    serial_data = fscanf(xx);
    [tag, remain] = strtok(serial_data,',');
    
    
    switch tag
        tic
        case '$GPS'
            GPS = strsplit(remain , ',');
            o = datestr(now,'HHMMSSFFF');
            GPS_data(cnt1,1) = str2num(o);
            GPS_data(cnt1,2) = cnt1;
            GPS_data(cnt1,3) =str2double(GPS(1,2)); 
            GPS_data(cnt1,4) =str2double(GPS(1,3));
            GPS_data(cnt1,5) =str2double(GPS(1,4));
            geoplot( GPS_data(:,3), GPS_data(:,4),'g->');
            geobasemap satellite;
            cnt1++;
            `
        case '$IMU'
            IMU = strsplit(remain , ',');
            n = datestr(now,'HHMMSSFFF');
            IMU_data(cnt2,1) = str2num(n);
            IMU_data(cnt2,2) = cnt2;
            IMU_data(cnt2,3) = str2double(IMU(1,2));%pitch
            IMU_data(cnt2,4) = str2double(IMU(1,3));%roll
            IMU_data(cnt2,5) = str2double(IMU(1,4));%yaw
            IMU_data(cnt2,6) = str2double(IMU(1,5));%accx
            IMU_data(cnt2,7) = str2double(IMU(1,6));%accy
            IMU_data(cnt2,8) = str2double(IMU(1,7));%accz
            IMU_data(cnt2,9) = str2double(IMU(1,8));%gyrox
            IMU_data(cnt2,10) = str2double(IMU(1,9));%gyroy
            IMU_data(cnt2,11) = str2double(IMU(1,10));%gyroz
            
            plot(IMU_data(:,2),IMU_data(:,3),'g',IMU_data(:,2),IMU_data(:,4),'b',IMU_data(:,2),IMU_data(:,5),'r');
            cnt2++;
            
        case '$VSCS'
            VSCS = strsplit(remain , ',');
            m = datestr(now,'HHMMSSFFF');
            VSCS_data(cnt3,1) = str2num(m);
            VSCS_data(cnt3,2) = cnt3;
            VSCS_data(cnt3,3) = str2double(VSCS(1,2));
            VSCS_data(cnt3,4) = str2double(VSCS(1,3));
            
            plot(VSCS_data(:,2),VSCS_data(:,3),'g',VSCS_data(:,2),VSCS_data(:,4),'r')
            cnt3++;
        case '$FEEDBACK'
            p = datestr(now,'HHMMSSFFF');
            FEEDBACK_data(cnt4,1) = str2num(p);
            FEEDBACK_data(cnt4,2) = remain;
            remain
            cnt4++;
        toc
    end
    
    
    
    
    global GPS_data
global IMU_data
global FEEDBACK_data
y= datestr(now,'DDmmYYYY');
   if ~isempty(instrfind)
       fclose(instrfind);
       delete(instrfind);
   end
   
   filename = 'GPS1.xlsx' ;  
writematrix(GPS_data,filename,'Sheet',y);
filename2 = 'IMU.xlsx' ;
writematrix(IMU_data,filename2,'Sheet',1);
filename3 = 'FEEDBACK.xlsx'  ; 
writematrix(FEEDBACK_data,filename3,'Sheet',1);
filename4 = 'VoltageCurrent.xlsx'  ; 
writematrix(VSCS_data,filename3,'Sheet',1);

    
   
    
 
    