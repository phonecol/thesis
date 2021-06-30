
clc;
clear all;
  if ~isempty(instrfind)
       fclose(instrfind);
       delete(instrfind);
  end

   
com=serial('COM13','BAUD', 9600);
%axes(handles.axes1);


%draw the box
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


%open serial port use the below command,
%open the serial com
fopen(com);

cnt = 1;

while 1
   tic 
    d = fscanf(com);
    
    %Multiple read
    [t1,remain] = strtok(d, ',');
    r1 = remain;
    [t2,remain] = strtok(r1, ',');
    r2 = remain;
    [t3,remain] = strtok(r2, ',');
    r3 = remain;
    [t4,remain] = strtok(r3, ',');
    % r4 = remain;
    % [t5,remain] = strtok(r4, ',');
    % r5 = remain;
    % [t6,remain] = strtok(r5, ',');
    % r6 = remain;
    % [t7,remain] = strtok(r6, ',');
    % r7 = remain;
    % [t8,remain] = strtok(r7, ',');
    % r8 = remain;
    % t9 = strtok(r8, ',');
    
    
    data(1,cnt) = cnt;
    data(2,cnt) = str2double(t2);%pitch
    data(3,cnt) = str2double(t3);%roll
    data(4,cnt) = str2double(t4);%yaw
    % data(5,cnt) = str2double(t4);
    % data(6,cnt) = str2double(t5);
    % data(7,cnt) = str2double(t6);
    % data(8,cnt) = str2double(t7);
    % data(9,cnt) = str2double(t8);
    % data(10,cnt) = str2double(t9);
    
  %  plot(data(1,:),data(2,:),'g',data(1,:),data(3,:),'b',data(1,:),data(4,:),'r');
    % yaw=0;
%    pitch=rho(j);
%    roll=phi(j);
   
   dcm = angle2dcm(data(4,cnt),data(2,cnt),data(3,cnt));
   %dcm_filtered = angle2dcm(FilteredAngle(3,j) , FilteredAngle(1,j), FilteredAngle(2,j));
   P = [A;B;F;H;G;C;A;D;E;H;F;D;E;C;G;B];
   %P_filtered = [A;B;F;H;G;C;A;D;E;H;F;D;E;C;G;B];
   
   %rotate about Center, but this is not good way to rotate about a point
   for i=1:length(P)
       P(i,:)=P(i,:)-Center;
       %P_filtered(i,:)=P_filtered(i,:)-Center; 
   end
   
   P = P*dcm;
   %P_filtered = P_filtered*dcm_filtered;
   for i=1:length(P)
       P(i,:)=P(i,:)+Center;
    %   P_filtered(i,:)=P_filtered(i,:)+Center+Displacement;
   end
   
   plot3(P(:,1),P(:,2),P(:,3),'Color','r');hold off;
   %plot3(handles.axes2,P_filtered(:,1),P_filtered(:,2),P_filtered(:,3));
   hold off;
   axis([-2 2 -2 2 -2 2]);
%    grid on;
%    view([1,1,1]) ;
%    xlabel('X');
%    ylabel('Y');
%    zlabel('Z');
   
   pause(0.05);
%     drawnow;
%     j=j+1;
    
    
   
    
%     ylim([-5 5]);
%     if(cnt <= 20)
%         xlim([0 40]);
%     else
%         xlim([cnt-20 cnt+20]);
%     end
    
   % pause(0.2);
    
    cnt = cnt + 1;
    toc
end
fclose(com)