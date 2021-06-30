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
    plotGraph3d = plot3(0,0,0'-r')  % every AnalogRead needs to be on its own Plotgraph
    hold on

set(plotGraph3d, 'XData', P(:,1), 'YData',P(:,2),'ZData',P(:,3));