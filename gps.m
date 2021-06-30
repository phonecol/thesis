clc;
cnt = 1;
lat1= 8.1;
    long1= 124.02;
    gpslat = [];
    gpslong = [];
    geobasemap satellite;
while cnt < 30
    cnt = cnt + 1;
    gpslat(1,1) = lat1;
    gpslong(2,1) = long1;
    newlat = lat1 + (rand*0.1);
    newlong = long1 + rand*0.1;
    gpslat(1,cnt) = newlat;
    gpslong(2,cnt) = newlong;
    geoplot( gpslat(1,:), gpslong(2,:),'g->');
    %geobasemap satellite;
    pause(0.5);
    
end