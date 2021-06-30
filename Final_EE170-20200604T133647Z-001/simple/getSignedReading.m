function [ signed_reading ] = getSignedReading( reading, offset )
%reading and offset are unsigned 16 bit integers
%The values coming from MPU6050 are unsigned 16 bit integers, range
%0-65535.  But we need signed values to calculate orientation.
LSB_num=65536;
LSB_num_half=LSB_num/2;
if(offset<LSB_num_half)
else
    offset=-(LSB_num-offset);
    %This makes sure the offset is always smaller than LSB_num
end
if(reading<LSB_num_half)
    signed_reading = reading-offset;
elseif(reading>=LSB_num_half)
    signed_reading=-(LSB_num-reading-offset);
end
signed_reading=-signed_reading;
end
