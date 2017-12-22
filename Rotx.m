
%Calculate a rotation matrix of a rotation in about the X axis by theta
function Rx=Rotx(t)
st=sin(t);
ct=cos(t);
Rx=[1, 0, 0;        
    0 ct -st;
    0 st ct];
end