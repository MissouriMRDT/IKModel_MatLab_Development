%Calculate a rotation matrix of a rotation in about the Y axis by theta
function Ry=Roty(t)
st=sin(t);
ct=cos(t);
Ry=[ct, 0, -st;        
    0   1   0;
    st  0  ct];
end