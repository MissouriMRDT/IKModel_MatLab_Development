%Calculate a rotation matrix of a rotation in about the Z axis by theta
function Rz=Rotz(t)
st=sin(t);
ct=cos(t);
Rz=[ct,-st, 0;        
    st  ct  0;
    0   0   1];
end