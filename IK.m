% This is Rhino's specific IK solution from 2016/2017 Dev (Zenith/Gryphon)

a=5;   %Arm lengths
b=0;
c=15;
d=15;
e=5;

old1=0;
old2=0;
old3=0;
old4=0;
old5=0;
old6=0;

xi=0;
zi=0;
zi=0;

pause('on');
for i=0:1:1000

 if xi<=0
     zi=zi+1;
 end
 if xi>=30
     zi=zi-1;
 end
 if zi<=0;
     xi=xi-1;
 end 
 if zi>=30
     xi=xi+1;
 end



x=-5+xi/3;        %input metrics (home position commented out) x=0
y=20;  %y=19.2024
z=10+zi/3;  %z=25.2722
w=0%1.5*sin(i/10);        %w=0
p=i/5.2;        %p=0
r=0;        %r=0


F=[0; 0; 11.0231; 0; 0; 0];

sw=sin(w);
cw=cos(w);
sp=sin(p);
cp=cos(p);
sr=sin(r);
cr=cos(r);

Rw=[1, 0, 0;        %Rotation matrices at wrist
    0 cw -sw;
    0 sw cw];
Rp=[cp 0 sp;
    0 1 0;
    -sp 0 cp];
Rr=[cr -sr 0;
    sr cr 0;
    0 0 1];
O=[0 1 0;
   0 0 1;
   1 0 0];
R=Rr*Rp*Rw;
o=[x; y; z];
oc=o-e*R*O*[0; 0; 1];

l=sqrt(oc(1)^2+oc(2)^2)-b;      %Geomtries to solve position IK
D=sqrt((l^2)+((oc(3)-a)^2));
alp=acos((((c^2)+(d^2)-(D^2))/(2*d*c)));
gam=asin((d/D)*sin(alp));

th1=atan2(-oc(2),oc(1));         %angls theta 1-3
th2=atan2(oc(3)-a,l)+gam-pi/2;
th3=(alp-pi/2);
s1=sin(th1);
c1=cos(th1);
s2=sin(th2);
c2=cos(th2);
s3=sin(th3);
c3=cos(th3);
A1=[-s1, 0, c1, b*-s1; %Transformation matrices 
    c1, 0, s1, b*c1;
    0, 1, 0, a;
    0, 0, 0, 1];
A2=[-s2, -c2, 0, -c*s2;
    c2, -s2, 0, c*c2;
    0, 0, 1, 0;
    0, 0, 0, 1];
A3=[c3, 0, s3, 0;
    s3, 0, -c3, 0;
    0, 1, 0, 0;
    0, 0, 0, 1];
T1=A1;
T2=T1*A2;
T3=T2*A3;

WR=transpose(T3(1:3,1:3))*R*O;


%if WR(1,3)<0
     th51=atan2(sqrt(1-(WR(3,3)^2)),WR(3,3));
     th41=atan2(WR(2,3),WR(1,3));
     th61=atan2(WR(3,2),-WR(3,1));
     
     th52=atan2(-sqrt(1-(WR(3,3)^2)),WR(3,3));
     %th52=-th51
     th42=atan2(-WR(2,3),-WR(1,3));
     th62=atan2(-WR(3,2),WR(3,1));
 if (angledist(old5,th51)+angledist(old4,th41)+angledist(old6,th61))>(angledist(old5,th52)+angledist(old4,th42)+angledist(old6,th62))
     th5=th52;
     th4=th42;
     th6=th62;
     orient='D';
 else
     th5=th51;
     th4=th41;
     th6=th61;
     orient='U';
 end
 
 if th5==0
        th4=old4;
        th6=atan2(WR(2,1),WR(1,1))-old4;
 end
 
 
%else
  %  th5=-atan2(WR(3,3),sqrt(1-(WR(3,3)^2)))+pi/2; 
   % if th5==0
   %     th4=0;
    %    th6=-atan2(WR(1,1),WR(2,1))+pi/2;
    %else
    %    th4=-atan2(WR(1,3),WR(2,3))+pi/2;
    %    th6=-atan2(-WR(3,1),WR(3,2))+pi/2;
    %end
%end;
%th4=0;
%th5=0;
%th6=0;

s4=sin(th4);
c4=cos(th4);
s5=sin(th5);
c5=cos(th5);
s6=sin(th6);
c6=cos(th6);
A4=[c4, 0, -s4, 0;   %Transformation matrices 
    s4, 0, c4, 0;
    0, -1, 0, d;
    0, 0, 0, 1];
A5=[c5, 0, s5, 0;
    s5, 0, -c5, 0;
    0, 1, 0, 0;
    0, 0, 0, 1];
A6=[c6, -s6, 0, 0;
    s6, c6, 0, 0;
    0, 0, 1, e;
    0, 0, 0, 1];
T4=T3*A4;
T5=T4*A5;
T6=T5*A6;

o0=[0; 0; 0];
o1=T1(1:3,4);
o2=T2(1:3,4);
o3=T3(1:3,4);
o4=T4(1:3,4);
o5=T5(1:3,4);
o6=T6(1:3,4);

z0=[0; 0; 1];
z1=T1(1:3,3);
z2=T2(1:3,3);
z3=T3(1:3,3);
z4=T4(1:3,3);
z5=T5(1:3,3);
z6=T6(1:3,3);

J1=[cross(z0,(o6-o0)), cross(z1,(o6-o1)), cross(z2,(o6-o2)), cross(z3,(o6-o3)), cross(z4,(o6-o4)), cross(z5,(o6-o5));...
    z0,         z1,         z2,         z3,         z4,         z5];
J2=[cross(z1,(o6-o1)), cross(z2,(o6-o2)), cross(z3,(o6-o3)), cross(z4,(o6-o4)), cross(z5,(o6-o5));...
    z1,         z2,         z3,         z4,         z5];
J3=[cross(z2,(o6-o2)), cross(z3,(o6-o3)), cross(z4,(o6-o4)), cross(z5,(o6-o5));...
    z2,         z3,         z4,         z5];
J4=[cross(z3,(o6-o3)), cross(z4,(o6-o4)), cross(z5,(o6-o5));...
    z3,         z4,         z5];
J5=[cross(z4,(o6-o4)), cross(z5,(o6-o5));...
    z4,         z5];
J6=[cross(z5,(o6-o5));...
    z5];

%J1t=inv(J1);
%J2t=J2'/(J2*J2');
%J3t=J3'/(J3*J3');
%J4t=J4'/(J4*J4');
%J5t=J5'/(J5*J5');
%J6t=J6'/(J6*J6');

T=J1'*F;

X=[T1(1,4);T2(1,4);T3(1,4);T4(1,4);T5(1,4);T6(1,4)]; %From here on all code is to display to the graph
Y=[T1(2,4);T2(2,4);T3(2,4);T4(2,4);T5(2,4);T6(2,4)];
Z=[T1(3,4);T2(3,4);T3(3,4);T4(3,4);T5(3,4);T6(3,4)];

plot3([0 X(1,1)],[0 Y(1,1)],[0 Z(1,1)],'k',...
X(1:2,1),Y(1:2,1),Z(1:2,1),'k',...
X(2:3,1),Y(2:3,1),Z(2:3,1),'k',...
X(3:4,1),Y(3:4,1),Z(3:4,1),'k',...
X(4:5,1),Y(4:5,1),Z(4:5,1),'k',...
X(5:6,1),Y(5:6,1),Z(5:6,1),'k',...
[X(6,1) X(6,1)+T6(1,1)*4],[Y(6,1) Y(6,1)+T6(2,1)*4],[Z(6,1) Z(6,1)+T6(3,1)*4],'r',...
[X(6,1) X(6,1)+T6(1,2)*4],[Y(6,1) Y(6,1)+T6(2,2)*4],[Z(6,1) Z(6,1)+T6(3,2)*4],'g',...
[X(6,1) X(6,1)+T6(1,3)*4],[Y(6,1) Y(6,1)+T6(2,3)*4],[Z(6,1) Z(6,1)+T6(3,3)*4],'b',...
[X(5,1) X(5,1)+T5(1,1)*4],[Y(5,1) Y(5,1)+T5(2,1)*4],[Z(5,1) Z(5,1)+T5(3,1)*4],'r',...
[X(5,1) X(5,1)+T5(1,2)*4],[Y(5,1) Y(5,1)+T5(2,2)*4],[Z(5,1) Z(5,1)+T5(3,2)*4],'g',...
[X(5,1) X(5,1)+T5(1,3)*4],[Y(5,1) Y(5,1)+T5(2,3)*4],[Z(5,1) Z(5,1)+T5(3,3)*4],'b',...
[X(4,1) X(4,1)+T4(1,1)*6],[Y(4,1) Y(4,1)+T4(2,1)*6],[Z(4,1) Z(4,1)+T4(3,1)*6],'r',...
[X(4,1) X(4,1)+T4(1,2)*6],[Y(4,1) Y(4,1)+T4(2,2)*6],[Z(4,1) Z(4,1)+T4(3,2)*6],'g',...
[X(4,1) X(4,1)+T4(1,3)*6],[Y(4,1) Y(4,1)+T4(2,3)*6],[Z(4,1) Z(4,1)+T4(3,3)*6],'b',...
[X(3,1) X(3,1)+T3(1,1)*4],[Y(3,1) Y(3,1)+T3(2,1)*4],[Z(3,1) Z(3,1)+T3(3,1)*4],'r',...
[X(3,1) X(3,1)+T3(1,2)*4],[Y(3,1) Y(3,1)+T3(2,2)*4],[Z(3,1) Z(3,1)+T3(3,2)*4],'g',...
[X(3,1) X(3,1)+T3(1,3)*4],[Y(3,1) Y(3,1)+T3(2,3)*4],[Z(3,1) Z(3,1)+T3(3,3)*4],'b',...
[X(2,1) X(2,1)+T2(1,1)*4],[Y(2,1) Y(2,1)+T2(2,1)*4],[Z(2,1) Z(2,1)+T2(3,1)*4],'r',...
[X(2,1) X(2,1)+T2(1,2)*4],[Y(2,1) Y(2,1)+T2(2,2)*4],[Z(2,1) Z(2,1)+T2(3,2)*4],'g',...
[X(2,1) X(2,1)+T2(1,3)*4],[Y(2,1) Y(2,1)+T2(2,3)*4],[Z(2,1) Z(2,1)+T2(3,3)*4],'b',...
[X(1,1) X(1,1)+T1(1,1)*4],[Y(1,1) Y(1,1)+T1(2,1)*4],[Z(1,1) Z(1,1)+T1(3,1)*4],'r',...
[X(1,1) X(1,1)+T1(1,2)*4],[Y(1,1) Y(1,1)+T1(2,2)*4],[Z(1,1) Z(1,1)+T1(3,2)*4],'g',...
[X(1,1) X(1,1)+T1(1,3)*4],[Y(1,1) Y(1,1)+T1(2,3)*4],[Z(1,1) Z(1,1)+T1(3,3)*4],'b',...
'LineWidth',3.5)
%{
[X(6,1) X(6,1)+T6(1,1)*4],[Y(6,1) Y(6,1)+T6(2,1)*4],[Z(6,1) Z(6,1)+T6(3,1)*4],'r',...
[X(6,1) X(6,1)+T6(1,2)*4],[Y(6,1) Y(6,1)+T6(2,2)*4],[Z(6,1) Z(6,1)+T6(3,2)*4],'g',...
[X(6,1) X(6,1)+T6(1,3)*4],[Y(6,1) Y(6,1)+T6(2,3)*4],[Z(6,1) Z(6,1)+T6(3,3)*4],'b',...
[X(5,1) X(5,1)+T5(1,3)*4],[Y(5,1) Y(5,1)+T5(2,3)*4],[Z(5,1) Z(5,1)+T5(3,3)*4],'b',...
[X(4,1) X(4,1)+T4(1,3)*4],[Y(4,1) Y(4,1)+T4(2,3)*4],[Z(4,1) Z(4,1)+T4(3,3)*4],'b',...
[X(3,1) X(3,1)+T3(1,3)*4],[Y(3,1) Y(3,1)+T3(2,3)*4],[Z(3,1) Z(3,1)+T3(3,3)*4],'b',...
[X(2,1) X(2,1)+T2(1,3)*4],[Y(2,1) Y(2,1)+T2(2,3)*4],[Z(2,1) Z(2,1)+T2(3,3)*4],'b',...
[X(1,1) X(1,1)+T1(1,3)*4],[Y(1,1) Y(1,1)+T1(2,3)*4],[Z(1,1) Z(1,1)+T1(3,3)*4],'b',...
'LineWidth',3.5)
%}



view(150,30);
grid on
axis([-40 40 -40 40 -40 40])
patch([6 6 -6 -6],[3 -10 -10 3],[0 0 0 0],'r');


   %this will display position and orientation at the endeffector.
disp(th1*(180/pi));%-old1*(180/pi));
disp(th2*(180/pi));%-old2*(180/pi));
disp(th3*(180/pi));%-old3*(180/pi));
disp(th4*(180/pi));%-old4*(180/pi));
disp(th5*(180/pi));%-old5*(180/pi));
disp(th6*(180/pi));%-old6*(180/pi));
disp(orient);
%disp(th41*(180/pi));
%disp(th42*(180/pi));
%disp(angledist(old4,th41));
%disp(angledist(old4,th41));
%disp(WR);
%disp(T);
disp((angledist(old5,th51)+angledist(old4,th41)+angledist(old6,th61))*(180/pi));
disp((angledist(old5,th52)+angledist(old4,th42)+angledist(old6,th62))*(180/pi));

old1=th1;
old2=th2;
old3=th3;
old4=th4;
old5=th5;
old6=th6;

pause(0.1);
end