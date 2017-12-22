% This is Novatny's general IK solution from 2018 Dev 

% Arm setup
% See Generalized Rover IK Document for DH frame assignments

%TEMPORARY
old1=0;
old2=0;
old3=0;
old4=0;
old5=0;
old6=0;


%D-H Parameters of Arm Model
th1offset=pi/2; %should be 90 in order for origin frame to comply with "Rover
% Coordinate Standard"
d1=2.8937; % height of bicep tilt axis from baseplate/origin
a1=0; %forward offset of bicep tilt axis relative to base rotate axis
alpha1=pi/2; %anglular offset of J1 about X1 axis. (SHOULD BE 90 UNLESS ARM 
           % DESIGN IS SUPER FUNKY)
th2offset=pi/2;%should be 90 in order to comply with DH convention 
d2=0;%offset to the right of the bicep relative to the base rotation axis(
     %should probably stay as 0 even if bicep is offset. this offset can 
     %also be accounted for using d3)
a2=17;%bicep length(distance between bicep tilt axis and elbow tilt axis)
alpha2=0;%angular offset of elbow tilt axis about x2 axis.(SHOULD BE 90 
         %UNLESS ARM DESIGN IS SUPER FUNKY)
th3offset=pi/2;%should be 90
d3=0;%offset to the right of the forearm relative to the bicep(see d2 
     %comment, if the bicep is offset from the base rotate axis but you 
     %have d2 as 0, then d3 must be the offset to the right of the forearm 
     %relative to the base rotate axis)
a3=2.837;%offset of forearm twist axis from the elbow tilt axis along the x2 
     %axis. (this is the "vertical" offset of the forearm.  DONT USE THIS 
     %if you calculated the actual distance between the elbow axis and 
     %wrist center and calculated the th3 offset accordingly. in that case 
     %a3 should be 0
alpha3=pi/2;%angular offset of forearm about x3 axis. (SHOULD BE 90 UNLESS ARM 
         %DESIGN IS SUPER FUNKY)
th4offset=0; %angular offset of forearm twist. should be 0 for standard 
             %spherical wrist orientation. (phoenix, horison, zenith, and 
             %gryphon's wrist joints all complied with this)
d4=17;%Forearm Length. If a3 is zero but there is a "vertical" offset of 
      %the forearm, this value needs to be the center to center distance 
      %between the elbow tilt axis and the wrist center.
a4=0; %needs to be 0 for spherical wrist
alpha4=-pi/2; %should be -90 for standard spherical wrist orientation. 
            %(phoenix, horiZon, zenith, and gryphon's wrist joints all 
            %complied with this)
th5offset=0; %wrist tilt angle offset. should be 0 unless there is a 
             %"vertical" forearm offset and you chose to use the center to 
             %center distances between the elbow tilt axis and the wrist 
             %center. if this is the case, th4offset needs to be calculated
             %as the angle between the line center line between the elbow 
             %tilt axis and wrist center with the axis of gripper rotate(j6)
d5=0;%needs to be 0 for spherical wrist
a5=0;%needs to be 0 for spherical wrist
alpha5=pi/2;%angular offset of gripper rotate axis from gripper tilt axis 
          %about x5 axis. needs to be 90 for spherical wrist 
th6offset=pi/2; %angular twist of gripper from normal orientation. should be 
              %90 for standard spherical wrist orientation. (phoenix, 
              %horiZon, zenith, and gryphon's wrist joints all complied with this)
d6=0;%keep as 0
a6=0;%keep as 0
alpha6=pi/2; %angular tilt of gripper from normal orientation. should be 90 
           %for standard spherical wrist orientation. (phoenix, horiZon, 
           %zenith, and gryphon's wrist joints all complied with this)

%CENTER POINT OF GRIPPER
OpPointoffset=[0;5.25; 0]; %x,y,z distances of operating point of gripper relative to the wrist center using the Rover Coordinate Standard from the perspective of the gripper.


%this shit is for animating the plot to test the IK
xi=0;
zi=0;
yi=0;
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

error='ALL GOOD FAM';

%HERE COMES THE IK MATH!!!
%operating point location
x=-10+xi/3;% Desired X coordinate of gripper relative to the Rover (where the arm attaches)
y=20;% Desired Y coordinate of gripper relative to the Rover (where the arm attaches)
z=20+zi/3;% Desired Z coordinate of gripper relative to the Rover (where the arm attaches)

%operating point orientation 
%The Order of these rotations matters for the final orientation. I will 
%choose to Yaw first, Then Pitch, and then%Roll.  this can be changed of 
%course, this is just how i am going to do it
yaw=0; %Rotation of gripper about Gripper's Z axis
pitch=pi/4; %Rotation of gripper about Gripper's X axis
roll=i/30;%Rotation of gripper about Gripper's Y axis

%Calculate the final desired Rotation matris(Gripper Orientation)
OpRot=Rotz(yaw)*Rotx(pitch)*Roty(roll); %Can add other rotations here if so 
%desired. might need to introduce new variables though.

%Calculate the Wrist Center location from Gripper Location and Orientation
OpPoint = [x;y;z];
WristCenter = OpPoint-OpRot*OpPointoffset;


%Position IK Problem
%This you will have likely have to solve yourself.  I will attempt to work it 
%in terms of the DH Model described in the documentation to this, but its not hard. 
%the inverse position problem is just simple Trigonometry (SEE DOCUMENTATION)
L=sqrt(WristCenter(1)^2+WristCenter(2)^2-(d2+d3)^2)-a1; % this is the horizontal distance the bicep and forearm must reach 
th1=atan2(WristCenter(2),WristCenter(1)) - atan2(L,((L/(L+a1)*(d2+d3))));%THIS ONLY SOLVES FOR FORWARD REACH (arm cant reach over head)
B=sqrt(a3^2 + d4^2);%center to center distance from J3 to Wrist Center
R=sqrt(L^2 +(WristCenter(3)-d1)^2);%Reaching distance of bicem and forearm
if R>=(a2+B)%This checks to see if the desired point is within the working envelope of the arm
 th2=old2;
 th3=old3;
 error='POSITION OUT OF RANGE';
else
    D=(R^2-a2^2 -B^2)/(2*a2*B); %acos of angle between bicep and B
    th2=(atan2(B*sqrt(1-D^2),(a2+B*D))+atan2((WristCenter(3)-d1),L)-(pi/2)); %Theta2
    th3=(atan2(-sqrt(1-D^2),D)+ atan2(d4,a3)-pi/2);%Theta3
end


%WRIST ORIENTATION IK
%Define Transformation Matricies of J1, J2, J3
A1=DHTrans(th1+th1offset, d1, a1, alpha1);
A2=DHTrans(th2+th2offset, d2, a2, alpha2);
A3=DHTrans(th3+th3offset, d3, a3, alpha3);
T1=A1;
T2=T1*A2;
T3=T2*A3;

%Find required rotation matrix R3 to 6(combined rot matrix of J4, J5,J6)
WR=transpose(T3(1:3,1:3))*OpRot; %See documentation for description of this

%inorder to choose between wrist-up case and wrist-down case, we need to
%compare the calcualted angles of the 2 solutions and choose the best one
th51=atan2(sqrt(1-(WR(3,2)^2)),WR(3,2));%calculate th5 wrist-up
th41=atan2(WR(2,2),WR(1,2));%calculate th4 wrist-up
th61=atan2(WR(3,1),-WR(3,3));%calculate th6 wrist-up
th52=atan2(-sqrt(1-(WR(3,2)^2)),WR(3,2));%calculate th5 wrist-down
th42=atan2(-WR(2,2),-WR(1,2));%calculate th4 wrist-down
th62=atan2(-WR(3,1),WR(3,3));%calculate th6 wrist-down

%The expression below compares the total angular distance the wrist joints
%would have to travel to reach each solution. it then chooses the solution
%requiring the least movement
if (angledist(old5,th51)+angledist(old4,th41)+angledist(old6,th61))> ...
   (angledist(old5,th52)+angledist(old4,th42)+angledist(old6,th62))
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
 
 %This handles the case if the wrist is at its singularity
 if abs(th5)<0.005
        th4=old4
        th6=atan2(WR(2,3),WR(1,3))-old4
 end

%th5=0;
%th4=0;
%th6=0;
 


A4=DHTrans(th4+th4offset,d4,a4,alpha4);
A5=DHTrans(th5+th5offset,d5,a5,alpha5);
A6=DHTrans(th6+th6offset,d6,a6,alpha6);
EE=[1, 0, 0, OpPointoffset(1);
    0, 1, 0, OpPointoffset(2);
    0, 0, 1, OpPointoffset(3);
    0, 0, 0, 1];
T4=T3*A4;
T5=T4*A5;
T6=T5*A6*EE



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
[X(5,1) X(5,1)+T5(1,3)*4],[Y(5,1) Y(5,1)+T5(2,3)*4],[Z(5,1) Z(5,1)+T5(3,3)*4],'b',...
[X(4,1) X(4,1)+T4(1,3)*6],[Y(4,1) Y(4,1)+T4(2,3)*6],[Z(4,1) Z(4,1)+T4(3,3)*6],'b',...
[X(3,1) X(3,1)+T3(1,3)*4],[Y(3,1) Y(3,1)+T3(2,3)*4],[Z(3,1) Z(3,1)+T3(3,3)*4],'b',...
[X(2,1) X(2,1)+T2(1,3)*4],[Y(2,1) Y(2,1)+T2(2,3)*4],[Z(2,1) Z(2,1)+T2(3,3)*4],'b',...
[X(1,1) X(1,1)+T1(1,3)*4],[Y(1,1) Y(1,1)+T1(2,3)*4],[Z(1,1) Z(1,1)+T1(3,3)*4],'b',...
'LineWidth',3.5)
view(150,30);
grid on
axis([-27 27 -5 27 -12 40])
patch([6 6 -6 -6],[3 -10 -10 3],[0 0 0 0],'r');


old1=th1;
old2=th2;
old3=th3;
old4=th4;
old5=th5;
old6=th6;
q = [th1*(180/pi), th2*(180/pi), th3*(180/pi), th4*(180/pi), th5*(180/pi), th6*(180/pi)]
disp(orient)
disp(error)
pause(0.1);
end
