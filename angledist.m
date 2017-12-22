function a=angledist(theta1,theta2)
    if abs(theta1-theta2)>pi
        a= (2*pi)-abs(theta1-theta2);
    else
        a=abs(theta1-theta2);
    end
end