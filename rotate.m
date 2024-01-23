function Ritob=rotate(x)
    Ritob=[cos(x(2))*cos(x(3))                               cos(x(2))*sin(x(3))                                -sin(x(2))
           sin(x(1))*sin(x(2))*cos(x(3))-cos(x(1))*sin(x(3))    sin(x(1))*sin(x(2))*sin(x(3))+cos(x(1))*cos(x(3))   sin(x(1))*cos(x(2))
           cos(x(1))*sin(x(2))*cos(x(3))+sin(x(1))*sin(x(3))    cos(x(1))*sin(x(2))*sin(x(3))-sin(x(1))*cos(x(3))   cos(x(1))*cos(x(2))];
end