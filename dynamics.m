function x_dot=dynamics(x,f,M,J,m)
    
    x_dot(1:3)=(rotate(x(7:9)).')*x(4:6);
    x_dot(4:6)=dynvel(x(4:6),x(10:12),f,m);
    x_dot(7:9)= kinang(x(7:9),x(10:12));
    x_dot(10:12)= dynang(x(10:12),M,J);
    
end

function b = dynang(p,M,J)
    c = [0      p(3) -p(2)
        -p(3)   0     p(1)
         p(2)   -p(1) 0];
    I=[1 0 0
       0 1 0
       0 0 1]; 
    b = (J\I)*(c*J*p + M);
end
function b = kinang(a,p)
    b=[1 sin(a(1))*tan(a(2)) cos(a(1))*tan(a(2))
       0 cos(a(1))            -sin(a(1))
       0 sin(a(1))*sec(a(2)) cos(a(1))*sec(a(2))]*p;
end
function b = dynvel(u,p,f,m)
    b=[p(3)*u(2)-p(2)*u(3) p(1)*u(3)-p(3)*u(1) p(2)*u(1)-p(1)*u(2)].' + f/m;

end