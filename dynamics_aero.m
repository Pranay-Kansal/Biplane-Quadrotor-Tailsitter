t_max=5;                % time of running
nt=200;                 % no of time steps
dt=t_max/(nt-1);        % dt

X=zeros(12,nt);         % state function - x y z u v w phi theta psi p q r
X_dot=zeros(12,nt);     % change of rate

%initial_conditions
X(:,1)=[0 0 0 0 0 0 0 0 0 0 0 0];

%parameters
m=12;
J=[1.86 0 0
   0 2.031 0
   0 0 3.617];
J_1=inv(J);
d1=0.5;
d2=0.5;
c=0.01;
uw=[0 0 0];

%input
f1=45;
f2=15;
f3=45;
f4=15;

tq1=c*f1;
tq2=c*f2;
tq3=c*f3;
tq4=c*f4;

T=f1+f2+f3+f4;
Fmotor_b=[0 0 -T].';
Fg_i=[0 0 m*10].';

D=5;
Y=5;
L=10;

for i=1:nt-1
    
    ur=X(4:6,i)-uw;
    alpha=atand(ur(3)/ur(1));
    beta=atand(ur(2)/(sqrt(ur(1)^2+ur(3)^2)));
    
    Fa_w=[-D Y L].';
    Fa_b=rotate([0 alpha 0])*(rotate([0 0 beta]).')*Fa_w;
    
    F_b=Fmotor_b+rotate(X(7:9,i))*Fg_i+Fa_b;
    M_b=[d1*(f1+f4-f2-f3) d2*(f1+f2-f3-f4) tq1+tq3-tq2-tq4].';
    
    X_dot(1:3,i)=(rotate(X(7:9,i)).')*X(4:6,i);
    X_dot(4:6,i)=dynvel(X(4:6,i),X(10:12,i),F_b,m);
    X_dot(7:9,i)= kinang(X(7:9,i),X(10:12,i));
    X_dot(10:12,i)= dynang(X(10:12,i),M_b,J,J_1);
     
    X(:,i+1)=X(:,i)+dt*X_dot(:,i);
end

plot(X(3,:))

function b = dynang(p,M,J,J_1)
    c = [0      p(3) -p(2)
        -p(3)   0     p(1)
         p(2)   -p(1) 0];
    b = J_1*(c*J*p + M);
end
function b = kinang(a,p)
    b=[1 sind(a(1))*tand(a(2)) cosd(a(1))*tand(a(2))
       0 cosd(a(1))            -sind(a(1))
       0 sind(a(1))*secd(a(2)) cosd(a(1))*secd(a(2))]*p;
end
function b = dynvel(u,p,f,m)
    b=[p(3)*u(2)-p(2)*u(3) p(1)*u(3)-p(3)*u(1) p(2)*u(1)-p(1)*u(2)].' + f/m;

end
function R = rotate(x)
    R=[cosd(x(2))*cosd(x(3))                                                       cosd(x(2))*sind(x(3))                                                     -sind(x(2))
        sind(x(1))*sind(x(2))*cosd(x(3))-cosd(x(1))*sind(x(3))    sind(x(1))*sind(x(2))*sind(x(3))+cosd(x(1))*cosd(x(3))   sind(x(1))*cosd(x(2))
        cosd(x(1))*sind(x(2))*cosd(x(3))+sind(x(1))*sind(x(3))    cosd(x(1))*sind(x(2))*sind(x(3))-sind(x(1))*cosd(x(3))   cosd(x(1))*cosd(x(2))];
end