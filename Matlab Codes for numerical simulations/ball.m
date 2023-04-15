%ball throw
%author: Danny

v0 = 8.41872912;  %出手速度
theta0 = pi/4;  %出手角度

v0x = v0 * cos(theta0);
v0y = v0 * sin(theta0);

Y = 3.05;  %篮筐高度
X = 6.75;  %篮筐水平距离
y0 = 2.6;  %出手点的高度

delta_y = Y - y0;  %高度差
eps = 0.05; %篮筐半径
dt = 0.005; %
Total = 1.5; %
count = 0;

T_res = []; %
X_res = []; %
Y_res = []; %

for t = 0: dt: Total
    
    x = ball_x(t,v0x);
    y = ball_y(t,v0y, y0);
    
    T_res = [T_res, t];
    X_res = [X_res, x];
    Y_res = [Y_res, y];

    if abs(x - X) <= eps && abs(y - H) <= eps
        count = count + 1;
        break
    end

    if x - X > eps
        break
    end
   
end

if count == 1
    disp('ok');
else
    disp('no');
end




plot(X_res, Y_res);