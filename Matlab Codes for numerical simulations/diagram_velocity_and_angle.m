h = 0.5;
Y0 = 2.4;
X0 = 12;
g = 9.81;


X_res = [];
Y_res = [];
X1_res = [];
Y1_res = [];


for theta0=0:pi/200:pi/2

    v0 = sqrt(g * X0 ^ 2) / sqrt( 2 * sin(theta0) * cos(theta0) - 2 * Y0 * cos(theta0)^2+ 2 * h * cos(theta0)^2);
    if v0 > 0

        X_res = [X_res, theta0];
        Y_res = [Y_res, v0];
    end

end

for theta1 = 0:pi/200:pi/2

    Ax = acceleration_x(C, p, r, v1, m, theta1); % 算t时刻水平方向的加速度
    Ay = acceleration_y(C, p, r, v1, m, theta1); % 算t时刻竖直方向的加速度
    
    
    vx = v1 * cos(theta1); % 算t时刻水平方向的速度
    vy = v1 * sin(theta1); % 算t时刻竖直方向的速度
    
    
    x = X_distance(vx,dt,Ax,x0); % 算t+dt时刻水平方向的位移
    y = Y_distance(vy,dt,Ay,y0); % 算t+dt时刻竖直方向的位移

    X0 = x;
    Y0 = y;
    
    % 存变量
    %T_res = [T_res, t1];
    %X_res = [X_res, theta1];
    %Y_res = [Y_res, v1];
    %vx_res= [vx_res, vx];

    
    
    vx = Vx(vx, dt, Ax); % 算t+dt时刻水平方向的速度
    vy = Vy(vy, dt, Ay); % 算t+dt时刻水平方向的速度
   
    
    %theta1 = degree(vx,vy); % 算t+dt时刻的角度
    
    % 速度的绝对值
    v1 = sqrt(vy * vy + vx * vx);

    X1_res = [X1_res, theta1];
    Y1_res = [Y1_res, v1];
end

plot(X_res, Y_res);
hold on
plot(X1_res, Y1_res);
xlabel("angle");
ylabel("velocity");
grid on;

