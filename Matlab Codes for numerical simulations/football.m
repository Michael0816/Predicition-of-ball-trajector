clear all;
close all;
% author: 0-0
% Date: 01.01.2023

% 数据初始化
v = input("your V0:");
theta0 = input("your theta0:");

H = 2.4; % 球筐高度

D = input("与球框的距离:");
h = input("出脚高度：");

C = 0.53; %足球阻力系数
p = 1.29; %空气密度
r = 0.11; %足球半径
m = 0.43; %足球质量

eps = 0.05; %冗余

dt = input("时间间隔:");
Total = 5.0;

x0 = 0;
y0 = h;

X_res = [x0]; 
Y_res = [h]; 
T_res = [];

vx_res = [];

count = 0;

v1 = v;
theta1 = theta0;

% 循环
for t1 = 0: dt: Total

    Ax = acceleration_x(C, p, r, v1, m, theta1); % 算t时刻水平方向的加速度
    Ay = acceleration_y(C, p, r, v1, m, theta1); % 算t时刻竖直方向的加速度
    
    
    vx = v1 * cos(theta1); % 算t时刻水平方向的速度
    vy = v1 * sin(theta1); % 算t时刻竖直方向的速度
    
    
    x = X_distance(vx,dt,Ax,x0); % 算t+dt时刻水平方向的位移
    y = Y_distance(vy,dt,Ay,y0); % 算t+dt时刻竖直方向的位移

    x0 = x;
    y0 = y;
    
    % 存变量
    T_res = [T_res, t1];
    X_res = [X_res, x];
    Y_res = [Y_res, y];
    vx_res= [vx_res, vx];
    
    
    vx = Vx(vx, dt, Ax); % 算t+dt时刻水平方向的速度
    vy = Vy(vy, dt, Ay); % 算t+dt时刻水平方向的速度
    
    
    theta1 = degree(vx,vy); % 算t+dt时刻的角度
    
    % 速度的绝对值
    v1 = sqrt(vy * vy + vx * vx);
    
    % 判断
    if abs(x - D) <= eps  &&  abs(y - H) <= 0.1
        count = count + 1;
        break % 跳出for loop
    end

    if x - D > eps
        break
    end
    
end

% 检验
if count == 1
    disp('有阻力，进了');
else
    disp('有阻力，没进');
end


% 没阻力的情况
T2_res = []; %
X2_res = []; %
Y2_res = []; %

v2 = v;
theta2 = theta0;
count2 = 0;

for t2 = 0: dt: Total
    
    vx2 = v2 * cos(theta2);
    vy2 = v2 * sin(theta2);
    
    x2 = ball_x(t2,vx2);
    y2 = ball_y(t2,vy2, h);
    
    T2_res = [T2_res, t2];
    X2_res = [X2_res, x2];
    Y2_res = [Y2_res, y2];

    if abs(x2 - D) <= eps && abs(y2 - H) <= 0.1
        count2 = count2 + 1;
        break
    end

    if x2 - D > eps
        break
    end
   
end

if count2 == 1
    disp('没阻力，进了');
else
    disp('没阻力，没进');
end

% 作图
figure(1)
plot(X_res, Y_res, '-o');
hold on
plot(X2_res, Y2_res);
legend("有阻力","没阻力");
xlabel("X-axis");
ylabel("Y-axis");
title("足球行进路线模拟");
grid on
figure(2)
plot(T_res, vx_res, '-o');
xlabel("Time");
ylabel("Velocity");
legend("时间");
title("速度与时间关系");
grid on
