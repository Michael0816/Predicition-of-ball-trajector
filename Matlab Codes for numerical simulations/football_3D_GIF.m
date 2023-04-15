clear all;
close all;
% author: 0-0
% Date: 01.01.2023

% 数据初始化
v = 30; %input("your V0:");
theta0 = 0.4; %input("your theta0:");
theta_hor = pi/4; %input("your theta_hor:");

H = 2.4; % 球筐高度
y_goal = 7.32;

D = 20; %input("与球框的距离:");
h = 0; %input("出脚高度：");

C = 0.53; %足球阻力系数
p = 1.29; %空气密度
r = 0.11; %足球半径
m = 0.43; %足球质量

eps = 0.05; %冗余

dt = 0.01;%input("时间间隔:");
Total = 5.0;

x0 = 0;
y0 = 0;
z0 = h;

X_res = [x0]; 
Y_res = [y0]; 
Z_res = [h];

v1 = v;
theta1 = theta0;

% 循环
for t1 = 0: dt: Total

    A_hor = acceleration_x(C, p, r, v1, m, theta1); % 算t时刻水平方向的加速度
    Az = acceleration_y(C, p, r, v1, m, theta1); % 算t时刻竖直方向的加速度
    Ax = A_hor * cos(theta_hor);
    Ay = A_hor * sin(theta_hor);
    
    vhor = v1 * cos(theta1);% 算t时刻水平方向的速度
    vx = vhor * cos(theta_hor);
    vy = vhor * sin(theta_hor);
    vz = v1 * sin(theta1); % 算t时刻竖直方向的速度
    
    
    x = X_distance(vx,dt,Ax,x0); % 算t+dt时刻水平方向的位移
    y = X_distance(vy, dt, Ay, y0);
    z = Y_distance(vz,dt,Az,z0); % 算t+dt时刻竖直方向的位移

    x0 = x;
    y0 = y;
    z0 = z;
    
    % 存变量
    if z >= 0
        X_res = [X_res, x];
        Y_res = [Y_res, y];
        Z_res = [Z_res, z];
    end
    
    vx = Vx(vx, dt, Ax); % 算t+dt时刻水平方向的速度
    vy = Vx(vy, dt, Ay);% 算t+dt时刻水平方向的速度
    vz = Vy(vz, dt, Az);
    
    theta_hor = atan(vy / vx);
    vhor = sqrt(vy^2 + vx^2);
    theta1 = degree(vhor,vz); % 算t+dt时刻的角度
    
    % 速度的绝对值
    v1 = sqrt( vhor^2 + vz^2);
    
end


gate_width = 7.32;
gate_height = H;
gate_position = [D, -gate_width/2, 0,  gate_width, 0,  gate_height];

a1 = [D, gate_width/2, 0];
a2 = [D, -gate_width/2, 0];
a3 = [D, gate_width/2, ];
a4 = [D, -gate_width/2, gate_height];


% 作图
% Initialize the figure and set up the plot
fig = figure();
set(fig,'color','white');
axis equal;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
grid on;
xlim([0 D*1.2])
ylim([-5 25])
zlim([0 6]);
hold on;
plot3(X_res, Y_res, Z_res, 'LineWidth', 2);

% Add the goalposts
hold on;
plot3([D,D], [-gate_width/2+20,gate_width/2+20], [0,0], 'color', 'black')
hold on;
plot3([D,D], [-gate_width/2+20,-gate_width/2+20], [0,gate_height], 'color', 'black')
hold on;
plot3([D,D], [gate_width/2+20,gate_width/2+20], [0,gate_height], 'color', 'black')
hold on;
plot3([D,D], [gate_width/2+20,-gate_width/2+20], [gate_height,gate_height], 'color', 'black')

% Set up the animation
filename = 'soccer_animation.gif';
frame_rate = 30;
n_frames = length(X_res);
view([-15, 10]);
for i = 1:n_frames

    cla;
    plot3(X_res(1:i), Y_res(1:i), Z_res(1:i), 'LineWidth', 2);
    plot3([D,D], [-gate_width/2+20,gate_width/2+20], [0,0], 'color', 'black')
    plot3([D,D], [-gate_width/2+20,-gate_width/2+20], [0,gate_height], 'color', 'black')
    plot3([D,D], [gate_width/2+20,gate_width/2+20], [0,gate_height], 'color', 'black')
    plot3([D,D], [gate_width/2+20,-gate_width/2+20], [gate_height,gate_height], 'color', 'black')
    title(sprintf('Frame %d', i));
    
    drawnow();
    frame = getframe(fig);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if i == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 1/frame_rate);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 1/frame_rate);
    end
end 
