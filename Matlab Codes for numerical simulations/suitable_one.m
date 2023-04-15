close all;
D = input("distance: ");
h = input("出手高度:");
dt = input("时间间隔: ");
theta_res = [];
v_res = [];



H = 3.05; % 篮筐高度

C = 0.53; %篮球阻力系数
p = 1.29; %空气密度
r = 0.123; % 篮球半径
m = 0.625; %篮球质量

eps = 0.102; %冗余

Total = 1.5;
    
x0 = 0;
y0 = h;

zez_res = []; %用于 存储一个theta0上的所有V0，每一个theta0后清零一次
gap_res = []; %用于 存储每一个theta0上的所有V0的最大值和最小值的差值
thetaGap_res = [];


for theta0= pi/6: 0.0174533: 1.39626340
    
    count = 0;
 
    for v0 = 3: 0.01: 15
        v1 = v0;
        theta1=theta0;    
        
        x2 = x0;
        y2 = y0;
            
        for t1= 0:dt:Total
            Ax = acceleration_x(C, p, r, v1, m, theta1);
            Ay = acceleration_y(C, p, r, v1, m, theta1);
        
    
            vx = v1 * cos(theta1);
            vy = v1 * sin(theta1);
    
    
            x = X_distance(vx,dt,Ax,x2);
            y = Y_distance(vy,dt,Ay,y2);
            x2 = x;
            y2 = y;
           
            
            vx = Vx(vx, dt, Ax);         
            vy = Vy(vy, dt, Ay);
    
            theta1 = degree(vx,vy);
    
    
            v1 = sqrt(vy * vy + vx * vx);
            
    
            if abs(x - D) <= eps  &&  abs(y - H) <= 0.1
                disp("1");
                zez_res = [zez_res, v0];
                v_res = [v_res,v0];
                theta_res = [theta_res,theta0];
                count = 1;
                break
            end
                
            if x - D > eps
                disp("2");
                
                break 
            end
            
        end
          
    end
    g = max(zez_res)- min(zez_res);
    gap_res = [gap_res, g];
    
    zez_res = [];
    
    if count == 1
        thetaGap_res = [thetaGap_res, theta0];
    end
end

theta_res = theta_res * 180/pi ;
thetaGap_res = thetaGap_res * 180/pi;

figure(1);
plot(theta_res, v_res);

figure(2);
plot(thetaGap_res, gap_res);