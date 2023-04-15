global tau1 tau2 

tau1 = -50;
tau2 = -10;

k1   = 10;
k2   = 10;

b1 = 5;
b2 = 3;

suitable_k1 = [];
suitable_k2 = [];
suitable_b1 = [];
suitable_b2 = [];


final_vel_list=[];
final_theta0_list=[];

for k1 = 9.5:0.5:9.5
    for k2 = 9.5:0.5:9.5
        for b1 = 3:1:3
            for b2 = 3:1:3     
                
                final_vel= calculate_u_v(k1,k2,b1,b2);
                final_theta0 = calculate_theta(k1,k2,b1,b2);
                final_vel_list = [final_vel_list, final_vel];
                final_theta0_list = [final_theta0_list, final_theta0];

                %final_xcoord = calculate_FinalXcoord(k1,k2,b1,b2);
                %final_ycoord = calculate_FinalYcoord(k1,k2,b1,b2);
                final_xcoord = 0;
                final_ycoord = 0;
                count = ball(final_vel,final_theta0,final_xcoord,final_ycoord);
                if count == 1
                    suitable_k1 = [suitable_k1, k1];
                    suitable_k2 = [suitable_k2, k2];
                    suitable_b1 = [suitable_b1, b1];
                    suitable_b2 = [suitable_b2, b2];
                    
                else
                    suitable_k1 = [suitable_k1,0];
                    suitable_k2 = [suitable_k2,0];
                    suitable_b1 = [suitable_b1,0];
                    suitable_b2 = [suitable_b2,0];
                    
                end
            end
        end
    end
end


datas = [suitable_k1;suitable_k2;suitable_b1;suitable_b2];
disp(datas);


    

function[Final_velocity] = calculate_u_v(k1,k2,b1,b2)

    %% Define fixed parameters
    m1 = 1.;
    m2 = 1.;
    l1 = 1.;
    l2 = .5;
    c1 = 0.5;
    c2 = 0.25;
    I1 = 0.05;
    I2 = 0.05;


    g  = 9.81;    
    
    th10 =0;
    th20 =0;
    


    dtt = 0.01; %% time increment
    tf =3; %% final time

    p   = [l1; l2 ; c1; c2; m1; m2; I1; I2; g; ...
               th10 ; th20 ; k1; k2 ; b1; b2];  % parameters

    %% Q2.1 Perform simulation
    z0 = [th10 th20 0 0]';
    opts = odeset('AbsTol',1e-10,'RelTol',1e-8);
    sol = ode45(@dynamics,[0 tf],z0,opts,p);
    
    final_state = sol.y(:,end);
    
%     figure(1)
%     clf
%     %% Q2.2 Plot Angles
%     plot(sol.x,sol.y(1:2,:))
%     xlabel('Time (s)')
%     ylabel('Angle (rad)');
%     legend({'\theta_1','\theta_2'});
%         
%     figure(3)
%     clf
%     %% Q2.1 Animation
%     h_pend = plot([0,0],'LineWidth',4);
%     axis([-2 2 -2 2])
%     xlabel('x (m)');
%     ylabel('y (m)')
%     h_title = title('t=0.0s');

    count=0;
    Xcall=[];
    Ycall=[];
    dXcall=[];
    dYcall=[];
    ddYcall=[100];
    
    for t = 0:dtt:tf
        z = deval(sol,t);
        if (z(1)<pi) && (z(2)<pi) && (ddYcall(end)>=g)
            z(1);
            keypoints = keypoints_double_pend_springs(z,p);   %% keypoints of the motion
%             h_pend.XData = [ keypoints(1,:) ];
%             h_pend.YData = [ keypoints(2,:) ];
            h_end.XData = z(1);
            h_end.YData = z(2);
            pause(.01);
            
%             h_title.String = sprintf('t=%.2fs',t);
            Xcall=[Xcall,keypoints(1,3)];
            Ycall=[Ycall,keypoints(2,3)];
            %% GET THE ACCELERATION AT THE HAND
            if count >=2
                ddYcall=[ddYcall,(Ycall(end)+Ycall(end-2)-2*Ycall(end-1))/dtt^2];
            elseif count>=1
                ddYcall=[ddYcall,0];
            end
            %% GET THE VELOCITY AT THE HAND
            if count >=1
                dYcall=[dYcall,(Ycall(end)-Ycall(end-1))/dtt];
                dXcall=[dXcall,(Xcall(end)-Xcall(end-1))/dtt];
            elseif count>=1
                dYcall=[dYcall,0];
                dXcall=[dXcall,0];
            end
            count = count+1;
        end
    end
    ddYcall = ddYcall(2:end);
%     %% plot the variaiton of acceleration at the hand over time
%     figure
%     plot(linspace(0,length(ddYcall),length(ddYcall))*dtt,ddYcall)
%     xlabel('Time (s)')
%     ylabel('Acc at the tip (rad/s^2)');
%     legend({'d^2y_c/dt^2'});
%     
%     figure
%     plot(linspace(0,length(dYcall),length(dYcall))*dtt,dYcall)
%     xlabel('Time (s)')
%     ylabel('Y vel at the tip (m/s)');
%     legend({'dy_c/dt'});

    %
   
    %% What is the x and y coordinate of the final configuration
    Final_coord=[Xcall(end), Ycall(end)];
    Final_velocity= (dXcall(end)*dXcall(end) + dYcall(end) * dYcall(end))^(1/2);
    Final_theta = [atan(dYcall(end)/dXcall(end))];

% end
end

function [Final_theta] = calculate_theta(k1,k2,b1,b2)

    %% Define fixed parameters
    m1 = 1.;
    m2 = 1.;
    l1 = 1.;
    l2 = .5;
    c1 = 0.5;
    c2 = 0.25;
    I1 = 0.05;
    I2 = 0.05;


    g  = 9.81;    
    
    th10 =0;
    th20 =0;
    


    dtt = 0.01; %% time increment
    tf =3; %% final time

    p   = [l1; l2 ; c1; c2; m1; m2; I1; I2; g; ...
               th10 ; th20 ; k1; k2 ; b1; b2];  % parameters

    %% Q2.1 Perform simulation
    z0 = [th10 th20 0 0]';
    opts = odeset('AbsTol',1e-10,'RelTol',1e-8);
    sol = ode45(@dynamics,[0 tf],z0,opts,p);
    
    final_state = sol.y(:,end);
    
%     figure(1)
%     clf
%     %% Q2.2 Plot Angles
%     plot(sol.x,sol.y(1:2,:))
%     xlabel('Time (s)')
%     ylabel('Angle (rad)');
%     legend({'\theta_1','\theta_2'});
%         
    figure(3)
    clf
    %% Q2.1 Animation
    h_pend = plot([0,0],'LineWidth',4);
    axis([-2 2 -2 2])
    xlabel('x (m)');
    ylabel('y (m)')
    h_title = title('t=0.0s');

    count=0;
    Xcall=[];
    Ycall=[];
    dXcall=[];
    dYcall=[];
    ddYcall=[100];
    
    for t = 0:dtt:tf
        z = deval(sol,t);
        if (z(1)<pi) && (z(2)<pi) && (ddYcall(end)>=-g)
            z(1);
            keypoints = keypoints_double_pend_springs(z,p);   %% keypoints of the motion
            h_pend.XData = [ keypoints(1,:) ];
            h_pend.YData = [ keypoints(2,:) ];
            h_end.XData = z(1);
            h_end.YData = z(2);
            pause(.01);
            
%             h_title.String = sprintf('t=%.2fs',t);
            Xcall=[Xcall,keypoints(1,3)];
            Ycall=[Ycall,keypoints(2,3)];
            %% GET THE ACCELERATION AT THE HAND
            if count >=2
                ddYcall=[ddYcall,(Ycall(end)+Ycall(end-2)-2*Ycall(end-1))/dtt^2];
            elseif count>=1
                ddYcall=[ddYcall,0];
            end
            %% GET THE VELOCITY AT THE HAND
            if count >=1
                dYcall=[dYcall,(Ycall(end)-Ycall(end-1))/dtt];
                dXcall=[dXcall,(Xcall(end)-Xcall(end-1))/dtt];
            elseif count>=1
                dYcall=[dYcall,0];
                dXcall=[dXcall,0];
            end
            count = count+1;
        end
    end
    ddYcall = ddYcall(2:end);
%     %% plot the variaiton of acceleration at the hand over time
%     figure
%     plot(linspace(0,length(ddYcall),length(ddYcall))*dtt,ddYcall)
%     xlabel('Time (s)')
%     ylabel('Acc at the tip (rad/s^2)');
%     legend({'d^2y_c/dt^2'});
%     
%     figure
%     plot(linspace(0,length(dYcall),length(dYcall))*dtt,dYcall)
%     xlabel('Time (s)')
%     ylabel('Y vel at the tip (m/s)');
%     legend({'dy_c/dt'});

    %
   
    %% What is the x and y coordinate of the final configuration
    Final_Xcoord = Xcall(end);
    Final_YcoordY = Ycall(end);
    Final_velocity= (dXcall(end)*dXcall(end) + dYcall(end) * dYcall(end))^(1/2);
    Final_theta = atan(dYcall(end)/dXcall(end));

% end
end

function [Final_Ycoord]  = calculate_FinalYcoord(k1,k2,b1,b2)

    %% Define fixed parameters
    m1 = 1.;
    m2 = 1.;
    l1 = 1.;
    l2 = .5;
    c1 = 0.5;
    c2 = 0.25;
    I1 = 0.05;
    I2 = 0.05;


    g  = 9.81;    
    
    th10 =0;
    th20 =0;
    


    dtt = 0.01; %% time increment
    tf =3; %% final time

    p   = [l1; l2 ; c1; c2; m1; m2; I1; I2; g; ...
               th10 ; th20 ; k1; k2 ; b1; b2];  % parameters

    %% Q2.1 Perform simulation
    z0 = [th10 th20 0 0]';
    opts = odeset('AbsTol',1e-10,'RelTol',1e-8);
    sol = ode45(@dynamics,[0 tf],z0,opts,p);
    
    final_state = sol.y(:,end);
    
%     figure(1)
%     clf
%     %% Q2.2 Plot Angles
%     plot(sol.x,sol.y(1:2,:))
%     xlabel('Time (s)')
%     ylabel('Angle (rad)');
%     legend({'\theta_1','\theta_2'});
%         
%     figure(3)
%     clf
%     %% Q2.1 Animation
%     h_pend = plot([0,0],'LineWidth',4);
%     axis([-2 2 -2 2])
%     xlabel('x (m)');
%     ylabel('y (m)')
%     h_title = title('t=0.0s');

    count=0;
    Xcall=[];
    Ycall=[];
    dXcall=[];
    dYcall=[];
    ddYcall=[100];
    
    for t = 0:dtt:tf
        z = deval(sol,t);
        if (z(1)<pi) && (z(2)<pi) && (ddYcall(end)>=-g)
            z(1);
            keypoints = keypoints_double_pend_springs(z,p);   %% keypoints of the motion
%             h_pend.XData = [ keypoints(1,:) ];
%             h_pend.YData = [ keypoints(2,:) ];
            h_end.XData = z(1);
            h_end.YData = z(2);
            pause(.01);
            
%             h_title.String = sprintf('t=%.2fs',t);
            Xcall=[Xcall,keypoints(1,3)];
            Ycall=[Ycall,keypoints(2,3)];
            %% GET THE ACCELERATION AT THE HAND
            if count >=2
                ddYcall=[ddYcall,(Ycall(end)+Ycall(end-2)-2*Ycall(end-1))/dtt^2];
            elseif count>=1
                ddYcall=[ddYcall,0];
            end
            %% GET THE VELOCITY AT THE HAND
            if count >=1
                dYcall=[dYcall,(Ycall(end)-Ycall(end-1))/dtt];
                dXcall=[dXcall,(Xcall(end)-Xcall(end-1))/dtt];
            elseif count>=1
                dYcall=[dYcall,0];
                dXcall=[dXcall,0];
            end
            count = count+1;
        end
    end
    ddYcall = ddYcall(2:end);
%     %% plot the variaiton of acceleration at the hand over time
%     figure
%     plot(linspace(0,length(ddYcall),length(ddYcall))*dtt,ddYcall)
%     xlabel('Time (s)')
%     ylabel('Acc at the tip (rad/s^2)');
%     legend({'d^2y_c/dt^2'});
%     
%     figure
%     plot(linspace(0,length(dYcall),length(dYcall))*dtt,dYcall)
%     xlabel('Time (s)')
%     ylabel('Y vel at the tip (m/s)');
%     legend({'dy_c/dt'});

    %
   
    %% What is the x and y coordinate of the final configuration
    Final_Xcoord = Xcall(end);
    Final_Ycoord = Ycall(end);
    Final_velocity= (dXcall(end)*dXcall(end) + dYcall(end) * dYcall(end))^(1/2);
    Final_theta = atan(dYcall(end)/dXcall(end));

% end
end

function [Final_Xcoord]  = calculate_FinalXcoord(k1,k2,b1,b2)

    %% Define fixed parameters
    m1 = 1.;
    m2 = 1.;
    l1 = 1.;
    l2 = .5;
    c1 = 0.5;
    c2 = 0.25;
    I1 = 0.05;
    I2 = 0.05;


    g  = 9.81;    
    
    th10 =0;
    th20 =0;
    


    dtt = 0.01; %% time increment
    tf =3; %% final time

    p   = [l1; l2 ; c1; c2; m1; m2; I1; I2; g; ...
               th10 ; th20 ; k1; k2 ; b1; b2];  % parameters

    %% Q2.1 Perform simulation
    z0 = [th10 th20 0 0]';
    opts = odeset('AbsTol',1e-10,'RelTol',1e-8);
    sol = ode45(@dynamics,[0 tf],z0,opts,p);
    
    final_state = sol.y(:,end);
    
%     figure(1)
%     clf
%     %% Q2.2 Plot Angles
%     plot(sol.x,sol.y(1:2,:))
%     xlabel('Time (s)')
%     ylabel('Angle (rad)');
%     legend({'\theta_1','\theta_2'});
%         
%     figure(3)
%     clf
%     %% Q2.1 Animation
%     h_pend = plot([0,0],'LineWidth',4);
%     axis([-2 2 -2 2])
%     xlabel('x (m)');
%     ylabel('y (m)')
%     h_title = title('t=0.0s');

    count=0;
    Xcall=[];
    Ycall=[];
    dXcall=[];
    dYcall=[];
    ddYcall=[100];
    
    for t = 0:dtt:tf
        z = deval(sol,t);
        if (z(1)<pi) && (z(2)<pi) && (ddYcall(end)>=-g)
            z(1);
            keypoints = keypoints_double_pend_springs(z,p);   %% keypoints of the motion
%             h_pend.XData = [ keypoints(1,:) ];
%             h_pend.YData = [ keypoints(2,:) ];
            h_end.XData = z(1);
            h_end.YData = z(2);
            pause(.01);
            
%             h_title.String = sprintf('t=%.2fs',t);
            Xcall=[Xcall,keypoints(1,3)];
            Ycall=[Ycall,keypoints(2,3)];
            %% GET THE ACCELERATION AT THE HAND
            if count >=2
                ddYcall=[ddYcall,(Ycall(end)+Ycall(end-2)-2*Ycall(end-1))/dtt^2];
            elseif count>=1
                ddYcall=[ddYcall,0];
            end
            %% GET THE VELOCITY AT THE HAND
            if count >=1
                dYcall=[dYcall,(Ycall(end)-Ycall(end-1))/dtt];
                dXcall=[dXcall,(Xcall(end)-Xcall(end-1))/dtt];
            elseif count>=1
                dYcall=[dYcall,0];
                dXcall=[dXcall,0];
            end
            count = count+1;
        end
    end
    ddYcall = ddYcall(2:end);
%     %% plot the variaiton of acceleration at the hand over time
%     figure
%     plot(linspace(0,length(ddYcall),length(ddYcall))*dtt,ddYcall)
%     xlabel('Time (s)')
%     ylabel('Acc at the tip (rad/s^2)');
%     legend({'d^2y_c/dt^2'});
%     
%     figure
%     plot(linspace(0,length(dYcall),length(dYcall))*dtt,dYcall)
%     xlabel('Time (s)')
%     ylabel('Y vel at the tip (m/s)');
%     legend({'dy_c/dt'});

    %
   
    %% What is the x and y coordinate of the final configuration
    Final_Xcoord = Xcall(end);
    Final_Ycoord = Ycall(end);
    Final_velocity= (dXcall(end)*dXcall(end) + dYcall(end) * dYcall(end))^(1/2);
    Final_theta = atan(dYcall(end)/dXcall(end));

% end
end


function dz = dynamics(~,z,p)
        global tau1 tau2 
        A = A_double_pend_springs(z,p);        
        u = [tau1 tau2]';
        b = b_double_pend_springs(z,u,p);
        qdd = A\b;  %% acceleration
        dz = 0*z;
        dz(1:2) = z(3:4);
        dz(3:4) = qdd;       
end

function [count] = ball(Final_veloci, Final_thet, final_xcoord, final_ycoord )

close all;

v = Final_veloci;
theta0 = Final_thet;
disp([v, theta0])

H = 3.05; % 篮筐高度

% D = 7 + final_xcoord; %人与篮筐的距离
% h = 2.6 + final_ycoord; %出手高度

D = 3.5; %人与篮筐的距离
h = H - 0.8098; %出手高度


C = 0.0; %篮球阻力系数
p = 1.29; %空气密度
r = 0.123; % 篮球半径
m = 0.625; %篮球质量
residual = 0.102; %冗余


dt = 0.001; %时间间隔
Total = 2.5; %总时间



x0 = 0; %篮球初始水平距离
y0 = h; %篮球初始竖直距离

X_res = [x0]; 
Y_res = [h]; 
T_res = []; 

count = 0;

v1 = v;
theta1 = theta0;
for t1 = 0: dt: Total
    Ax = acceleration_x(C, p, r, v1, m, theta1);
    Ay = acceleration_y(C, p, r, v1, m, theta1);
    
    
    vx = v1 * cos(theta1); 
    vy = v1 * sin(theta1);
    
    
    x = X_distance(vx,dt,Ax,x0);
    y = Y_distance(vy,dt,Ay,y0);
    x0 = x;
    y0 = y;
    
    T_res = [T_res, t1];
    X_res = [X_res, x];
    Y_res = [Y_res, y];
    
    
    vx = Vx(vx, dt, Ax);
    vy = Vy(vy, dt, Ay);
   
    
    theta1 = degree(vx,vy);
    
    
    v1 = sqrt(vy * vy + vx * vx);
    
    
    if abs(x - D) <= residual  &&  abs(y - H) <= 0.1
        count = count + 1;
        break
    end

    if x - D > residual
        break
    end
    
    
end

if count == 1
    disp('有阻力，进了');
else
    disp('有阻力，美金');
end
end



