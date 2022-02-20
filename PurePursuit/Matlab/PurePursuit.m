% Purepursuit
% 作者：阿马
% 日期：20210518
close all;
clear all;
clc;

load  reftraj.mat

%% 相关参数定义
RefTraj = reftraj;       % 参考轨迹点
Control_Period = 0.05;   % 控制周期
L = 2.6;                 % 前后车轮轴距

%% 主程序
x = RefTraj(1,1);% 小车初始状态
y = RefTraj(1,2);
phi = RefTraj(1,3);
v = 0;
state = [x y phi v];
err_state_ini = [0 -1 -5/57.3 0];%小车初始位置偏差
state = state + err_state_ini;

idx_target = 1;

t = 0;
dt = Control_Period;

i = 1;
result_state(i,1) = t;
result_state(i,2) = state(1);
result_state(i,3) = state(2);
result_state(i,4) = state(3);
result_state(i,5) = state(4);
[TrackingErrorOfPosition, TrackingErrorOfAttitude] = CalculateTrackingError(RefTraj, state);
result_TrackingError(i,1) = t;
result_TrackingError(i,2) = TrackingErrorOfPosition;
result_TrackingError(i,3) = TrackingErrorOfAttitude;
% 循环遍历轨迹点
while idx_target < size(RefTraj,1)-2
    
    t = t+dt;
    
    [TrackingErrorOfPosition, TrackingErrorOfAttitude] = CalculateTrackingError(RefTraj, state);
    
    state_measured = UpdateStateMeasured(state);
    
    [v_command, delta_f_command, idx_target] = PID_add_FeedForwardControl(RefTraj, state_measured, Control_Period, L);
    
%     [v_command, delta_f_command, idx_target] = pure_pursuit_control(RefTraj, state_measured, L);
    
    state = UpdateState(v_command, delta_f_command, state, dt,L);
    
    result_state(i,1) = t;
    result_state(i,2) = state(1);
    result_state(i,3) = state(2);
    result_state(i,4) = state(3);
    result_state(i,5) = state(4);
    result_TrackingError(i,1) = t;
    result_TrackingError(i,2) = TrackingErrorOfPosition;
    result_TrackingError(i,3) = TrackingErrorOfAttitude;

    i = i+1;
    
    TrackingErrorOfPosition;

end

save result_state_PurePursuit result_state;
save result_TrackingError_PurePursuit result_TrackingError;

% 画图
figure('name','轨迹')
plot(RefTraj(:,1), RefTraj(:,2), 'b'); grid on;
xlabel('纵坐标 / m');
ylabel('横坐标 / m');
hold on 
for i = 1:size(result_state,1)
    scatter(result_state(i,2), result_state(i,3),80, '.r');
    pause(0.0001);
end
% plot(result_state(:,2), result_state(:,3),'.r');
legend('规划轨迹', '实际轨迹');

figure('name','跟踪误差')
subplot(2,1,1);
plot(result_TrackingError(:,1), result_TrackingError(:,2)*100, 'b'); grid on;
xlabel('时间 / s');
ylabel('位置误差 / cm');
subplot(2,1,2);
plot(result_TrackingError(:,1), result_TrackingError(:,3)*57.3, 'b'); grid on;
xlabel('时间 / s');
ylabel('姿态误差 / 度');

%% 计算控制量:速度和方向盘转角
function [v_command, delta_f_command, idx_target] = PID_add_FeedForwardControl(RefTraj, state_measured, Control_Period, L)
	SizeOfRefTraj = size(RefTraj,1);
    for i = 1:1:SizeOfRefTraj
        dist(i,1) = norm(RefTraj(i,1:2) - state_measured(1,1:2));   
    end
    [~,idx] = min(dist); % 距离最近轨迹点编号
    
    if idx == 1
        idx_target = idx;
    elseif idx == SizeOfRefTraj
        idx_target = idx-1;
    else
        if dist(idx-1)<dist(idx+1)
            idx_target = idx-1;
        else
            idx_target = idx;
        end
    end
    dist1 = dist(idx_target);
    dist2 = dist(idx_target+1);
    RefPoint = ([RefTraj(idx_target,1) RefTraj(idx_target,2) 0]*dist2+[RefTraj(idx_target+1,1) RefTraj(idx_target+1,2) 0]*dist1)/(dist1+dist2);
    Ref_Psi = (RefTraj(idx_target,3)*dist2+RefTraj(idx_target+1,3)*dist1)/(dist1+dist2);
    Ref_Cur = (RefTraj(idx_target,5)*dist2+RefTraj(idx_target+1,5)*dist1)/(dist1+dist2);
    Ref_Vel = (RefTraj(idx_target,4)*dist2+RefTraj(idx_target+1,4)*dist1)/(dist1+dist2);
%     RefPoint = [RefTraj(idx_target+1,1) RefTraj(idx_target+1,2) 0];
%     Ref_Psi = RefTraj(idx_target+1,3);
%     Ref_Cur = RefTraj(idx_target+1,5);
%     Ref_Vel = RefTraj(idx_target+1,4);
    Kp = 1.5; Ki0 = 1.8; Kd = 3.5;
    persistent wz_i_command;
    if isempty(wz_i_command)
        wz_i_command = 0;
    end
    err_y_command = 0;
    P = [state_measured(1) state_measured(2) 0];
    Q1 = RefPoint;
    while Ref_Psi > 2*pi
        Ref_Psi = Ref_Psi-2*pi;
    end
    while Ref_Psi < -2*pi
        Ref_Psi = Ref_Psi+2*pi;
    end
    if Ref_Psi>=0&&Ref_Psi<pi/2
        det_x = 1;
    elseif Ref_Psi>pi/2&&Ref_Psi<pi*3/2
        det_x = -1;
    elseif Ref_Psi>pi*3/2&&Ref_Psi<pi*2
        det_x = 1;
    end
    Q2 = [RefPoint(1)+det_x RefPoint(2)+det_x*tan(Ref_Psi) 0];
    err_y_vec = cross(Q1-P,Q2-Q1)/norm(Q2-Q1);
    err_y = err_y_vec(3);
    err_psi = state_measured(3)-Ref_Psi;
    if abs(Ref_Cur)<=0.1
        dis_i = 0.015;
    elseif abs(Ref_Cur)<=0.2
        dis_i = 0.015+0.04*(abs(Ref_Cur)-0.1)/(0.2-0.1);
    else
        dis_i = 0.055; % Todo
    end
    if abs(err_y)>=dis_i
        Ki = 0;
    else
        Ki = Ki0*abs(err_y)/dis_i;
    end
    if abs(err_y)<=dis_i
        Ki1 = 1.0;
    elseif abs(err_y)<=dis_i+0.02
        Ki1 = 1.0-(abs(err_y)-dis_i)/0.02;
    else
        Ki1 = 0.0;
    end
    CurVel_compensate = state_measured(4)*Ref_Cur; % 左转为正,右转为负
    wz_i_command = wz_i_command+Ki*(err_y_command-err_y)*Control_Period;
    wz_i_command = wz_i_command*Ki1;
    wz_command = Kp*(err_y_command-err_y)+wz_i_command-Kd*err_psi+CurVel_compensate*1.0;
    delta_f_command = atan(wz_command*L/state_measured(4));
%     [dis_i Ki wz_i_command]
    v_command = Ref_Vel;
end

%% 计算控制量:速度和方向盘转角
function [v_command, delta_f_command, idx_target] = pure_pursuit_control(RefTraj, state_measured, L)
    SizeOfRefTraj = size(RefTraj,1);
    for i = 1:1:SizeOfRefTraj
        dist(i,1) = norm(RefTraj(i,1:2) - state_measured(1,1:2));   
    end
    [~,idx] = min(dist); % 距离最近轨迹点编号
    
    Kv = 0.1;                % 前视距离系数
    Ld0 = 0.7;               % Ld0是预瞄距离的下限值
    Ld = Kv*state_measured(4) + Ld0;
    L_steps = 0;% 轨迹线长度
    while L_steps < Ld && idx < SizeOfRefTraj
        L_steps = L_steps + norm(RefTraj(idx+1,1:2) - RefTraj(idx,1:2));
        idx = idx+1;
    end
    idx_target = idx; % 预瞄轨迹点编号
    LookaheadPoint = RefTraj(idx_target,:);
    alpha = atan2(LookaheadPoint(1,2) - state_measured(1,2), LookaheadPoint(1,1) - state_measured(1,1))  - state_measured(1,3);

    delta_f_command = atan2(2*L*sin(alpha), Ld);
    v_command = LookaheadPoint(1,4);
end

%% 更新小车量测信息
function state_measured = UpdateStateMeasured(state)
    gate_noise = 0;
    err_measure_x = 0.02;
    err_measure_y = 0.02;
    err_measure_psi = 0.5/57.3;
    err_measure_v = 0.02;
    state_measured(1) = state(1) + gate_noise*err_measure_x; %横坐标
    state_measured(2) = state(2) + gate_noise*err_measure_y; %纵坐标
    state_measured(3) = state(3) + gate_noise*err_measure_psi; %横摆角
    state_measured(4) = state(4) + gate_noise*err_measure_v; %速度
end

%% 计算控跟踪误差:位置误差和姿态误差
function [TrackingErrorOfPosition, TrackingErrorOfAttitude] = CalculateTrackingError(RefTraj, state)
	SizeOfRefTraj = size(RefTraj,1);
    for i = 1:1:SizeOfRefTraj
        dist(i,1) = norm(RefTraj(i,1:2) - state(1,1:2));   
    end
    [~,idx] = min(dist); % 距离最近轨迹点编号
    
    if idx == 1
        idx_target = idx;
    elseif idx == SizeOfRefTraj
        idx_target = idx-1;
    else
        if dist(idx-1)<dist(idx+1)
            idx_target = idx-1;
        else
            idx_target = idx;
        end
    end
    dist1 = dist(idx_target);
    dist2 = dist(idx_target+1);
    RefPoint = ([RefTraj(idx_target,1) RefTraj(idx_target,2) 0]*dist2+[RefTraj(idx_target+1,1) RefTraj(idx_target+1,2) 0]*dist1)/(dist1+dist2);
    Ref_Psi = (RefTraj(idx_target,3)*dist2+RefTraj(idx_target+1,3)*dist1)/(dist1+dist2);
    P = [state(1) state(2) 0];
    Q1 = RefPoint;
    while Ref_Psi > 2*pi
        Ref_Psi = Ref_Psi-2*pi;
    end
    while Ref_Psi < -2*pi
        Ref_Psi = Ref_Psi+2*pi;
    end
    if Ref_Psi>=0&&Ref_Psi<pi/2
        det_x = 1;
    elseif Ref_Psi>pi/2&&Ref_Psi<pi*3/2
        det_x = -1;
    elseif Ref_Psi>pi*3/2&&Ref_Psi<pi*2
        det_x = 1;
    end
    Q2 = [RefPoint(1)+det_x RefPoint(2)+det_x*tan(Ref_Psi) 0];
    err_y_vec = cross(Q1-P,Q2-Q1)/norm(Q2-Q1);
    TrackingErrorOfPosition = err_y_vec(3);
    TrackingErrorOfAttitude = state(3)-Ref_Psi;
end

%% 更新小车状态量
function state_new = UpdateState(v_command,delta_f_command,state_old,dt,L)
    gate_model = 1; % 0为理想模型;1为含方向盘转角零位偏差的一阶模型;2为含反向盘转角偏差的近理想模型
    if gate_model==0
        state_new(1) = state_old(1) + state_old(4)*cos(state_old(3))*dt; %横坐标
        state_new(2) = state_old(2) + state_old(4)*sin(state_old(3))*dt; %纵坐标
        state_new(3) = state_old(3) + tan(delta_f_command)*state_old(4)/L*dt;    %横摆角
        state_new(4) = v_command;                                        %速度
    elseif gate_model==1
        Tv = 0.10; a = (v_command-state_old(4))/Tv;%变参--与运动速度和载荷相关
        persistent delta_f;
        if isempty(delta_f)
            delta_f=0;
        end
        err_delta_f0 = 0.5/57.3;
        Tw = 0.15; d_delta_f = (delta_f_command-delta_f)/Tw; delta_f = delta_f + d_delta_f*dt;
        state_new(1) = state_old(1) + state_old(4)*cos(state_old(3))*dt; %横坐标
        state_new(2) = state_old(2) + state_old(4)*sin(state_old(3))*dt; %纵坐标
        state_new(3) = state_old(3) + tan(delta_f+err_delta_f0)*state_old(4)/L*dt;    %横摆角
        state_new(4) = state_old(4) + a*dt;                              %速度
    elseif gate_model==2
        err_delta_f0 = 0.5/57.3;
        state_new(1) = state_old(1) + state_old(4)*cos(state_old(3))*dt; %横坐标
        state_new(2) = state_old(2) + state_old(4)*sin(state_old(3))*dt; %纵坐标
        state_new(3) = state_old(3) + tan(delta_f_command+err_delta_f0)*state_old(4)/L*dt;    %横摆角
        state_new(4) = v_command;                                        %速度
    end
end
    
