load("Pd.mat")
a = length(Pd); 
%% Given Parameters 
SOC_min = 30;
SOC_max = 80;
SOC_range = SOC_min:1:SOC_max; %States (30, 31,...,79, 80) Total 51
s_range=length(SOC_range); % States matrix
P_motor_max = 500; % Maximum Motor Power
P_motor_min = -500; % Minimum Motor Power
P_batt = P_motor_min:5:P_motor_max; %Controls (-500, -495...., 495, 500) Total 201
eta_elec = 0.9; % Effieciency of electrical motor (90%) 
Peng_max = 2000;  % Maximum Engine Power
Peng_min = 0; % Minimum Engine Power
%% Defining Cost Function
J_star = zeros(length(SOC_range), a); % Cost function matrix
U_elec = zeros(length(SOC_range), a); % Electric motor control input matrix
U_Eng = zeros(length(SOC_range), a); % Engine control input matrix
t = 1;
J_star(:, a) = 1e30; % Assign 1e30 in the last time step so we make the elements 
J_star(SOC_range == 70, end) = 0; % Assign 0 in the last column
% Deriving the cost function
for i = a-1:-1:1
    for j = 1:length(SOC_range) %Evaluate cost and optimal control inputs for each step at current time step
        x = SOC_range(j); % Current state value
        c_min = inf; % Initialize c_min at infinity
        for l = 1:length(P_batt)
            p_elec = P_batt(l);
            P_engnet = Pd(i) - p_elec * eta_elec; % Net load provided by the engine (demanded power -xx)
            etaeng = 0.3 - (P_engnet - 400).^2 / (400^2 / 0.3); % Engine efficiency
            
            etaeng=max(etaeng, 0.05); % Minimum allowed efficiency is 5%
            Peng = P_engnet / etaeng; % Power output of the engine
            if Peng >= Peng_min+50 && Peng <= Peng_max-50 % Engine min & max power

                x_dot=- (p_elec / 36000) * 100; % Rate of change of batt SOC (Plant)
                x1 = x + x_dot; % New State
                if x1 >= SOC_min && x1 <= SOC_max

                    cost = Peng; % Current control input's cost
                    next_cost = J_star(SOC_range == round(x1), i+1); % Next cost from next step i+1 rounding current state
                    cost_sum = cost + next_cost;
                    if cost_sum < c_min 
                        c_min = cost_sum;
                        opt_elec = p_elec;
                        opt_eng = Peng;
                    end
                end
            end
        end
        J_star(j, i) = c_min; 
        U_elec(j, i) = opt_elec;
        U_Eng(j, i) = opt_eng;
    end
end
%% Optimal Trajectory
opt_soc = zeros(1,a);
u_opt_elec = zeros(1,a)';
u_opt_eng = zeros(1,a)';
% Initial and Final SOC
opt_soc(1) = 70;
opt_soc(end) = 70;
for k = 1:a - 1
    rounded_opt_soc = round(opt_soc(k));
    soc_id = (SOC_range == rounded_opt_soc);
    if ~any(soc_id)
        soc_id = 1;
    end
    u_opt_elec(k) = U_elec(soc_id, k);
    u_opt_eng(k) = U_Eng(soc_id, k);
    opt_soc_increment = (-u_opt_elec(k) / 36000) * 100;
    opt_soc(k + 1) = opt_soc(k) + opt_soc_increment;
    cost_opt(k) = J_star(soc_id, k);
end
%% Results
% Optimal SOC
figure;
plot(opt_soc,LineWidth=1)
title('Optimal State of Charge Trajectory ');
xlabel('Time (s)');
ylabel('State of Charge (%)');
ylim([0,100])
grid on
hold off
% Electric Motor Power
figure;
plot(u_opt_elec,LineWidth=0.5)
title('Optimal Electric Motor Power Trajectory');
xlabel('Time (s)');
ylabel('Power (kW)');
grid on
hold off
% Engine Power
figure;
plot(u_opt_eng,LineWidth=0.5)
title('Optimal Power of Engine Trajectory');
xlabel('Time (s)');
ylabel('Power (kW)');
grid on
hold off
% Cost associated with Optimal Trajectory
figure;
plot(cost_opt,LineWidth=0.2)
title('Cost Associated With the Optimal Trajectory');
xlabel('Time (s)');
ylabel('Cost');
grid on
hold off