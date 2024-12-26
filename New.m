load("Pd.mat")
a = length(Pd); 
%% Given Parameters
SOC_min = 30;
SOC_max = 80;
SOC_range = SOC_min:1:SOC_max;
s_range=length(SOC_range);
P_motor_max = 500;
P_motor_min = -500;
P_batt = P_motor_min:5:P_motor_max;
eta_elec = 0.9;
Peng_max = 2000;
Peng_min = 0;
%% Defining Cost Function
J_star = zeros(length(SOC_range), a);
U_elec = zeros(length(SOC_range), a);
U_Eng = zeros(length(SOC_range), a);
t = 1;
J_star(:, a) = 1e30;
J_star(SOC_range == 70, end) = 0;
% Deriving the cost function
for i = a-1:-1:1
    for j = 1:length(SOC_range)
        x = SOC_range(j);
        c_min = inf;
        for l = 1:length(P_batt)
            p_elec = P_batt(l);
            P_engnet = Pd(i) - p_elec * eta_elec; 
            etaeng = 0.3 - (P_engnet - 400).^2 / (400^2 / 0.3);
            
            etaeng=max(etaeng, 0.05);
            Peng = P_engnet / etaeng;
            if Peng >= Peng_min+50 && Peng <= Peng_max-50

                x_dot=- (p_elec / 36000) * 100;
                x1 = x + x_dot;
                if x1 >= SOC_min && x1 <= SOC_max

                    cost = Peng;
                    next_cost = J_star(SOC_range == round(x1), i+1);
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