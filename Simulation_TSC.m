function [ mean_delay, V_num, Arr_all ] = Simulation_TSC( d, delta, L, v_max, L_z )
% Simulation program for the traffic signal control
% Input: d - demand vector for eight approaches
%        delta - minimum allowable distance between vehicles
%        L - vehicle length
%        v_max - maximum vehicle speed
%        L_z - length of the adjusting zone
% Output: mean_delay - average vehicle delay
%         V_num - number of vehicles generated in the simulation horizon
%         Arr_all - Number of vehicles admitted to enter the intersection
%         before 3600 seconds

%% Initialization
waste = 2; % Phase transition loss
d_m1 = max(d(1,1),d(3,1));
d_m2 = max(d(2,1),d(4,1));
d_m3 = max(d(5,1),d(7,1));
d_m4 = max(d(6,1),d(8,1));
gap = (L+delta)/v_max; % Gap = minimum allowable following headway
saturation = 1/gap * 3600;
sat_rate = (d_m1 + d_m2 + d_m3 + d_m4)/saturation;
if sat_rate < 1
    T = (1.5 * 4 * waste + 5)/(1-sat_rate);
    T = max(20,T);
    T = min(180,T);
else
    T = 180; % Maximum cycle length
end


TimeHor = 10000;
Simulation_end_time = 3950;
Lane_id = [1;1;1;2;2;2;3;3;3;4;4;4;5;5;6;6;7;7;8;8];
T_eff = T - 4*waste; % Effective green time

%% Signal timing
t_1 = d_m1/(d_m1+d_m2+d_m3+d_m4) * T_eff;
t_2 = d_m2/(d_m1+d_m2+d_m3+d_m4) * T_eff;
t_3 = d_m3/(d_m1+d_m2+d_m3+d_m4) * T_eff;
t_4 = d_m4/(d_m1+d_m2+d_m3+d_m4) * T_eff;
ET_1 = cell(20,1);
ET_2 = cell(20,1);
for k = 1:20
    Rs = Lane_id(k,1);
    if Rs == 1 || Rs == 3
        S = 0:T:TimeHor+T; S = S'; ET_1{k,1} = S;
        Sa = t_1:T:TimeHor+T; Sa = Sa'; ET_2{k,1} = Sa;
    elseif Rs == 2 || Rs == 4
        S = t_1+waste:T:TimeHor+T; S = S'; ET_1{k,1} = S;
        Sa = t_1+t_2+waste:T:TimeHor+T; Sa = Sa'; ET_2{k,1} = Sa;
    elseif Rs == 5 || Rs == 7
        S = t_1+t_2+2*waste:T:TimeHor+T; S = S'; ET_1{k,1} = S;
        Sa = t_1+t_2+t_3+2*waste:T:TimeHor+T; Sa = Sa'; ET_2{k,1} = Sa;
    elseif Rs == 6 || Rs == 8
        S = t_1+t_2+t_3+3*waste:T:TimeHor+T; S = S'; ET_1{k,1} = S;
        Sa = t_1+t_2+t_3+t_4+3*waste:T:TimeHor+T; Sa = Sa'; ET_2{k,1} = Sa;
    end
end

%% Arrival generation: a shifted Poisson's process
Arr = cell(20,1);
for k = 1:20
    d_k = d(Lane_id(k,1),1);
    S = 0;
    q = 0;
    while q == 0
        Sa = exprnd((3600-(L+delta)/v_max*d_k)/d_k);
        if S(end,1)+(L+delta)/v_max+Sa > Simulation_end_time
            q = 1;
            continue;
        end
        S = [S; S(end,1)+(L+delta)/v_max+Sa];
    end
    Arr{k,1} = S;
end

%% Simulation
delay = [];
ET_v = [];
t_min = L_z/v_max;
for k = 1:20
    Last_v = 0;
    Arr_k = Arr{k,1};
    ETE_c = ET_1{k,1};
    ETO_c = ET_2{k,1};
    for i = 1:size(Arr_k,1)
        Arr_c = Arr_k(i,1);
        E_t_1 = Arr_c + t_min;
        E_t_2 = Last_v + gap;
        E_t = max(E_t_1, E_t_2);
        LB = max(find(ETE_c<=E_t));
        UB = min(find(ETO_c>E_t));
        if LB < UB | isempty(LB)
            LB_n = min(find(ETE_c>E_t));
            E_t = ETE_c(LB_n,1);
        end
        Last_v = E_t;
        delay = [delay; E_t - Arr_c - t_min];
        ET_v = [ET_v; E_t];
    end
end


%% Results
mean_delay = mean(delay);
V_num = size(delay,1);
Arr_all = find(ET_v(:,1) < 3600);
Arr_all = size(Arr_all,1);

end

