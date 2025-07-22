function [ mean_delay, V_num, Arr_all ]...
    = Simulation_RC( d, delta, L, v_max, W, L_z)
% Simulation program for the original RC
% Input: d - demand vector for eight approaches
%        delta - minimum allowable distance between vehicles
%        L - vehicle length
%        v_max - maximum vehicle speed
%        W - vehicle width
%        L_z - length of the adjusting zone
% Output: mean_delay - average vehicle delay
%         V_num - number of vehicles generated in the simulation horizon
%         Arr_all - Number of vehicles admitted to enter the intersection
%         before 3600 seconds

%% Initialization
TimeHor = 10000;
Simulation_end_time = 3950;

gap = 2*(L+W+1.4*delta)/v_max; % Gap = 2 beats

Lane_id = [1;1;1;2;2;2;3;3;3;4;4;4;5;5;6;6;7;7;8;8];

%% Admitted entrance time for each lane
ET = cell(20,1);
for k = 1:4
    S = 0:gap:TimeHor; S = S';
    ET{3*(k-1)+1,1} = S;
    S = gap/2:gap:TimeHor; S = S';
    ET{3*(k-1)+2,1} = S;
    S = 0:gap:TimeHor; S = S';
    ET{3*(k-1)+3,1} = S;
end

for k = 1:4
    S = 0:gap:TimeHor; S = S';
    ET{12+2*(k-1)+1,1} = S;
    S = gap/2:gap:TimeHor; S = S';
    ET{12+2*(k-1)+2,1} = S;
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
    Arr_k = Arr{k,1};
    ET_k = ET{k,1};
    for i = 1:size(Arr_k,1)
        Arr_c = Arr_k(i,1);
        usable_id = min(find(ET_k>=Arr_c+t_min));
        ET_c = ET_k(usable_id,1);
        delay = [delay; ET_c - t_min - Arr_c];
        ET_v = [ET_v; ET_c];
        ET_k = ET_k(usable_id+1:end,1);
    end
end

%% Results
mean_delay = mean(delay);
V_num = size(delay,1);
Arr_all = find(ET_v(:,1) < 3600);
Arr_all = size(Arr_all,1);

end

