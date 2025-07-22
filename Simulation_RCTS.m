function [ mean_delay, V_num, Arr_all ] = ...
    Simulation_RCTS( d, delta, L, v_max, W, L_z, loss, N_1, N_2, N_3)
% Simulation program for the hybrid RC-TS
% Input: d - demand vector for eight approaches
%        delta - minimum allowable distance between vehicles
%        L - vehicle length
%        v_max - maximum vehicle speed
%        W - vehicle width
%        L_z - length of the adjusting zone
%        loss - phase transition loss
%        N_1 - Number of entry time points for RC phase in one cycle
%        N_2 - Number of entry time points for signal phase with the
%        largest demand
%        N_3 - Number of entry time points for signal phase with the second
%        largest demand
% Output: mean_delay - average vehicle delay
%         V_num - number of vehicles generated in the simulation horizon
%         Arr_all - Number of vehicles admitted to enter the intersection
%         before 3600 seconds

%% Initialization
TimeHor = 10000;
Simulation_end_time = 3950;
gap = (L+delta)/v_max; % Gap = minimum allowable following headway
gap_A = 2*(L+W+1.4*delta)/v_max; % Gap_A = 2 RC beats
Lane_id = [1;1;1;2;2;2;3;3;3;4;4;4;5;5;6;6;7;7;8;8];
Direc_seq = [1;2;1;2;3;4;3;4];
Nt = [N_1;N_2;N_3];

phase_num = size(find(Nt(:,1)>0),1) - 1;
c_length = (phase_num+1)*loss + (N_1-1)*gap_A + max(N_2-1,0)*gap + max(N_3-1,0)*gap;
lambda = [(N_1-1)*gap_A/c_length; max(N_2-1,0)*gap/c_length; max(N_3-1,0)*gap/c_length];

d_m1 = max(d(1,1),d(3,1));
d_m2 = max(d(2,1),d(4,1));
d_m3 = max(d(5,1),d(7,1));
d_m4 = max(d(6,1),d(8,1));

[d_m_tilde, d_index] = sort([d_m1;d_m2;d_m3;d_m4],'descend');

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

if phase_num == 0
    [ mean_delay, V_num, Arr_all ] = Simulation_RC( d, delta, L, v_max, W, L_z);
elseif phase_num == 1
    t_1 = 0;
    t_2 = lambda(1,1)*c_length + loss;
    ET_1 = cell(20,1);
    ET_2 = cell(20,1);
    ET_LACS_1 = cell(20,1);
    ET_LACS_2 = cell(20,1);
    for k = 1:20
        Rs = Lane_id(k,1);
        phase = Direc_seq(Rs,1);
        if phase == d_index(1,1)
            S = t_2:c_length:TimeHor+c_length; S = S'; ET_1{k,1} = S;
            Sa = c_length-loss:c_length:TimeHor+c_length; Sa = Sa'; ET_2{k,1} = Sa;
        end
        S = t_1:c_length:TimeHor+c_length; S = S';
        ET_LACS_1{k,1} = S;
        Sa = t_2-loss:c_length:TimeHor+c_length; Sa = Sa';
        ET_LACS_2{k,1} = Sa;
    end
    
    %RC entry time
    ET = cell(20,1);
    for k = 1:4
        S = 0:gap_A:TimeHor; S = S';
        ET{3*(k-1)+1,1} = S;
        S = gap_A/2:gap_A:TimeHor; S = S';
        ET{3*(k-1)+2,1} = S;
        S = 0:gap_A:TimeHor; S = S';
        ET{3*(k-1)+3,1} = S;
    end
    
    for k = 1:4
        S = 0:gap_A:TimeHor; S = S';
        ET{12+2*(k-1)+1,1} = S;
        S = gap_A/2:gap_A:TimeHor; S = S';
        ET{12+2*(k-1)+2,1} = S;
    end
    
    for k = 1:20
        RR = ET{k,1};
        RR_id = zeros(size(RR,1),1);
        E1 = ET_LACS_1{k,1};
        E2 = ET_LACS_2{k,1};
        for i = 1:size(RR,1)
            LB = max(find(E1<=RR(i,1)));
            UB = min(find(E2>RR(i,1)));
            if LB == UB
                RR_id(i,1) = 1;
            end
        end
        RR = RR(find(RR_id(:,1) == 1),1);
        ET{k,1} = RR;
    end
    
elseif phase_num == 2
    
    t_1 = 0;
    t_2 = lambda(1,1)*c_length + loss;
    t_3 = lambda(1,1)*c_length + lambda(3,1)*c_length + 2 * loss;
    ET_1 = cell(20,1);
    ET_2 = cell(20,1);
    ET_LACS_1 = cell(20,1);
    ET_LACS_2 = cell(20,1);
    for k = 1:20
        Rs = Lane_id(k,1);
        phase = Direc_seq(Rs,1);
        if phase == d_index(1,1)
            S = t_3:c_length:TimeHor+c_length; S = S'; ET_1{k,1} = S;
            Sa = c_length-loss:c_length:TimeHor+c_length; Sa = Sa'; ET_2{k,1} = Sa;
        elseif phase == d_index(2,1)
            S = t_2:c_length:TimeHor+c_length; S = S'; ET_1{k,1} = S;
            Sa = t_3-loss:c_length:TimeHor+c_length; Sa = Sa'; ET_2{k,1} = Sa;
        end
        S = t_1:c_length:TimeHor+c_length; S = S';
        ET_LACS_1{k,1} = S;
        Sa = t_2-loss:c_length:TimeHor+c_length; Sa = Sa';
        ET_LACS_2{k,1} = Sa;
    end
    
    %RC entry time
    ET = cell(20,1);
    for k = 1:4
        S = 0:gap_A:TimeHor; S = S';
        ET{3*(k-1)+1,1} = S;
        S = gap_A/2:gap_A:TimeHor; S = S';
        ET{3*(k-1)+2,1} = S;
        S = 0:gap_A:TimeHor; S = S';
        ET{3*(k-1)+3,1} = S;
    end
    
    for k = 1:4
        S = 0:gap_A:TimeHor; S = S';
        ET{12+2*(k-1)+1,1} = S;
        S = gap_A/2:gap_A:TimeHor; S = S';
        ET{12+2*(k-1)+2,1} = S;
    end
    
    for k = 1:20
        RR = ET{k,1};
        RR_id = zeros(size(RR,1),1);
        E1 = ET_LACS_1{k,1};
        E2 = ET_LACS_2{k,1};
        for i = 1:size(RR,1)
            LB = max(find(E1<=RR(i,1)));
            UB = min(find(E2>RR(i,1)));
            if LB == UB
                RR_id(i,1) = 1;
            end
        end
        RR = RR(find(RR_id(:,1) == 1),1);
        ET{k,1} = RR;
    end
    
end

%% Simulation

if phase_num > 0.5
    delay = [];
    delay_2 = [];
    ET_v = [];
    t_min = L_z/v_max;
    
    for k = 1:20
        Last_v = 0;
        Arr_k = Arr{k,1};
        ETE_c = ET_1{k,1};
        ETO_c = ET_2{k,1};
        ET_k = ET{k,1};
        if ~isempty(ETE_c)
            for i = 1:size(Arr_k,1)
                Arr_c = Arr_k(i,1);
                E_t_1 = Arr_c + t_min;
                E_t_2 = Last_v + gap;
                E_t = max(E_t_1, E_t_2);
                LB = max(find(ETE_c<=E_t));
                UB = min(find(ETO_c>E_t));
                if LB < UB | isempty(LB)
                    LB_a = min(find(ET_k>=E_t));
                    E_t_a = ET_k(LB_a,1);
                    LB_n = min(find(ETE_c>E_t));
                    E_t_n = ETE_c(LB_n,1);
                    if ~isempty(E_t_a) && ~isempty(E_t_n)
                        E_t = min(E_t_a, E_t_n);
                    elseif isempty(E_t_a)
                        E_t = E_t_n;
                    elseif isempty(E_t_n)
                        E_t = E_t_a;
                    end
                end
                Last_v = E_t;
                delay = [delay; E_t - Arr_c - t_min];
                ET_v = [ET_v; E_t];
                if k < 1.5
                    delay_2 = [delay_2; Arr_c, E_t - Arr_c - t_min];
                end
            end
        else
            for i = 1:size(Arr_k,1)
                Arr_c = Arr_k(i,1);
                E_t_1 = Arr_c + t_min;
                E_t_2 = Last_v + gap;
                E_t = max(E_t_1, E_t_2);
                LB_a = min(find(ET_k>=E_t));
                E_t_a = ET_k(LB_a,1);
                E_t = E_t_a;
                Last_v = E_t;
                delay = [delay; E_t - Arr_c - t_min];
                ET_v = [ET_v; E_t];
                if k < 1.5
                    delay_2 = [delay_2; Arr_c, E_t - Arr_c - t_min];
                end
            end
        end
    end
    
    
    %% Results
    mean_delay = mean(delay);
    V_num = size(delay,1);
    Arr_all = find(ET_v(:,1) < 3600);
    Arr_all = size(Arr_all,1);
end

end

