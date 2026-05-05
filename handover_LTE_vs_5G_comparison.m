%% =========================================================
%  Project 7: Optimization of Handover Mechanisms in 5G V2X
%  Step 3: LTE vs 5G Handover Efficiency Comparison
%% =========================================================

clc; clear; close all;

%% --- 1. SIMULATION PARAMETERS ---
sim_time  = 60;         % seconds
dt        = 0.1;
t         = 0:dt:sim_time;
nSteps    = length(t);

speeds_kmh = [60, 100, 120, 160, 200];
nSpeeds    = length(speeds_kmh);

% Base station layout (highway)
BS_positions = 0:300:3000;
nBS          = length(BS_positions);

%% --- 2. LTE vs 5G TECHNOLOGY PARAMETERS ---
%
%  Key differences:
%  - 5G has lower latency (faster X2/Xn interface)
%  - 5G has shorter TTT due to faster measurement reporting
%  - 5G uses higher frequency -> faster signal decay but beamforming gain
%  - LTE has larger coverage per cell but slower HO execution

tech(1).name          = 'LTE';
tech(1).freq_GHz      = 1.8;      % LTE Band 3
tech(1).Tx_dBm        = 46;
tech(1).TTT           = 0.160;    % 160ms TTT (3GPP TS 36.331)
tech(1).HO_prep_ms    = 50;       % ms
tech(1).HO_exec_ms    = 40;       % ms
tech(1).HO_margin_dB  = 3;
tech(1).coverage_m    = 500;
tech(1).color         = [0.2 0.4 0.8];   % Blue

tech(2).name          = '5G NR';
tech(2).freq_GHz      = 3.5;      % 5G n78 band
tech(2).Tx_dBm        = 46;
tech(2).TTT           = 0.040;    % 40ms TTT (3GPP TS 38.331)
tech(2).HO_prep_ms    = 10;       % ms
tech(2).HO_exec_ms    = 20;       % ms
tech(2).HO_margin_dB  = 3;
tech(2).coverage_m    = 300;
tech(2).color         = [0.85 0.33 0.1]; % Orange

nTech = length(tech);

%% --- 3. HELPER FUNCTIONS ---

function pl = path_loss(dist_m, freq_GHz)
    dist_m = max(dist_m, 1);
    pl = 28 + 22*log10(dist_m) + 20*log10(freq_GHz);
end

function rsrp = compute_RSRP(dist_m, Tx_dBm, freq_GHz)
    pl   = path_loss(dist_m, freq_GHz);
    rsrp = Tx_dBm - pl + randn()*2;
end

function p_fail = ho_failure_prob(speed_kmh, rsrp_dBm, ttt_s)
    % Faster TTT = fewer failures at high speed
    speed_factor  = speed_kmh / 200;
    signal_factor = max(0, (-rsrp_dBm - 90) / 50);
    ttt_factor    = ttt_s / 0.160;   % normalized to LTE TTT
    p_fail = min(0.05 + 0.25*speed_factor*ttt_factor + 0.15*signal_factor, 0.7);
end

function lat = ho_latency_ms(speed_kmh, success, prep_ms, exec_ms)
    base  = prep_ms + exec_ms;
    spd_p = speed_kmh * 0.05;
    if ~success
        lat = base * 3 + spd_p;
    else
        lat = base + spd_p;
    end
end

function tput = throughput_mbps(rsrp_dBm, during_ho, tech_name)
    % Shannon-like throughput estimate
    snr_dB  = rsrp_dBm + 120;           % rough SNR estimate
    snr_lin = 10^(snr_dB/10);
    bw_hz   = 20e6;                      % 20MHz bandwidth
    if strcmp(tech_name,'5G NR')
        bw_hz = 100e6;                   % 5G uses 100MHz
    end
    tput = (bw_hz * log2(1 + snr_lin)) / 1e6;  % Mbps
    if during_ho
        tput = tput * 0.1;  % 90% throughput drop during HO
    end
    tput = max(tput, 0);
end

%% --- 4. MAIN COMPARISON LOOP ---
fprintf('Running LTE vs 5G Handover Comparison...\n\n');

% Pre-allocate results
for ti = 1:nTech
    for si = 1:nSpeeds
        res(ti,si).nHO       = 0;
        res(ti,si).nFail     = 0;
        res(ti,si).fail_rate = 0;
        res(ti,si).avg_lat   = 0;
        res(ti,si).avg_tput  = 0;
        res(ti,si).latencies = [];
        res(ti,si).tputs     = [];
        res(ti,si).pp_count  = 0;  % ping-pong count
    end
end

for ti = 1:nTech
    T = tech(ti);
    for si = 1:nSpeeds
        speed_kmh = speeds_kmh(si);
        speed_ms  = speed_kmh / 3.6;
        veh_pos   = speed_ms * t;

        serving_BS  = 1;
        TTT_counter = 0;
        ttt_active  = false;
        candidate_BS= 0;
        last_HO_BS  = 0;

        ho_lats   = [];
        ho_succ   = [];
        tput_log  = zeros(1,nSteps);
        pp_count  = 0;
        in_ho     = false;
        ho_dur    = 0;

        for k = 1:nSteps
            pos      = veh_pos(k);
            dist_srv = abs(pos - BS_positions(serving_BS));
            rsrp_srv = compute_RSRP(dist_srv, T.Tx_dBm, T.freq_GHz);

            % Best neighbor RSRP
            rsrp_all = zeros(1,nBS);
            for b = 1:nBS
                d = abs(pos - BS_positions(b));
                rsrp_all(b) = compute_RSRP(d, T.Tx_dBm, T.freq_GHz);
            end
            rsrp_all(serving_BS) = -Inf;
            [rsrp_best, best_BS] = max(rsrp_all);

            % Throughput (drops during HO execution)
            tput_log(k) = throughput_mbps(rsrp_srv, in_ho, T.name);
            if in_ho
                ho_dur = ho_dur + dt;
                if ho_dur*1000 >= T.HO_exec_ms
                    in_ho  = false;
                    ho_dur = 0;
                end
            end

            % A3 Event
            if rsrp_best > rsrp_srv + T.HO_margin_dB
                if ~ttt_active
                    ttt_active   = true;
                    TTT_counter  = 0;
                    candidate_BS = best_BS;
                else
                    TTT_counter = TTT_counter + dt;
                end
            else
                ttt_active  = false;
                TTT_counter = 0;
            end

            % Handover trigger
            if ttt_active && TTT_counter >= T.TTT
                ttt_active  = false;
                TTT_counter = 0;

                p_fail  = ho_failure_prob(speed_kmh, rsrp_srv, T.TTT);
                success = rand() > p_fail;
                lat     = ho_latency_ms(speed_kmh, success, ...
                                        T.HO_prep_ms, T.HO_exec_ms);
                ho_lats(end+1) = lat;
                ho_succ(end+1) = success;

                % Ping-pong: HO back to previous BS within short time
                if success && candidate_BS == last_HO_BS
                    pp_count = pp_count + 1;
                end

                if success
                    last_HO_BS = serving_BS;
                    serving_BS = candidate_BS;
                    in_ho      = true;
                    ho_dur     = 0;
                end
            end
        end

        % Store
        nHO = length(ho_lats);
        res(ti,si).nHO       = nHO;
        res(ti,si).nFail     = sum(ho_succ == 0);
        res(ti,si).fail_rate = sum(ho_succ==0)/max(nHO,1)*100;
        res(ti,si).avg_lat   = mean(ho_lats);
        res(ti,si).avg_tput  = mean(tput_log);
        res(ti,si).latencies = ho_lats;
        res(ti,si).tputs     = tput_log;
        res(ti,si).pp_count  = pp_count;

        fprintf('[%s] Speed: %3d km/h | HOs: %2d | Fail: %.1f%% | Lat: %.1fms | Tput: %.1fMbps | PP: %d\n', ...
            T.name, speed_kmh, nHO, res(ti,si).fail_rate, ...
            res(ti,si).avg_lat, res(ti,si).avg_tput, pp_count);
    end
    fprintf('\n');
end

%% --- 5. EXTRACT METRICS FOR PLOTTING ---
fail_lte = [res(1,:).fail_rate];
fail_5g  = [res(2,:).fail_rate];
lat_lte  = [res(1,:).avg_lat];
lat_5g   = [res(2,:).avg_lat];
tput_lte = [res(1,:).avg_tput];
tput_5g  = [res(2,:).avg_tput];
pp_lte   = [res(1,:).pp_count];
pp_5g    = [res(2,:).pp_count];
nho_lte  = [res(1,:).nHO];
nho_5g   = [res(2,:).nHO];

x = 1:nSpeeds;
xlabels = arrayfun(@(s) sprintf('%d',s), speeds_kmh, 'UniformOutput', false);

%% --- 6. COMPARISON PLOTS ---
figure('Name','LTE vs 5G Handover Comparison','NumberTitle','off', ...
       'Position',[50 50 1400 950]);

%-- Plot 1: Failure Rate --
subplot(3,2,1);
b = bar(x, [fail_lte; fail_5g]', 0.6);
b(1).FaceColor = tech(1).color;
b(2).FaceColor = tech(2).color;
set(gca,'XTickLabel', xlabels);
xlabel('Speed (km/h)'); ylabel('Failure Rate (%)');
title('Handover Failure Rate'); legend('LTE','5G NR'); grid on;

%-- Plot 2: Average Latency --
subplot(3,2,2);
b = bar(x, [lat_lte; lat_5g]', 0.6);
b(1).FaceColor = tech(1).color;
b(2).FaceColor = tech(2).color;
set(gca,'XTickLabel', xlabels);
xlabel('Speed (km/h)'); ylabel('Latency (ms)');
title('Average Handover Latency'); legend('LTE','5G NR'); grid on;

%-- Plot 3: Average Throughput --
subplot(3,2,3);
b = bar(x, [tput_lte; tput_5g]', 0.6);
b(1).FaceColor = tech(1).color;
b(2).FaceColor = tech(2).color;
set(gca,'XTickLabel', xlabels);
xlabel('Speed (km/h)'); ylabel('Throughput (Mbps)');
title('Average Throughput'); legend('LTE','5G NR'); grid on;

%-- Plot 4: Ping-Pong Handovers --
subplot(3,2,4);
b = bar(x, [pp_lte; pp_5g]', 0.6);
b(1).FaceColor = tech(1).color;
b(2).FaceColor = tech(2).color;
set(gca,'XTickLabel', xlabels);
xlabel('Speed (km/h)'); ylabel('Ping-Pong Count');
title('Ping-Pong Handovers'); legend('LTE','5G NR'); grid on;

%-- Plot 5: Total Handovers --
subplot(3,2,5);
b = bar(x, [nho_lte; nho_5g]', 0.6);
b(1).FaceColor = tech(1).color;
b(2).FaceColor = tech(2).color;
set(gca,'XTickLabel', xlabels);
xlabel('Speed (km/h)'); ylabel('Number of HOs');
title('Total Handovers'); legend('LTE','5G NR'); grid on;

%-- Plot 6: Latency Line Comparison --
subplot(3,2,6);
plot(speeds_kmh, lat_lte, 'b-o', 'LineWidth',2,'MarkerSize',7); hold on;
plot(speeds_kmh, lat_5g,  '-o',  'LineWidth',2,'MarkerSize',7, ...
     'Color', tech(2).color);
xlabel('Speed (km/h)'); ylabel('Latency (ms)');
title('Latency Trend: LTE vs 5G NR');
legend('LTE','5G NR','Location','northwest'); grid on;

sgtitle('Step 3: LTE vs 5G NR Handover Efficiency Comparison — 5G V2X');

%% --- 7. IMPROVEMENT SUMMARY TABLE ---
fprintf('============================================================\n');
fprintf('        LTE vs 5G NR — PERFORMANCE IMPROVEMENT SUMMARY\n');
fprintf('============================================================\n');
fprintf('%-12s | %-10s %-10s | %-10s %-10s | %-12s\n', ...
    'Speed(km/h)','LTE Fail%','5G Fail%','LTE Lat ms','5G Lat ms','Lat Improve%');
fprintf('%s\n', repmat('-',1,68));
for si = 1:nSpeeds
    lat_imp = (lat_lte(si) - lat_5g(si)) / lat_lte(si) * 100;
    fprintf('%-12d | %-10.1f %-10.1f | %-10.1f %-10.1f | %-12.1f\n', ...
        speeds_kmh(si), fail_lte(si), fail_5g(si), ...
        lat_lte(si), lat_5g(si), lat_imp);
end
fprintf('============================================================\n');
fprintf('\nAvg Throughput — LTE: %.1f Mbps | 5G NR: %.1f Mbps\n', ...
    mean(tput_lte), mean(tput_5g));
fprintf('Avg Latency    — LTE: %.1f ms   | 5G NR: %.1f ms\n', ...
    mean(lat_lte),  mean(lat_5g));
fprintf('Overall Latency Improvement: %.1f%%\n', ...
    (mean(lat_lte)-mean(lat_5g))/mean(lat_lte)*100);
