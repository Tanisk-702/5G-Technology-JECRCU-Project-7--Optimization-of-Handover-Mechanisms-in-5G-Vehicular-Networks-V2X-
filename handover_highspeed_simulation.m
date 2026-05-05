%% =========================================================
%  Project 7: Optimization of Handover Mechanisms in 5G V2X
%  Step 2: Simulate Handover Failures & Latency at High Speed
%  =========================================================

clc; clear; close all;

%% --- 1. SIMULATION PARAMETERS ---
sim_time     = 60;        % Total simulation time (seconds)
dt           = 0.1;       % Time step (seconds)
t            = 0:dt:sim_time;
nSteps       = length(t);

% Vehicle speeds to test (km/h)
speeds_kmh   = [60, 100, 120, 160, 200];
nSpeeds      = length(speeds_kmh);

% Base Station (BS) layout along a highway (meters)
% Vehicle moves in a straight line; BSs placed every 300m
BS_positions = 0:300:3000;   % 11 base stations
nBS          = length(BS_positions);

% Coverage radius of each BS (meters)
coverage_radius = 180;       % Overlap zone starts at 150m

% 5G Handover Parameters
HO_execution_time  = 0.020;  % 20ms  - 5G handover execution delay
HO_preparation_time= 0.010;  % 10ms  - 5G preparation phase
TTT                = 0.040;  % 40ms  - Time-To-Trigger (hysteresis timer)
HO_margin          = 3;      % 3 dB  - Hysteresis margin (A3 event)

% Path loss model (simplified urban macro)
% RSRP(dBm) = Tx_power - PathLoss
Tx_power_dBm = 46;           % 5G gNB transmit power (dBm)
freq_GHz     = 3.5;          % 5G mid-band frequency

%% --- 2. HELPER FUNCTIONS ---

% Path loss (Urban Macro - 3GPP TR 38.901)
function pl = path_loss(dist_m, freq_GHz)
    dist_m = max(dist_m, 1);  % avoid log(0)
    pl = 28 + 22*log10(dist_m) + 20*log10(freq_GHz);
end

% RSRP from distance
function rsrp = compute_RSRP(dist_m, Tx_dBm, freq_GHz)
    pl   = path_loss(dist_m, freq_GHz);
    rsrp = Tx_dBm - pl + randn()*2;  % Add 2dB shadowing noise
end

% Handover failure probability (based on speed & RSRP)
function p_fail = ho_failure_prob(speed_kmh, rsrp_dBm)
    % Higher speed & weaker signal = more failures
    speed_factor = (speed_kmh / 200);          % normalized 0-1
    signal_factor= max(0, (-rsrp_dBm - 90)/50);% weak signal penalty
    p_fail = min(0.05 + 0.3*speed_factor + 0.2*signal_factor, 0.6);
end

% Handover latency (ms) - includes preparation + execution + speed penalty
function lat = ho_latency_ms(speed_kmh, success, HO_prep, HO_exec)
    base_lat = (HO_prep + HO_exec) * 1000;     % convert to ms
    speed_penalty = speed_kmh * 0.05;           % 0.05ms per km/h
    if ~success
        lat = base_lat * 3 + speed_penalty;     % failure = 3x latency
    else
        lat = base_lat + speed_penalty;
    end
end

%% --- 3. MAIN SIMULATION LOOP ---
fprintf('Running high-speed handover simulation...\n\n');

% Storage for results
results = struct();

for si = 1:nSpeeds
    speed_kmh  = speeds_kmh(si);
    speed_ms   = speed_kmh / 3.6;  % convert to m/s

    % Vehicle position over time
    vehicle_pos = speed_ms * t;

    % Track serving BS
    serving_BS  = 1;
    TTT_counter = 0;
    ttt_active  = false;
    candidate_BS= 0;

    % Metrics
    ho_times      = [];   % Times when HO occurred
    ho_latencies  = [];   % Latency per HO event (ms)
    ho_success    = [];   % 1=success, 0=failure
    rsrp_serving  = zeros(1, nSteps);
    rsrp_neighbor = zeros(1, nSteps);
    active_BS_log = zeros(1, nSteps);

    for k = 1:nSteps
        pos = vehicle_pos(k);

        % Compute RSRP from serving BS
        dist_srv = abs(pos - BS_positions(serving_BS));
        rsrp_srv = compute_RSRP(dist_srv, Tx_power_dBm, freq_GHz);

        % Find best neighbor BS
        rsrp_neighbors = zeros(1, nBS);
        for b = 1:nBS
            d = abs(pos - BS_positions(b));
            rsrp_neighbors(b) = compute_RSRP(d, Tx_power_dBm, freq_GHz);
        end
        rsrp_neighbors(serving_BS) = -Inf;  % exclude serving
        [rsrp_best, best_BS] = max(rsrp_neighbors);

        % Store for plotting
        rsrp_serving(k)  = rsrp_srv;
        rsrp_neighbor(k) = rsrp_best;
        active_BS_log(k) = serving_BS;

        % --- A3 Event Trigger (neighbor > serving + margin) ---
        if rsrp_best > rsrp_srv + HO_margin
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

        % --- Trigger Handover after TTT ---
        if ttt_active && TTT_counter >= TTT
            ttt_active  = false;
            TTT_counter = 0;

            % Determine success/failure
            p_fail  = ho_failure_prob(speed_kmh, rsrp_srv);
            success = rand() > p_fail;

            % Compute latency
            lat = ho_latency_ms(speed_kmh, success, ...
                                HO_preparation_time, HO_execution_time);

            % Log handover event
            ho_times(end+1)    = t(k);
            ho_latencies(end+1)= lat;
            ho_success(end+1)  = success;

            if success
                serving_BS = candidate_BS;
            end
            % On failure, stay with current BS (degraded connection)
        end
    end

    % Store results
    results(si).speed       = speed_kmh;
    results(si).ho_times    = ho_times;
    results(si).latencies   = ho_latencies;
    results(si).success     = ho_success;
    results(si).rsrp        = rsrp_serving;
    results(si).rsrp_nbr    = rsrp_neighbor;
    results(si).bs_log      = active_BS_log;
    results(si).nHO         = length(ho_times);
    results(si).nFail       = sum(ho_success == 0);
    results(si).avg_lat     = mean(ho_latencies);
    results(si).fail_rate   = sum(ho_success==0) / max(length(ho_times),1) * 100;

    fprintf('Speed: %3d km/h | HOs: %2d | Failures: %2d | Fail Rate: %.1f%% | Avg Latency: %.1f ms\n', ...
        speed_kmh, results(si).nHO, results(si).nFail, ...
        results(si).fail_rate, results(si).avg_lat);
end

%% --- 4. PLOTS ---
figure('Name','High-Speed Handover Simulation','NumberTitle','off', ...
       'Position',[50 50 1400 900]);

%-- Plot 1: RSRP trace for a selected speed (160 km/h) --
idx = find([results.speed] == 160);
subplot(3,2,1);
plot(t, results(idx).rsrp,     'b-', 'LineWidth', 1.2); hold on;
plot(t, results(idx).rsrp_nbr, 'r--','LineWidth', 1.0);
% Mark HO events
for h = 1:length(results(idx).ho_times)
    clr = [0 0.6 0]; if ~results(idx).success(h), clr = [0.8 0 0]; end
    xline(results(idx).ho_times(h), '-', 'Color', clr, 'LineWidth', 1.5);
end
xlabel('Time (s)'); ylabel('RSRP (dBm)');
title('RSRP Trace @ 160 km/h (Green=HO Success, Red=Failure)');
legend('Serving BS','Best Neighbor','Location','southwest');
grid on;

%-- Plot 2: Serving BS index over time @ 160 km/h --
subplot(3,2,2);
stairs(t, results(idx).bs_log, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Base Station Index');
title('Serving Base Station over Time @ 160 km/h');
yticks(1:nBS); grid on;

%-- Plot 3: Handover failure rate vs speed --
subplot(3,2,3);
fail_rates = [results.fail_rate];
bar([results.speed], fail_rates, 0.5, 'FaceColor', [0.85 0.33 0.1]);
xlabel('Vehicle Speed (km/h)'); ylabel('Failure Rate (%)');
title('HO Failure Rate vs Vehicle Speed');
grid on; ylim([0 100]);

%-- Plot 4: Average latency vs speed --
subplot(3,2,4);
avg_lats = [results.avg_lat];
bar([results.speed], avg_lats, 0.5, 'FaceColor', [0.2 0.6 0.8]);
xlabel('Vehicle Speed (km/h)'); ylabel('Avg Latency (ms)');
title('Average HO Latency vs Vehicle Speed');
grid on;

%-- Plot 5: Latency distribution @ 160 km/h --
subplot(3,2,5);
if ~isempty(results(idx).latencies)
    histogram(results(idx).latencies, 10, 'FaceColor', [0.4 0.7 0.4]);
    xlabel('Latency (ms)'); ylabel('Count');
    title('HO Latency Distribution @ 160 km/h');
    grid on;
end

%-- Plot 6: Number of HOs vs speed --
subplot(3,2,6);
nHOs = [results.nHO];
bar([results.speed], nHOs, 0.5, 'FaceColor', [0.5 0.3 0.7]);
xlabel('Vehicle Speed (km/h)'); ylabel('Number of Handovers');
title('Total Handovers vs Vehicle Speed');
grid on;

sgtitle('Step 2: 5G V2X Handover Simulation — High-Speed Mobility');

%% --- 5. SUMMARY TABLE ---
fprintf('\n========== SIMULATION SUMMARY ==========\n');
fprintf('%-12s %-8s %-10s %-12s %-14s\n', ...
    'Speed(km/h)','# HOs','# Failures','Fail Rate(%)','Avg Lat(ms)');
fprintf('%s\n', repmat('-', 1, 58));
for si = 1:nSpeeds
    fprintf('%-12d %-8d %-10d %-12.1f %-14.1f\n', ...
        results(si).speed, results(si).nHO, results(si).nFail, ...
        results(si).fail_rate, results(si).avg_lat);
end
fprintf('=========================================\n');
