%% =========================================================
%  Project 7: Optimization of Handover Mechanisms in 5G V2X
%  Step 1: MDP-based Handover using Q-Learning
%  =========================================================

clc; clear; close all;

%% --- 1. STATE SPACE DEFINITION ---
% State = [RSRP_level, Speed_level, PingPong_flag]
% RSRP levels:  1=Poor(<-110dBm), 2=Medium(-110 to -90dBm), 3=Good(>-90dBm)
% Speed levels: 1=Low(<50km/h),   2=Medium(50-100km/h),     3=High(>100km/h)
% PingPong:     0=No recent HO,   1=Recent HO (risk of ping-pong)

nRSRP     = 3;
nSpeed    = 3;
nPingPong = 2;
nStates   = nRSRP * nSpeed * nPingPong;  % 18 states total

% Action space
% 1 = Stay with current base station
% 2 = Handover to neighboring base station
nActions = 2;
ACTION_STAY = 1;
ACTION_HO   = 2;

%% --- 2. Q-TABLE INITIALIZATION ---
Q = zeros(nStates, nActions);  % Q(state, action)

%% --- 3. HYPERPARAMETERS ---
alpha      = 0.1;    % Learning rate
gamma      = 0.9;    % Discount factor
epsilon    = 1.0;    % Initial exploration rate (epsilon-greedy)
eps_min    = 0.01;   % Minimum epsilon
eps_decay  = 0.995;  % Epsilon decay per episode
nEpisodes  = 5000;   % Training episodes
nSteps     = 50;     % Steps per episode (time slots)

%% --- 4. HELPER FUNCTIONS ---

% Encode state (RSRP, Speed, PingPong) -> single state index
function s = encode_state(rsrp, speed, pp)
    s = (rsrp-1)*6 + (speed-1)*2 + (pp+1);  % index 1..18
end

% Reward function
function r = get_reward(action, rsrp, pp, success)
    if action == 2  % Handover attempted
        if ~success
            r = -10;   % Handover failure
        elseif pp == 1
            r = -5;    % Ping-pong handover
        else
            r = +10;   % Successful, necessary handover
        end
    else  % Stay
        if rsrp == 1
            r = -3;    % Staying with poor signal
        elseif rsrp == 3
            r = +2;    % Staying with good signal
        else
            r = 0;     % Neutral
        end
    end
end

% Simulate environment transition
function [next_rsrp, next_speed, next_pp, success] = ...
         step_env(action, rsrp, speed, pp)
    success = true;

    % Speed changes randomly (vehicle accelerates/decelerates slightly)
    speed_delta = randi([-1, 1]);
    next_speed  = min(max(speed + speed_delta, 1), 3);

    if action == 2  % Handover
        % HO failure probability increases at high speed & poor signal
        fail_prob = 0.05 * speed + 0.1 * (rsrp == 1);
        if rand() < fail_prob
            success   = false;
            next_rsrp = max(rsrp - 1, 1);  % Signal degrades on failure
            next_pp   = 0;
        else
            % Successful HO improves signal
            next_rsrp = min(rsrp + 1, 3);
            next_pp   = 1;  % Flag ping-pong risk
        end
    else  % Stay
        % Signal drifts randomly
        rsrp_delta = randi([-1, 1]);
        next_rsrp  = min(max(rsrp + rsrp_delta, 1), 3);
        next_pp    = 0;
    end
end

%% --- 5. Q-LEARNING TRAINING LOOP ---
fprintf('Training Q-Learning agent...\n');

total_rewards = zeros(1, nEpisodes);
ho_counts     = zeros(1, nEpisodes);
failure_counts= zeros(1, nEpisodes);

for ep = 1:nEpisodes
    % Random initial state
    rsrp  = randi(3);
    speed = randi(3);
    pp    = randi([0,1]);
    s     = encode_state(rsrp, speed, pp);

    ep_reward   = 0;
    ep_ho       = 0;
    ep_failures = 0;

    for t = 1:nSteps
        % Epsilon-greedy action selection
        if rand() < epsilon
            a = randi(nActions);          % Explore
        else
            [~, a] = max(Q(s, :));        % Exploit
        end

        % Environment step
        [next_rsrp, next_speed, next_pp, success] = ...
            step_env(a, rsrp, speed, pp);

        % Reward
        r = get_reward(a, rsrp, pp, success);

        % Next state
        s_next = encode_state(next_rsrp, next_speed, next_pp);

        % Q-Table update (Bellman equation)
        Q(s, a) = Q(s, a) + alpha * (r + gamma * max(Q(s_next, :)) - Q(s, a));

        % Track metrics
        ep_reward = ep_reward + r;
        if a == ACTION_HO
            ep_ho = ep_ho + 1;
            if ~success, ep_failures = ep_failures + 1; end
        end

        % Transition
        rsrp  = next_rsrp;
        speed = next_speed;
        pp    = next_pp;
        s     = s_next;
    end

    total_rewards(ep)  = ep_reward;
    ho_counts(ep)      = ep_ho;
    failure_counts(ep) = ep_failures;

    % Decay epsilon
    epsilon = max(epsilon * eps_decay, eps_min);

    if mod(ep, 500) == 0
        fprintf('Episode %4d | Avg Reward: %.2f | Epsilon: %.3f\n', ...
            ep, mean(total_rewards(max(1,ep-499):ep)), epsilon);
    end
end

fprintf('Training complete.\n\n');

%% --- 6. DISPLAY LEARNED POLICY ---
fprintf('--- Learned Optimal Policy ---\n');
fprintf('%-6s %-8s %-10s | %-20s\n', 'RSRP', 'Speed', 'PingPong', 'Action');
fprintf('%s\n', repmat('-', 1, 48));

rsrp_labels  = {'Poor', 'Medium', 'Good'};
speed_labels = {'Low', 'Medium', 'High'};
action_labels= {'STAY', 'HANDOVER'};

for r = 1:3
    for sp = 1:3
        for pp = 0:1
            s   = encode_state(r, sp, pp);
            [~, best_a] = max(Q(s, :));
            fprintf('%-6s %-8s %-10d | %s\n', ...
                rsrp_labels{r}, speed_labels{sp}, pp, action_labels{best_a});
        end
    end
end

%% --- 7. PLOTS ---

% Smooth rewards for plotting
window = 100;
smooth_rewards = movmean(total_rewards, window);

figure('Name', 'Q-Learning Training Results', 'NumberTitle', 'off', ...
       'Position', [100 100 1200 800]);

% Plot 1: Reward over episodes
subplot(2,2,1);
plot(1:nEpisodes, smooth_rewards, 'b-', 'LineWidth', 1.5);
xlabel('Episode'); ylabel('Avg Reward (smoothed)');
title('Cumulative Reward per Episode');
grid on;

% Plot 2: Handover count per episode
subplot(2,2,2);
plot(1:nEpisodes, movmean(ho_counts, window), 'g-', 'LineWidth', 1.5);
xlabel('Episode'); ylabel('Handover Count');
title('Handovers per Episode');
grid on;

% Plot 3: Failure count per episode
subplot(2,2,3);
plot(1:nEpisodes, movmean(failure_counts, window), 'r-', 'LineWidth', 1.5);
xlabel('Episode'); ylabel('Failure Count');
title('Handover Failures per Episode');
grid on;

% Plot 4: Q-Table heatmap (Action 1 = Stay vs Action 2 = HO)
subplot(2,2,4);
q_diff = Q(:,2) - Q(:,1);  % Positive = prefer HO, Negative = prefer Stay
bar(q_diff, 'FaceColor', [0.2 0.6 0.8]);
xlabel('State Index'); ylabel('Q(HO) - Q(Stay)');
title('Policy Preference: HO vs Stay per State');
yline(0, 'r--', 'LineWidth', 1.5);
grid on;

sgtitle('MDP Q-Learning: 5G V2X Handover Optimization');

%% --- 8. FINAL STATISTICS ---
fprintf('\n--- Final Training Statistics ---\n');
fprintf('Avg Reward (last 500 eps):   %.2f\n', mean(total_rewards(end-499:end)));
fprintf('Avg HO Count (last 500 eps): %.2f\n', mean(ho_counts(end-499:end)));
fprintf('Avg Failures (last 500 eps): %.2f\n', mean(failure_counts(end-499:end)));
fprintf('Final Epsilon:               %.4f\n', epsilon);
