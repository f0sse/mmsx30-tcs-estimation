clc;

time = xhat_sim.time(:);

z_meas = z_sim.Data';          % [m x N]
z_pred = Z_pred_sim.Data;      % [m x N]
innovation = innovation_sim.Data;

m = size(z_meas,1);

% =========================
% LABELS (AUTO)
% =========================
if m == 5
    labels = ["Acc_x","Acc_y","Vel_x","Vel_y","wz"];
elseif m == 10
    labels = ["Acc_x","Acc_y","Vel_x","Vel_y","wz","omega_rl","omega_rr", "Omega_Tr_RL","Omega_Tr_RR", "FyH"];
else
    labels = "z" + (1:m);
end

% =========================
% MEASUREMENT VS PREDICTION
% =========================
figure(100); clf;

for i = 1:m
    subplot(m,1,i)

    plot(z_sim.Time, z_meas(i,:), 'k', 'LineWidth',1.5); hold on
    plot(time, z_pred(i,:), '--r', 'LineWidth',1.5)

    grid on
    ylabel(labels(i))

    if i == 1
        title('Measurement vs Prediction')
        legend({'Measured','Predicted'}, 'Location','best')
    end

    if i == m
        xlabel('Time [s]')
    end

    %legend({'Measured','Predicted'}, 'Location','best')
end

%% =========================
% INNOVATION
% =========================
figure(101); clf;

for i = 1:m
    subplot(m,1,i)

    plot(time, innovation(i,:), 'b', 'LineWidth',1.5); hold on
    yline(0,'k--')

    grid on
    ylabel(labels(i))

    if i == 1
        title('Innovation (z - z_{pred})')
        legend({'Innovation','Zero line'}, 'Location','best')
    end

    if i == m
        xlabel('Time [s]')
    end

    
end

%% =========================
%% NIS (ROBUST VERSION)
%% =========================
idx_start = 5000;

time_cut = time(idx_start:end);
S_all = S_sim.Data(:,:,idx_start:end);
innovation_cut = innovation(:, idx_start:end);

N = length(time_cut);
NIS = NaN(N,1);
dof = NaN(N,1);

for k = 1:N

    nu = innovation_cut(:,k);
    Sk = S_all(:,:,k);

    % välj giltiga mätningar
    valid = ~isnan(nu);

    nu = nu(valid);
    Sk = Sk(valid,valid);

    if isempty(nu)
        continue
    end

    % symmetri + regularisering
    Sk = 0.5*(Sk + Sk');
    Sk = Sk + 1e-9*eye(size(Sk));

    % condition check (debug)
    if rcond(Sk) < 1e-12
        warning('S near singular at k=%d',k);
    end

    % stabil lösning
    NIS(k) = nu' * (Sk \ nu);
    dof(k) = length(nu);
end

figure(102); clf
plot(time_cut, NIS, 'LineWidth',2)
grid on
title('NIS (innovation consistency)')
xlabel('Time [s]')
ylabel('NIS')
hold on

% robust DOF
valid_dof = dof(~isnan(dof));
if isempty(valid_dof)
    m_eff = 1;
else
    m_eff = round(mean(valid_dof));
end

alpha = 0.01;
lower = chi2inv(alpha/2, m_eff);
upper = chi2inv(1-alpha/2, m_eff);

yline(lower,'r--','Lower bound')
yline(upper,'r--','Upper bound')

legend({'NIS','Confidence bounds'}, 'Location','best')

%% =========================
%% NIS DISTRIBUTION
%% =========================
figure(103); clf

valid_idx = ~isnan(NIS);
NIS_clean = NIS(valid_idx);

if isempty(NIS_clean)
    warning('No valid NIS values to plot')
else
    histogram(NIS_clean,50,'Normalization','pdf')
    hold on

    xmax = max(NIS_clean);

    if isnan(xmax) || xmax <= 0
        xmax = 10;
    end

    x = linspace(0, xmax, 200);
    y = chi2pdf(x, m_eff);

    plot(x,y,'r','LineWidth',2)

    title('NIS distribution vs theoretical \chi^2')
    xlabel('NIS')
    ylabel('PDF')
    legend({'Empirical','Theoretical \chi^2'}, 'Location','best')
    grid on
end

%% =========================
%% KALMAN GAIN
%% =========================
K = K_sim.Data; % [n x m x N]
[n, mK, NK] = size(K);

figure(104); clf

for i = 1:n
    for j = 1:mK

        subplot(n,mK,(i-1)*mK + j)

        plot(time, squeeze(K(i,j,:)), 'LineWidth',1.2)
        grid on

        if i == 1
            title(['Meas: ', labels(j)])
        end

        if j == 1
            ylabel(['State ', num2str(i)])
        end

        if i == n
            xlabel('Time [s]')
        end
    end
end

sgtitle('Kalman Gain Evolution')