clc;
%close all;

%% =========================================================
%% GLOBAL PLOT SETTINGS
%% =========================================================
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

set(groot,'defaultAxesFontSize',18);
set(groot,'defaultTextFontSize',22);
set(groot,'defaultLegendFontSize',22);
%% =========================================================
%% LOAD DATA
%% =========================================================
time = xhat_sim.time(:);

xhat_values_full = xhat_sim.Data;     % [8 x N]
dxdt_values_full = dxdt_sim.Data;     % [8 x N]
P_values_full    = P_sim.Data;        % [8 x 8 x N]

%% =========================================================
%% TIME STEP
%% =========================================================
Ts = mean(diff(time));

%% =========================================================
%% REFERENCES
%% =========================================================
vx_ref = Car_vx.Data(:);
vy_ref = Car_vy.Data(:);
wz_ref = Car_wz.Data(:);
psi_ref=psi.Data(:);
wz_tr_ref=Tr_YawRate.Data(:);
vy_ref = vy_ref + (3.473 - 3.6)*wz_ref;

dvx_ref = gradient(vx_ref, Ts);
dvy_ref = gradient(vy_ref, Ts);
dwz_ref = gradient(wz_ref, Ts);
dwz_tr_ref = gradient(Tr_YawRate.Data(:), Ts);

CyFL_ref = -remove_ts_spikes(CyFL.Data(:));
CyFR_ref = -remove_ts_spikes(CyFR.Data(:));
CyRL_ref = -remove_ts_spikes(CyRL.Data(:));
CyRR_ref = -remove_ts_spikes(CyRR.Data(:));

CyML_ref = -remove_ts_spikes(CyML.Data(:));
CyMR_ref = -remove_ts_spikes(CyMR.Data(:));

% CyFL.Data(1:1000) = nan;
% CyFR.Data(1:1000) = nan;
% CyFL.Data(6000:end) = CyFL.Data(6000);
% CyFR.Data(6000:end) = CyFR.Data(6000);
% 
% CyRL.Data(1:1000) = nan;
% CyRR.Data(1:1000) = nan;
% CyRL.Data(6000:end) = CyRL.Data(6000);
% CyRR.Data(6000:end) = CyRR.Data(6000);
% 
% CyML.Data(1:1000) = nan;
% CyMR.Data(1:1000) = nan;
% CyML.Data(6000:end) = CyML.Data(6000);
% CyMR.Data(6000:end) = CyMR.Data(6000);

%% =========================================================
%% DOWNSAMPLING
%% =========================================================
ds = 1;
start=3100;
time_p = time(start:ds:end);

xhat_values = xhat_values_full(:,start:ds:end);
dxdt_values = dxdt_values_full(:,start:ds:end);
P_values    = P_values_full(:,:,start:ds:end);

vx_ref_p = vx_ref(start:ds:end);
vy_ref_p = vy_ref(start:ds:end);
wz_ref_p = wz_ref(start:ds:end);
psi_ref_p=psi_ref(start:ds:end);
wz_tr_ref_p=wz_tr_ref(start:ds:end);

CyFL_ref = CyFL_ref(start:ds:end);
CyFR_ref = CyFR_ref(start:ds:end);
CyRL_ref = CyRL_ref(start:ds:end);
CyRR_ref = CyRR_ref(start:ds:end);
CyML_ref = CyML_ref(start:ds:end);
CyMR_ref = CyMR_ref(start:ds:end);

%% =========================================================
%% STATE DEFINITIONS
%% =========================================================
state_names = { ...
    'v_x \; [\mathrm{m/s}]', ...
    'v_y \; [\mathrm{m/s}]', ...
    '\omega_z \; [\mathrm{rad/s}]', ...
    'C_f', ...
    'C_r', ...
    '\psi \; [\mathrm{rad}]', ...
    '\omega_{z,tr} \; [\mathrm{rad/s}]', ...
    'C_{mr}'};
titles = { 'Longitudinal\ Velocity', 'Lateral\ Velocity','Yaw\ Rate', 'Front\ Tire\ Cornering\ Stiffness', 'Rear\ Tire\ Cornering\ Stiffness','Articulation\ Angle','Trailer\ Yaw\ Rate', 'Trailer\ Middel\ Rear\ Tire\ Cornering\ Stiffness'  };

colors = lines(8);


%% =========================================================
%% STATE PLOTS (3σ)
%% =========================================================
for i = 1:8

    figure(i); clf; hold on;

    sigma = sqrt(squeeze(P_values(i,i,:)));
    x = xhat_values(i,:)';

    upper = x + 3*sigma;
    lower = x - 3*sigma;

    fill([time_p; flip(time_p)], ...
         [upper; flip(lower)], ...
         colors(i,:), ...
         'FaceAlpha',0.2, ...
         'EdgeColor','none');

    plot(time_p, x, 'LineWidth',2, 'Color',colors(i,:));
    xlim([30 time(end)])
    %axis([30 time(end) -100 100]);

    %% =========================
    %% REFERENCES
    %% =========================
    switch i
        case 1
            plot(time_p, vx_ref_p,'k--','LineWidth',1.8);
            xline(95,'k--')
        case 2
            plot(time_p, vy_ref_p,'k--','LineWidth',1.8);
            xline(95,'k--')
        case 3
            plot(time_p, wz_ref_p,'k--','LineWidth',1.8);
            xline(95,'k--')
        case 4
            plot(time_p,CyFL_ref,'k--','LineWidth',1.8);
            plot(time_p,CyFR_ref,'k--','LineWidth',1.8);
            axis([30 time(end) 0 15]);
        case 5
            plot(time_p,CyRL_ref,'k--','LineWidth',1.8);
            plot(time_p,CyRR_ref,'k--','LineWidth',1.8);
            axis([30 time(end) 0 15]);
        case 6
            plot(time_p, psi_ref_p,'k--','LineWidth',1.8);
        case 7
            plot(time_p, wz_tr_ref_p,'k--','LineWidth',1.8);
        case 8
            plot(time_p,CyML_ref,'k--','LineWidth',1.8);
            plot(time_p,CyMR_ref,'k--','LineWidth',1.8);
            %axis([30 time(end) 0 15]);
    end

    %% =====================================================
    %% FIGURE FORMATTING
    %% =====================================================
    xlabel('$Time \; [\mathrm{s}]$');

    ylabel(['$', state_names{i}, '$'], 'Interpreter','latex')

    title(['$\mathbf{',  titles{i}, '\;with\;3\sigma\;CI}$'], ...
        'Interpreter','latex')

    legend({ '$3\sigma$ CI', 'Estimate', 'True'} );

    grid on;
    box on;

end

%% =========================================================
%% DXDT COMPARISON (FULLY CORRECT)
%% =========================================================
idx_start = 100;
idx_end   = length(time)-100;

time_cut = time(idx_start:idx_end);

%% dvx
figure(9); clf; hold on;

plot(time_cut, dxdt_values_full(1,idx_start:idx_end),'LineWidth',2);
plot(time_cut, dvx_ref(idx_start:idx_end),'k--','LineWidth',2);
xlim([30 time(end)])
title(['$\mathbf{', 'Longitudinal\ Acceleration', '}$'], ...
        'Interpreter','latex')
xlabel('$ Time\; [s]$');
ylabel('$\dot{v}_x\;[m/s^2]$');

legend({'Estimate','True'});
grid on; box on;

%% dvy
figure(10); clf; hold on;

plot(time_cut, dxdt_values_full(2,idx_start:idx_end),'LineWidth',2);
plot(time_cut, dvy_ref(idx_start:idx_end),'k--','LineWidth',2);
xlim([30 time(end)])
title(['$\mathbf{', 'Lateral\ Acceleration', '}$'], ...
        'Interpreter','latex')
xlabel('$Time\; [s]$');
ylabel('$\dot{v}_y\;[m/s^2]$');

legend({'Estimate','True'});
grid on; box on;

%% dwz
figure(11); clf; hold on;

plot(time_cut, dxdt_values_full(3,idx_start:idx_end),'LineWidth',2);
plot(time_cut, dwz_ref(idx_start:idx_end),'k--','LineWidth',2);
xlim([30 time(end)])
title(['$\mathbf{', 'Yaw\ Acceleration', '}$'], ...
        'Interpreter','latex')
xlabel('$Time;[s]$');
ylabel('$\dot{\omega}_z\;[rad/s^2]$');

legend({'Estimate','True'});
grid on; box on;

%% trailer yaw rate
figure(12); clf; hold on;

plot(time_cut, dxdt_values_full(6,idx_start:idx_end),'LineWidth',2);
plot(time_cut, dwz_tr_ref(idx_start:idx_end),'k--','LineWidth',2);
xlim([30 time(end)])
title(['$\mathbf{', 'Trailer\ Yaw\ Acceleration', '}$'], ...
        'Interpreter','latex')

xlabel('$Time\;[s]$');
ylabel('$\dot{\omega}_{z,tr}\;[rad/s^2]$');

legend({'Estimate','True'});
grid on; box on;
%% =========================================================
%% STATE ERROR PLOTS
%% =========================================================
ref_states = {vx_ref_p, vy_ref_p, wz_ref_p,psi_ref_p,wz_tr_ref_p};
error_titles={ 'Longitudinal\ Velocity\ Error', 'Lateral\ Velocity\ Error','Yaw\ Rate\ Error', 'Articulation\ Angle\ Error', 'Trailer\ Yaw\ Rate\ Error' };
ref_idx = [1 2 3 4 5];
state_error={ ...
    'v_x \; [\mathrm{m/s}]', ...
    'v_y \; [\mathrm{m/s}]', ...
    '\omega_z \; [\mathrm{rad/s}]', ...
    '\psi \; [\mathrm{rad}]', ...
    '\omega_{z,tr} \; [\mathrm{rad/s}]'};
for k = 1:5

    i = ref_idx(k);

    figure(12+k); clf; hold on;
    if i==4
        i=6;
    elseif i==5
        i=7;
    end
    xhat = xhat_values(i,:)';
    ref  = ref_states{k};

    N = min(length(xhat), length(ref));

    e = ref(1:N) - xhat(1:N);

    sigma = sqrt(squeeze(P_values(i,i,1:N)));

    plot(time_p(1:N), e,'r','LineWidth',2);
    plot(time_p(1:N),  3*sigma,'k--','LineWidth',2);
    plot(time_p(1:N), -3*sigma,'k--','LineWidth',2);
    yline(0,'k-');
    xlim([30 time(end)])
   if i==6
        i=4;
    elseif i==7
        i=5;
    end
     xlabel('$Time\;[s]$');
     ylabel(['$', state_error{i}, '$'], 'Interpreter','latex')
     title(['$\mathbf{',  error_titles{i}, '}$'], ...
        'Interpreter','latex')
     legend({ 'Error','$ \pm3\sigma$ '} );
    grid on; box on;

end

%% =========================
%% NEES (ROBUST VERSION)
%% =========================
idx_start = 300;
idx_end   = length(time) - 100;

idx_vec = idx_start:idx_end;
time_nees = time(idx_vec);

xhat_cut = xhat_sim.Data(1:3, idx_vec);
P_cut    = P_sim.Data(1:3,1:3,idx_vec);

vx_cut = vx_ref(idx_vec);
vy_cut = vy_ref(idx_vec);
wz_cut = wz_ref(idx_vec);

N = length(time_nees);
n = 3;

NEES = NaN(N,1);

for k = 1:N

    xhat = xhat_cut(:,k);
    xref = [vx_cut(k); vy_cut(k); wz_cut(k)];

    Pk = P_cut(:,:,k);

    Pk = 0.5*(Pk + Pk');
    Pk = Pk + 1e-9*eye(size(Pk));

    e = xref - xhat;

    nees = e' * (Pk \ e);

    if nees < 100
        NEES(k) = nees;
    end
end

alpha = 0.01;
lower = chi2inv(alpha/2,n);
upper = chi2inv(1-alpha/2,n);

figure(18); clf
plot(time_nees,NEES,'r','LineWidth',2)
hold on
yline(lower,'k--','Lower bound')
yline(upper,'k--','Upper bound')

xlabel('Time [s]')
ylabel('NEES')
title('Tractor NEES consistency test')
grid on

%%=========================
%% NEES HISTOGRAM
%% =========================
figure(19); clf

NEES_clean = NEES(~isnan(NEES) & NEES < 100);

if isempty(NEES_clean)
    warning('No valid NEES values')
else

    edges = 0:0.5:20;
    histogram(NEES_clean, edges, 'Normalization','pdf')
    hold on

    x_pdf = linspace(0,20,200);
    y_pdf = chi2pdf(x_pdf,n);

    plot(x_pdf,y_pdf,'r','LineWidth',2)

    xlim([0 20])

    xlabel('NEES')
    ylabel('PDF')
    title('Tractor NEES distribution')
    legend('Empirical','Theoretical')
    grid on
end

%% NEES (ROBUST VERSION)
%% =========================
idx_start = 300;
idx_end   = length(time) - 100;

idx_vec = idx_start:idx_end;
time_nees = time(idx_vec);

xhat_cut = xhat_sim.Data(6:7, idx_vec);
P_cut    = P_sim.Data(6:7,6:7,idx_vec);

psi_cut = psi_ref(idx_vec);
wr_tr_cut = wz_tr_ref(idx_vec);
%wz_cut = wz_ref(idx_vec);

N = length(time_nees);
n = 2;

NEES = NaN(N,1);

for k = 1:N

    xhat = xhat_cut(:,k);
    xref = [psi_cut(k); wz_tr_ref(k)];

    Pk = P_cut(:,:,k);

    Pk = 0.5*(Pk + Pk');
    Pk = Pk + 1e-9*eye(size(Pk));

    e = xref - xhat;

    nees = e' * (Pk \ e);

    if nees < 100
        NEES(k) = nees;
    end
end

alpha = 0.01;
lower = chi2inv(alpha/2,n);
upper = chi2inv(1-alpha/2,n);

figure(20); clf
plot(time_nees,NEES,'r','LineWidth',2)
hold on
yline(lower,'k--','Lower bound')
yline(upper,'k--','Upper bound')

xlabel('Time [s]')
ylabel('NEES')
title('Trailer NEES consistency test')
grid on

%%=========================
%% NEES HISTOGRAM
%% =========================
figure(21); clf

NEES_clean = NEES(~isnan(NEES) & NEES < 100);

if isempty(NEES_clean)
    warning('No valid NEES values')
else

    edges = 0:0.5:20;
    histogram(NEES_clean, edges, 'Normalization','pdf')
    hold on

    x_pdf = linspace(0,20,200);
    y_pdf = chi2pdf(x_pdf,n);

    plot(x_pdf,y_pdf,'r','LineWidth',2)

    xlim([0 20])

    xlabel('NEES')
    ylabel('PDF')
    title('Trailer NEES distribution')
    legend('Empirical','Theoretical')
    grid on
end


%% =========================================================
%% SAVE FIGURES
%% =========================================================
% saveFolder = ...
% 'C:\Users\A546783\OneDrive - Volvo Group\MasterThesis_2026\Results\TruckAndTrailer';
% 
% if ~exist(saveFolder,'dir')
%     mkdir(saveFolder);
% end
% 
% figureNames = { ...
%     'vx_tr', ...
%     'vy_tr', ...
%     'wz_tr', ...
%     'Cf_tr', ...
%     'Cr_tr', ...
%     'psi_tr', ...
%     'wztr_tr', ...
%     'Crtr_tr', ...
%     'dvx_tr', ...
%     'dvy_tr', ...
%     'dwz_tr', ...
%     'dwztr_tr', ...
%     'vx_tr_err', ...
%     'vy_tr_err', ...
%     'wz_tr_err', ...
%     'psi_tr_err', ...
%     'wztr_tr_err'};
% 
% figureNumbers = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17];
% 
% for k = 1:length(figureNumbers)
% 
%     figNum = figureNumbers(k);
% 
%     if isgraphics(figNum)
% 
%         figure(figNum);
% 
%         set(gcf,'Color','w');
% 
%         filename = fullfile( ...
%             saveFolder, ...
%             [figureNames{k}, '_WS.pdf']);
% 
%         exportgraphics(gcf, ...
%                        filename, ...
%                        'ContentType','vector');
% 
%         %disp(['Saved: ', filename]);
% 
%     end
% end


%% Process and Measurement  Noise values  when the trailer tires are lumped at middel axsis
% Q=  diag([2e-2 0.02 3e-3 0 0  0.02 1e-2 0.000 ]);
% R= diag([2; 2; 0.2; 0.2; 0.5; 0.5; 0.5; 0.5; 0.5; 2e4*3 ])/3;


%% GLOBAL PLOT SETTINGS
% %% =========================================================
% set(groot,'defaultTextInterpreter','latex');
% set(groot,'defaultAxesTickLabelInterpreter','latex');
% set(groot,'defaultLegendInterpreter','latex');
% 
% set(groot,'defaultAxesFontSize',18);
% set(groot,'defaultTextFontSize',22);
% set(groot,'defaultLegendFontSize',22);
% 
% %% NEES for TRACTOR and TRAILER (Separate Units)
% %% =========================
% idx_start = 300;
% idx_end   = length(time) - 100;
% idx_vec = idx_start:idx_end;
% time_nees = time(idx_vec);
% 
% %% =========================================================
% %% TRACTOR NEES (states: vx, vy, wz) - n = 3
% %% =========================================================
% % Extract tractor states
% xhat_tractor = xhat_sim.Data(1:3, idx_vec);    % vx, vy, wz
% 
% % Extract tractor covariances (3x3xN)
% P_tractor = P_sim.Data(1:3, 1:3, idx_vec);
% 
% % Reference states for tractor
% vx_ref_cut   = vx_ref(idx_vec);
% vy_ref_cut   = vy_ref(idx_vec);
% wz_ref_cut   = wz_ref(idx_vec);
% 
% N = length(time_nees);
% n_tractor = 3;
% 
% NEES_tractor = NaN(N,1);
% 
% for k = 1:N
%     xhat = xhat_tractor(:, k);
%     xref = [vx_ref_cut(k); vy_ref_cut(k); wz_ref_cut(k)];
%     Pk = P_tractor(:,:,k);
% 
%     Pk = 0.5*(Pk + Pk');
%     Pk = Pk + 1e-9*eye(n_tractor);
% 
%     e = xref - xhat;
%     nees = e' * (Pk \ e);
% 
%     if nees < 100
%         NEES_tractor(k) = nees;
%     end
% end
% 
% %% =========================================================
% %% TRAILER NEES (states: psi, wz_tr) - n = 2
% %% =========================================================
% % Extract trailer states
% xhat_trailer = xhat_sim.Data(6:7, idx_vec);    % psi, wz_tr
% 
% % Extract trailer covariances (2x2xN)
% P_trailer = P_sim.Data(6:7, 6:7, idx_vec);
% 
% % Reference states for trailer
% psi_ref_cut    = psi_ref(idx_vec);
% wz_tr_ref_cut  = wz_tr_ref(idx_vec);
% 
% n_trailer = 2;
% 
% NEES_trailer = NaN(N,1);
% 
% for k = 1:N
%     xhat = xhat_trailer(:, k);
%     xref = [psi_ref_cut(k); wz_tr_ref_cut(k)];
%     Pk = P_trailer(:,:,k);
% 
%     Pk = 0.5*(Pk + Pk');
%     Pk = Pk + 1e-9*eye(n_trailer);
% 
%     e = xref - xhat;
%     nees = e' * (Pk \ e);
% 
%     if nees < 100
%         NEES_trailer(k) = nees;
%     end
% end
% 
% %% =========================================================
% %% PLOT TRACTOR NEES
% %% =========================================================
% alpha = 0.01;
% lower_tractor = chi2inv(alpha/2, n_tractor);
% upper_tractor = chi2inv(1-alpha/2, n_tractor);
% 
% figure(18); clf
% plot(time_nees, NEES_tractor,'r' ,'LineWidth', 1.5)
% hold on
% yline(lower_tractor, 'k--', 'Lower bound', 'LineWidth', 1.5)
% yline(upper_tractor, 'k--', 'Upper bound', 'LineWidth', 1.5)
% xlabel('Time [s]')
% ylabel('NEES')
% title(sprintf('Tractor NEES Consistency Test (n=%d)', n_tractor))
% legend('NEES', 'Bounds', 'Location', 'best')
% grid on
% ylim([0 20])
% 
% %% =========================================================
% %% PLOT TRAILER NEES
% %% =========================================================
% lower_trailer = chi2inv(alpha/2, n_trailer);
% upper_trailer = chi2inv(1-alpha/2, n_trailer);
% 
% figure(19); clf
% plot(time_nees, NEES_trailer,'r', 'LineWidth', 1.5)
% hold on
% yline(lower_trailer, 'k--', 'Lower bound', 'LineWidth', 1.5)
% yline(upper_trailer, 'k--', 'Upper bound', 'LineWidth', 1.5)
% xlabel('Time [s]')
% ylabel('NEES')
% title(sprintf('Trailer NEES Consistency Test (n=%d)', n_trailer))
% legend('NEES', 'Bounds', 'Location', 'best')
% grid on
% ylim([0 20])
% 
% %% =========================================================
% %% HISTOGRAM TRACTOR
% %% =========================================================
% figure(20); clf
% NEES_clean_tractor = NEES_tractor(~isnan(NEES_tractor) & NEES_tractor < 100);
% 
% if ~isempty(NEES_clean_tractor)
%     edges = 0:0.5:15;
%     histogram(NEES_clean_tractor, edges, 'Normalization', 'pdf', 'FaceAlpha', 0.5)
%     hold on
% 
%     x_pdf = linspace(0, 15, 200);
%     y_pdf = chi2pdf(x_pdf, n_tractor);
%     plot(x_pdf, y_pdf, 'r-', 'LineWidth', 2)
% 
%     xlim([0 15])
%     xlabel('NEES')
%     ylabel('PDF')
%     title(sprintf('Tractor NEES Distribution vs χ²(%d)', n_tractor))
%     legend('Empirical', sprintf('Theoretical χ²(%d)', n_tractor), 'Location', 'best')
%     grid on
% end
% 
% %% =========================================================
% %% HISTOGRAM TRAILER
% %% =========================================================
% figure(21); clf
% NEES_clean_trailer = NEES_trailer(~isnan(NEES_trailer) & NEES_trailer < 100);
% 
% if ~isempty(NEES_clean_trailer)
%     edges = 0:0.5:15;
%     histogram(NEES_clean_trailer, edges, 'Normalization', 'pdf', 'FaceAlpha', 0.5)
%     hold on
% 
%     x_pdf = linspace(0, 15, 200);
%     y_pdf = chi2pdf(x_pdf, n_trailer);
%     plot(x_pdf, y_pdf, 'r-', 'LineWidth', 2)
% 
%     xlim([0 15])
%     xlabel('NEES')
%     ylabel('PDF')
%     title(sprintf('Trailer NEES Distribution vs χ²(%d)', n_trailer))
%     legend('Empirical', sprintf('Theoretical χ²(%d)', n_trailer), 'Location', 'best')
%     grid on
% end
% 
% 
% %% Save NEES Figures
% % Folder where figures will be saved
% saveFolder = 'C:\Users\A546783\OneDrive - Volvo Group\MasterThesis_2026\Results\TruckAndTrailer';
% 
% % Create folder if it does not exist
% if ~exist(saveFolder, 'dir')
%     mkdir(saveFolder);
% end
% 
% % Names for NEES figures
% figureNames = { ...
%     'Tractor_NEES_TimeSeries', ...
%     'Trailer_NEES_TimeSeries', ...
%     'Tractor_NEES_Histogram', ...
%     'Trailer_NEES_Histogram'};
% 
% % Corresponding figure numbers
% figureNumbers = [18, 19, 20, 21];
% 
% for k = 1:length(figureNumbers)
%     figNum = figureNumbers(k);
% 
%     if isgraphics(figNum)
%         figure(figNum);
% 
%         % White background
%         set(gcf, 'Color', 'w');
% 
%         % Full filename
%         filename = fullfile(saveFolder, [figureNames{k}, '.pdf']);
% 
%         % Save figure
%         exportgraphics(gcf, filename, 'ContentType', 'vector');
% 
%         disp(['Saved: ', filename]);
%     else
%         warning('Figure %d does not exist or is not a graphics object', figNum);
%     end
% end
% 
% disp('All NEES figures saved successfully!');
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% %%
% %% Plot off-diagonal covariances as time series
% % P_sim.Data är 8x8xN
% 
% % Välj tidsintervall
% idx_start = 300;
% idx_end = length(time) - 100;
% idx_vec = idx_start:idx_end;
% time_plot = time(idx_vec);
% 
% % Välj vilka state-par du vill plotta
% % Exempel: kovarians mellan vx och vy (index 1 och 2)
% cov_vx_vy = squeeze(P_sim.Data(1, 2, idx_vec));
% 
% % Kovarians mellan psi och wz_tr (index 6 och 7)
% cov_psi_wztr = squeeze(P_sim.Data(6, 7, idx_vec));
% 
% % Kovarians mellan vx och wz (index 1 och 3)
% cov_vx_wz = squeeze(P_sim.Data(1, 3, idx_vec));
% 
% figure;
% % subplot(3,1,1);
% % plot(time_plot, cov_vx_vy, 'b-', 'LineWidth', 1.5);
% % ylabel('Cov(v_x, v_y)');
% % title('Off-diagonal Covariances');
% % grid on;
% 
% %subplot(3,1,2);
% plot(time_plot, cov_psi_wztr, 'r-', 'LineWidth', 1.5);
% ylabel('Cov(\psi, \omega_{z,tr})');
% grid on;
% 
% % subplot(3,1,3);
% % plot(time_plot, cov_vx_wz, 'g-', 'LineWidth', 1.5);
% % ylabel('Cov(v_x, \omega_z)');
% % xlabel('Time [s]');
% % grid on;

function y = remove_ts_spikes(ts)
    dx = [0; diff(ts)];

    % Calculate scaled Median Absolute Deviation for robust thresholding
    threshold = 0.75 * median(abs(dx - median(dx, 'omitnan')), 'omitnan');

    % Locate indices where the discrete change exceeds the threshold
    spike_indices = abs(dx) > threshold;

    % Nullify the instantaneous spikes
    y = ts;
    y(spike_indices) = nan;
    y = fillmissing(y, 'pchip', EndValues='previous');
end