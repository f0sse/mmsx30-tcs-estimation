clc;
% close all;

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

xhat_values_full = xhat_sim.Data;     % [5 x N]
dxdt_values_full = dxdt_sim.Data;     % [5 x N]
P_values_full    = P_sim.Data;        % [5 x 5 x N]

%% =========================================================
%% SAMPLING TIME
%% =========================================================
Ts = mean(diff(time));

%% =========================================================
%% REFERENCES
%% =========================================================
vx_ref = Car_vx.Data(:);
vy_ref = Car_vy.Data(:);
wz_ref = Car_wz.Data(:);

vy_ref = vy_ref + (3.473 - 3.6)*wz_ref;

dvx_ref = gradient(vx_ref, Ts);
dvy_ref = gradient(vy_ref, Ts);
dwz_ref = gradient(wz_ref, Ts);

CyF_ref = CF_sim.Data(:);
CyR_ref = CR_sim.Data(:);

%% =========================================================
%% TIRE STIFFNESS CLEANUP
%% =========================================================
CyFL.Data(1:1000) = nan;
CyFR.Data(1:1000) = nan;

CyRL.Data(1:1000) = nan;
CyRR.Data(1:1000) = nan;

CyFL.Data(6000:end) = CyFL.Data(6000);
CyFR.Data(6000:end) = CyFR.Data(6000);

CyRL.Data(6000:end) = CyRL.Data(6000);
CyRR.Data(6000:end) = CyRR.Data(6000);

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

%% =========================================================
%% STATE DEFINITIONS
%% =========================================================
state_names = { ...
    'v_x \; [\mathrm{m/s}]', ...
    'v_y \; [\mathrm{m/s}]', ...
    '\omega_z \; [\mathrm{rad/s}]', ...
    'C_f', ...
    'C_r'};
 
titles = { ...
    'Longitudinal\ Velocity', ...
    'Lateral\ Velocity', ...
    'Yaw\ Rate', ...
    'Front\ Tire\ Cornering\ Stiffness', ...
    'Rear\ Tire\ Cornering\ Stiffness'};

colors = lines(5);

%% =========================================================
%% STATE PLOTS
%% =========================================================
for i = 1:5

    figure(i);
    clf;
    hold on;

    sigma = sqrt(squeeze(P_values(i,i,:)));
    x     = xhat_values(i,:)';

    upper = x + 3*sigma;
    lower = x - 3*sigma;

    %% 3-sigma confidence interval
    fill([time_p; flip(time_p)], ...
         [upper; flip(lower)], ...
         colors(i,:), ...
         'FaceAlpha',0.2, ...
         'EdgeColor','none');

    %% Estimate
    plot(time_p, ...
         x, ...
         'Color',colors(i,:), ...
         'LineWidth',2); 
    xlim([30 time(end)])
    %% =====================================================
    %% REFERENCES
    %% =====================================================
    if i == 1

        plot(time_p,vx_ref_p,'k--','LineWidth',2); 
        %xline(95,'k--');

    elseif i == 2

        plot(time_p, vy_ref_p, 'k--', 'LineWidth',2);

        %xline(95,'k--');

    elseif i == 3

        plot(time_p, wz_ref_p, 'k--','LineWidth',2);

        %xline(95,'k--');

    elseif i == 4

        plot(CyFL.Time, -CyFL.Data, 'k--', 'LineWidth',2);

        plot(CyFR.Time, -CyFR.Data, 'k--', 'LineWidth',2);

        %axis([0,time(end),0,15]);

    elseif i == 5

        plot(CyRL.Time, -CyRL.Data,'k--', 'LineWidth',2);

        plot(CyRR.Time, -CyRR.Data, 'k--', 'LineWidth',2);

        %axis([0,time(end),0,15]);

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
%% NEES
%% =========================================================
idx_start = 300;
idx_end   = length(time) - 100;

idx_vec   = idx_start:idx_end;
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

    xref = [vx_cut(k);
            vy_cut(k);
            wz_cut(k)];

    Pk = P_cut(:,:,k);

    Pk = 0.5*(Pk + Pk');
    Pk = Pk + 1e-9*eye(size(Pk));

    e = xref - xhat;

    nees = e' * (Pk \ e);

    if nees < 100
        NEES(k) = nees;
    end
end

%% =========================================================
%% NEES PLOT
%% =========================================================
alpha = 0.01;

lower = chi2inv(alpha/2,n);
upper = chi2inv(1-alpha/2,n);

figure(6);
clf;

plot(time_nees, ...
     NEES, ...
     'r','LineWidth',2);

hold on;

yline(lower,'k--','Lower bound','LineWidth',2);
yline(upper,'k--','Upper bound','LineWidth',2);

xlabel('$t \; [\mathrm{s}]$');
ylabel('NEES');

title('$\mathbf{NEES\;Consistency\;Test}$');

legend('NEES','Bounds');

grid on;
box on;

%% =========================================================
%% NEES HISTOGRAM
%% =========================================================
figure(7);
clf;

NEES_clean = NEES(~isnan(NEES) & NEES < 100);

edges = 0:0.5:20;

histogram(NEES_clean, ...
          edges, ...
          'Normalization','pdf');

hold on;

x_pdf = linspace(0,20,200);
y_pdf = chi2pdf(x_pdf,n);

plot(x_pdf, ...
     y_pdf, ...
     'r', ...
     'LineWidth',2);

xlabel('NEES');
ylabel('PDF');

title('$\mathbf{NEES\;Distribution}$');

legend('Empirical','Theoretical');

xlim([0 20]);

grid on;
box on;

%% =========================================================
%% DXDT COMPARISON
%% =========================================================
idx_start = 50;

time_cut = time(idx_start:end);

%% =========================================================
%% dvx
%% =========================================================
figure(8);
clf;

plot(time_cut, ...
     dxdt_values_full(1,idx_start:end), ...
     'LineWidth',2);

hold on;

plot(time_cut, ...
     dvx_ref(idx_start:end),'k--', ...
     'LineWidth',3);

xlim([30 time(end)])

xlabel('$Time \; [\mathrm{s}]$');
ylabel('$\dot{v}_x \; [\mathrm{m/s^2}]$');

title('$\mathbf{Longitudinal\;Acceleration}$', ...
      'Interpreter','latex')

legend('Estimate','True');

grid on;
box on;

%% =========================================================
%% dvy
%% =========================================================
figure(9);
clf;

plot(time_cut, ...
     dxdt_values_full(2,idx_start:end), ...
     'LineWidth',2);

hold on;

plot(time_cut, ...
     dvy_ref(idx_start:end),'k--', ...
     'LineWidth',3);

xlim([30 time(end)])
xlabel('$Time \; [\mathrm{s}]$');

ylabel('$\dot{v}_y \; [\mathrm{m/s^2}]$');

title('$\mathbf{Lateral\;Acceleration}$', ...
      'Interpreter','latex')

legend('Estimate','True');

grid on;
box on;

%% =========================================================
%% dwz
%% =========================================================
figure(10);
clf;

plot(time_cut, ...
     dxdt_values_full(3,idx_start:end), ...
     'LineWidth',2);

hold on;

plot(time_cut, ...
     dwz_ref(idx_start:end),'k--', ...
     'LineWidth',3);

xlim([30 time(end)])
xlabel('$Time \; [\mathrm{s}]$');
ylabel('$\dot{\omega}_z \; [\mathrm{rad/s^2}]$');

title('$\mathbf{Yaw\;Acceleration}$', ...
      'Interpreter','latex')
legend('Estimate','True');

grid on;
box on;


%% =========================================================
%% ERROR PLOTS: STATES + ACCELERATIONS
%% =========================================================
%% -------------------------
%% STATES (vx, vy, wz)
%% -------------------------
ref_states = {vx_ref_p, vy_ref_p, wz_ref_p};
error_titles={ 'Longitudinal\ Velocity\ Error', 'Lateral\ Velocity\ Error','Yaw\ Rate\ Error'};
ref_idx = [1 2 3];
state_error={ ...
    'v_x \; [\mathrm{m/s}]', ...
    'v_y \; [\mathrm{m/s}]', ...
    '\omega_z \; [\mathrm{rad/s}]'};
state_names_short = {'v_x','v_y','\omega_z'};

for i = 1:3

    figure(10+i);
    clf;
    hold on;

    xhat = xhat_values(i,:)';
    ref  = ref_states{i};

    N = min(length(ref), length(xhat));

    e = ref(1:N) - xhat(1:N);

    sigma = sqrt(squeeze(P_values(i,i,1:N)));

    plot(time_p(1:N), e, 'r','LineWidth',2);
    plot(time_p(1:N),  3*sigma, 'k--', 'LineWidth',2);
    plot(time_p(1:N), -3*sigma, 'k--', 'LineWidth',2);

    %yline(0,'k-');
    xlim([30 time(end)])
    xlabel('$Time \; [\mathrm{s}]$');
    ylabel(['$', state_error{i}, '$']);

    title(['$\mathbf{',  error_titles{i}, '}$'], ...
        'Interpreter','latex')

    legend({'Error','$ \pm 3\sigma$'});

    grid on; box on;

end
%% =========================================================
%% SAVE FIGURES INDIVIDUALLY
%% =========================================================

% 
% saveFolder = ...
% 'C:\Users\A546783\OneDrive - Volvo Group\MasterThesis_2026\Results\Truck';
% 
% if ~exist(saveFolder,'dir')
%     mkdir(saveFolder);
% end
% 
% figureNames = { ...
%     'vx', ...
%     'vy', ...
%     'wz', ...
%     'Cf', ...
%     'Cr', ...
%     'NEES', ...
%     'NEES_histogram', ...
%     'dvx', ...
%     'dvy', ...
%     'dwz', ...
%     'vx_err', ...
%     'vy_err', ...
%     'wz_err'};
% 
% figureNumbers = [1 2 3 4 5 6 7 8 9 10 11 12 13];
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
%         disp(['Saved: ', filename]);
% 
%     end
% end

% %% -------------------------
% %% ACCELERATIONS
% %% -------------------------
% ref_acc = {dvx_ref, dvy_ref, dwz_ref};
% est_acc = dxdt_values_full;
% 
% acc_names = {'\dot{v}_x','\dot{v}_y','\dot{\omega}_z'};
% 
% for i = 1:3
% 
%     figure(40+i);
%     clf;
%     hold on;
% 
%     est = est_acc(i,idx_start:end)';
%     ref = ref_acc{i}(idx_start:end);
% 
%     N = min(length(ref), length(est));
% 
%     e = ref(1:N) - est(1:N);
% 
%     %% NO covariance available -> only error plot
%     plot(time_cut(1:N), e, 'LineWidth',2);
% 
%     yline(0,'k-');
% 
%     xlabel('$t \; [\mathrm{s}]$');
% 
%     ylabel(['$', acc_names{i}, '\;error$']);
% 
%     title(['$\mathbf{Acceleration\;Error:\;', acc_names{i}, '}$']);
% 
%     grid on;
%     box on;
% 
% end


% % Define the load range
% load_N = linspace(1e4, 5e4, 200);
% 
% % Calculate curve fit based on scaled image coordinates
% x_scaled = load_N / 1e4;
% a = -0.1167;
% b = 1.0417;
% c = 0.025;
% 
% stiffness_scaled = a * x_scaled.^2 + b * x_scaled + c;
% stiffness = stiffness_scaled * 1e5;
% 
% % Generate figure
% figure('Color', 'w');
% plot(load_N, stiffness, 'b-', 'LineWidth', 1);
% 
% % Configure axes limits and ticks
% xlim([1e4, 5e4]);
% ylim([0.8e5, 2.4e5]);
% xticks(1e4:1e4:5e4);
% yticks(0.8e5:0.2e5:2.4e5);
% 
% % Configure labels and aesthetics
% xlabel('Load (N)');
% ylabel('Cornering stiffness (N/load)');
% set(gca, 'Box', 'on', 'TickDir', 'in');
 