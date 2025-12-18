%% GdS Hausaufgabe 1: Fahrdynamik-Simulation Gruppe B
% Rekonstruktion Bahnübergangsunfall – Hauptskript
clearvars; close all; clc;

%%
%  SYSTEMPARAMETER (Hausaufgabe)

% Massen
m_lok   = 90;               % [t]
m_wagen = 20 * 90;          % [t]
m_ges   = m_lok + m_wagen;  %[t]

% Massenfaktor
lambda   = 1.04;

% Lokbremse (konstant)
F_lok_bremse = 130;         % [kN]

% Zeitschritte
dt_beschl = 0.05;
dt_bremse = 0.025;

g = 9.81;                   % [m/s^2]

% Strecke
s0_km        = 112.130;     % [km]
s_unfall_km  = 115.713;     % [km]
s_diff_m     = (s_unfall_km - s0_km) * 1000; %[m]

% Ziel-Kollisionsgeschwindigkeit
v_koll_kmh   = 37.0;        % [km/h]




%%
%  TEIL a) Z-v-Diagramm der Lok

v_tab_kmh = (0:0.5:120)';               % Δv = 0.5 km/h
[Z_tab_kN, v_eck] = lok_Zv(v_tab_kmh);



%%
%  TEIL b) Bremskraftdiagramm eines Wagens

B_wagen_tab_kN = wagen_FBv(v_tab_kmh);







%%
%  TEIL c) Reine Beschleunigung bis Unfallort (ohne Bremsung)

fprintf('\n=== TEIL c) Beschleunigung (max. Zugkraft, ohne Bremsung) ===\n');

t_c   = 0;              % [s]
v_c   = 0;              % [km/ h]
s_c   = 0;              % [m]

t_hist_c = t_c;
v_hist_c = v_c;
s_hist_c = s_c;

dt = dt_beschl;

while (s_c < s_diff_m) && (v_c < 120 - 1e-6)  % bis Unfallort oder vmax
    % Zugkraft aus Z-v-Tabelle
    Z_kN = interp1(v_tab_kmh, Z_tab_kN, v_c, 'linear', 'extrap');
    
    % Laufwiderstand nach Strahl
    W_lauf_kN = laufwiderstand_strahl(v_c, m_ges);
    
    % Nettokraft
    F_net_kN = Z_kN - W_lauf_kN;
    
    % Beschleunigung
    a_ms2 = (F_net_kN * 1000) / (m_ges * 1000 * lambda); % N / (kg)
    
    % Euler-Schritt
    v_ms = v_c / 3.6;
    v_ms = v_ms + a_ms2 * dt;
    v_ms = max(v_ms, 0);
    v_c  = v_ms * 3.6;
    
    s_c = s_c + v_ms * dt;
    t_c = t_c + dt;
    
    t_hist_c(end+1,1) = t_c;
    v_hist_c(end+1,1) = v_c;
    s_hist_c(end+1,1) = s_c;
end

fprintf('Geschwindigkeit am Unfallort ohne Bremsung: %.2f km/h\n', v_c);
fprintf('Zeit bis Unfallort: %.1f s, Weg: %.1f m\n', t_c, s_c);


%  TEIL d/e) Gemeinsame Simulation Beschleunigung + Bremsung
%            und Iteration von v0 (Bremsbeginn)

fprintf('\n=== TEIL d/e) Beschleunigung + Bremsung, Bestimmung von v0 ===\n');

% Hilfsfunktion: für gegebenes v0 gesamte Fahrt bis Unfallort simulieren
% Rückgabe: Kollisionsgeschwindigkeit v_end_kmh und Gesamtweg s_total
simu_v0 = @(v0_kmh) simulate_full_run(v0_kmh, ...
    m_ges, lambda, F_lok_bremse, dt_beschl, dt_bremse, ...
    s_diff_m, g, v_tab_kmh, Z_tab_kN);

% Grobe Grenzen für v0: Zug muss schneller als 37 km/h gewesen sein
v0_low  = 40;   % [km/h]
v0_high = 120;  % [km/h]

% Bisection auf v0, so dass v_end ≈ 37 km/h
tol_v   = 0.1;      % Toleranz in km/h
max_it  = 40;

[v_end_low, ~]  = simu_v0(v0_low);
[v_end_high, ~] = simu_v0(v0_high);

% Sicherstellen, dass das Vorzeichen wechselt (eine Seite über, eine unter 37)
if (v_end_low - v_koll_kmh) * (v_end_high - v_koll_kmh) > 0
    warning('v0-Intervall trifft die Zielgeschwindigkeit 37 km/h nicht eindeutig.');
end

for it = 1:max_it
    v0_mid = 0.5 * (v0_low + v0_high);
    [v_end_mid, ~, t_bremse_mid, v_bremse_mid, s_bremse_mid] = simu_v0(v0_mid);
    
    if abs(v_end_mid - v_koll_kmh) < tol_v
        break;
    end
    
    if (v_end_low - v_koll_kmh) * (v_end_mid - v_koll_kmh) < 0
        v0_high   = v0_mid;
        v_end_high = v_end_mid;
    else
        v0_low    = v0_mid;
        v_end_low = v_end_mid;
    end
end

best_v0   = v0_mid;
best_vend = v_end_mid;
best_tb_path = t_bremse_mid;
best_vb_path = v_bremse_mid;
best_sb_path = s_bremse_mid;

fprintf('Gefundener Bremsbeginn v0*: %.2f km/h\n', best_v0);
fprintf('Geschwindigkeit am Unfallort nach Bremsung: %.2f km/h (Soll: 37)\n', best_vend);


%  PLOTS

figure('Position',[100 100 1200 800]);

%% 
% Z-v-Diagramm
subplot(2,3,1);

plot(v_tab_kmh, Z_tab_kN, 'b-', 'LineWidth', 1.5); 
hold on;

Z_eck = interp1(v_tab_kmh, Z_tab_kN, v_eck, 'linear');
plot(v_eck, Z_eck, 'go', 'MarkerSize', 8, 'LineWidth', 2);

grid on;
xlabel('v [km/h]');
ylabel('Z(v) [kN]');
title('Z-v-Diagramm Lok');
xlim([-5 120]);
ylim([0 1.1*max(Z_tab_kN)]);

% Bremskraftdiagramm Wagen
subplot(2,3,2);
plot(v_tab_kmh, B_wagen_tab_kN, 'b-', 'LineWidth', 2);
grid on;
xlabel('v [km/h]'); ylabel('B_W(v) [kN]');
title('Bremskraftdiagramm Wagen');
xlim([-5 120]);

% Beschleunigung: v(t) bis Unfallort (ohne Bremsung)
subplot(2,3,3);
plot(t_hist_c, v_hist_c, 'r-', 'LineWidth', 2);
grid on;
xlabel('t [s]'); ylabel('v [km/h]');
title('Beschleunigung: v(t) bis Unfallort (ohne Bremsung)');
ylim([0 v_c]);

% Beschleunigung: s(t)
subplot(2,3,4);
plot(t_hist_c, s_hist_c/1000, 'r-', 'LineWidth', 2);
grid on;
xlabel('t [s]'); ylabel('s [km]');
title('Beschleunigung: s(t)');

% Bremsung: v(t) für best_v0
subplot(2,3,5);
plot(best_tb_path, best_vb_path, 'b-', 'LineWidth', 2);
grid on;
xlabel('t [s]'); ylabel('v [km/h]');
title(sprintf('Bremsung: v(t), v_0* = %.1f km/h', best_v0));

% Bremsung: s(t)
subplot(2,3,6);
plot(best_tb_path, best_sb_path/1000, 'b-', 'LineWidth', 2);
grid on;
xlabel('t [s]'); ylabel('s [km]');
title('Bremsung: s(t) ab Bremsbeginn');

sgtitle('GdS Hausaufgabe 1: Fahrdynamiksimulation','FontSize',14,'FontWeight','bold');


%  ZUSAMMENFASSUNG

fprintf('\n=== ZUSAMMENFASSUNG ===\n');
fprintf('Gesamtmasse: %.0f t, Massenfaktor: %.2f\n', m_ges, lambda);
fprintf('Leistungseckpunkt Lok: v_eck = %.1f km/h\n', v_eck);
fprintf('Geschwindigkeit am Unfallort ohne Bremsung: %.2f km/h\n', v_c);
fprintf('Gefundener Bremsbeginn v0*: %.2f km/h\n', best_v0);
fprintf('Geschwindigkeit am Unfallort nach Bremsung: %.2f km/h (Soll: 37)\n', best_vend);
fprintf('Soll-Strecke Deedorf–Unfallort: %.1f m\n', s_diff_m);


%  LOKALE FUNKTION: gemeinsame Fahrt für gegebenes v0

function [v_end_kmh, s_total, t_bremse, v_bremse, s_bremse] = simulate_full_run( ...
    v0_kmh, m_ges, lambda, F_lok_bremse, dt_beschl, dt_bremse, ...
    s_diff_m, g, v_tab_kmh, Z_tab_kN)

    % 1) Beschleunigung bis v0 oder Bahnübergang
    t   = 0;
    v   = 0;
    s   = 0;
    dt  = dt_beschl;
    
    while (v < v0_kmh - 1e-3) && (s < s_diff_m)
        Z_kN = interp1(v_tab_kmh, Z_tab_kN, v, 'linear', 'extrap');
        W_lauf_kN = laufwiderstand_strahl(v, m_ges);
        F_net_kN  = Z_kN - W_lauf_kN;
        a_ms2     = (F_net_kN * 1000) / (m_ges * 1000 * lambda);
        v_ms      = v / 3.6;
        v_ms      = v_ms + a_ms2 * dt;
        v_ms      = max(v_ms, 0);
        v         = v_ms * 3.6;
        s         = s + v_ms * dt;
        t         = t + dt;
    end
    
    % wenn Unfallort schon erreicht, ist v_end = aktuelle v
    if s >= s_diff_m
        v_end_kmh = v;
        s_total   = s;
        t_bremse  = t;
        v_bremse  = v;
        s_bremse  = s;
        return;
    end
    
    % 2) Bremsung ab diesem Zeitpunkt bis Unfallort
    dt = dt_bremse;
    t_schwelle = 5.0;   % [s]
    
    t_bremse = 0;
    v_bremse = v;
    s_bremse = 0;
    
    t_hist = t_bremse;
    v_hist = v_bremse;
    s_hist = s_bremse;
    
    while s + s_bremse < s_diff_m && v_bremse > 0.1
        W_lauf_kN = laufwiderstand_strahl(v_bremse, m_ges);
        B_wagen_kN = wagen_FBv(v_bremse);
        B_ges_kN_max = F_lok_bremse + 20 * B_wagen_kN;
        
        if t_bremse < t_schwelle
            faktor = t_bremse / t_schwelle;
        else
            faktor = 1.0;
        end
        B_ges_kN = faktor * B_ges_kN_max;
        
        F_net_kN = -(B_ges_kN + W_lauf_kN);
        
        a_ms2 = (F_net_kN * 1000) / (m_ges * 1000 * lambda);
        
        v_ms = v_bremse / 3.6;
        v_ms = v_ms + a_ms2 * dt;
        v_ms = max(v_ms, 0);
        v_bremse = v_ms * 3.6;
        
        s_bremse = s_bremse + v_ms * dt;
        t_bremse = t_bremse + dt;
        
        t_hist(end+1,1) = t_bremse;
        v_hist(end+1,1) = v_bremse;
        s_hist(end+1,1) = s_bremse;
    end
    
    % Geschwindigkeit genau am Unfallort
    v_end_kmh = v_bremse;
    s_total   = s + s_bremse;
    
    % für Plots
    t_bremse = t_hist;
    v_bremse = v_hist;
    s_bremse = s_hist;
end