%% Grundlagen der Spürführung
% Hausaufgabe 1
% Stand: 08.12.2025


%% empty workspace
clear; clc; close all;
%% Define Parameters

%Train Parameter
mass_loco_t = 90;       % in t
mass_wagon_t = 90;      % in t
number_wagon = 20;      
mass_total_train_set = mass_loco_t + (20 * number_wagon * mass_wagon_t); % in t
mass_total_kg = mass_total_train_set * 1000; % in kg

mass_factor = 1.04;
mass_dynamic = mass_factor * mass_total_kg;

gravity = 9.81; % in m/s^2

% Distance
start_kilometer = 112.130;
crash_kilometer = 115.713;
total_distance= (crash_kilometer - start_kilometer) * 1000 ; % given distance in m

% Collision Metric
v_collision_kmh = 37; % in km/h
v_collision_ms = v_collision_kmh / 3.6; % in m/s


%% (a) Z-v-Diagram of the train set
v_kmh = 0:0.5:120; % in 0.5 km/h increment
Z_kN  = tractionForceTrainSet(v_kmh);

figure;
plot(v_kmh, Z_kN, 'LineWidth', 1.5);
grid on;
xlabel('v [km/h]');
ylabel('Traction Force Locomotive Z(v) [kN]');
title('Z-v-Diagram of the Locomotive');

%% (b) Brake Force Diagram of a wagon
BW_kN = brakeForceWagon(v_kmh);

figure;
plot(v_kmh, BW_kN, 'LineWidth', 1.5);
grid on;
xlabel('v [km/h]');
ylabel('Wagon Brakeforce B_W(v) [kN]');
title('Wagon Brakeforce Diagram');