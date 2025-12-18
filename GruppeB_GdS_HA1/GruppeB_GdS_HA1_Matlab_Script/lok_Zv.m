function [Z_kN, v_eck] = lok_Zv(v_kmh)
% lok_Zv  Berechnet das Z-v-Diagramm der Lok in kN
% Eingabe:
%   v_kmh : Spaltenvektor der Geschwindigkeit [km/h]
% Ausgabe:
%   Z_kN  : Spaltenvektor der Zugkraft [kN]
%   v_eck : Leistungseckpunkt [km/h] (wird beim ersten Aufruf berechnet)

    % Berechne Leistungseckpunkt nur einmal
    persistent v_eck_cached;
    if isempty(v_eck_cached)
        v_eck_cached = fzero(@(v) (300 - 0.25*v) - 6400./v, 60);  % start value of 60 km/h
    end
    v_eck = v_eck_cached;
    Z_kN = zeros(size(v_kmh));
    
    i_low  = v_kmh <= v_eck_cached; % Kraftschlussbereich: Linearer Abfall
    i_high = v_kmh >  v_eck_cached; % Leistungsbereich: hyperbolische Form
    
    % Berechne Z(v) für alle Geschwindigkeiten
    Z_kN(i_low)  = 300 - 0.25 .* v_kmh(i_low);
    Z_kN(i_high) = 6400 ./ v_kmh(i_high); 
end