function FB_kN = wagen_FBv(v_kmh)
%% wagen_FBv  Bremskraft eines Wagens in Abhängigkeit von Geschwindigkeit
% Eingabe:
%   v_kmh : Geschwindigkeit [km/h]
% Ausgabe:
%   FB_kN : Bremskraft [kN]
%
% Formel (hyperbolisch):
%  F_B(v) = 25 kN / (0.1*(h/km)*v + 1) + 65 kN / (0.4*(h/km)^0.5*sqrt(v) + 1)

    v = v_kmh(:);   % als Spaltenvektor
    

    % Summe
    FB_kN = 25 ./ (0.1 * v + 1) + 65 ./ (sqrt(0.04 * v) + 1);
    
    % Ergebnis auf gleiche Größe wie Eingabe bringen
    FB_kN = reshape(FB_kN, size(v_kmh));
end