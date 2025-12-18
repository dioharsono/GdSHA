function W_lauf_kN = laufwiderstand_strahl(v_kmh, m_ges_t)
% Laufwiderstand nach Strahl
% v_kmh   : Geschwindigkeit [km/h]
% m_ges_t : Zugmasse [t]
% Ausgabe : W_lauf_kN [kN]

    % c1 und c2 aus Übung 2
    c1 = 1.4;   % Rollenlager
    c2 = 0.040;   % Ganzzug aus geschlossenen Wagen

    w_L = c1 + (0.007 + c2) .* (v_kmh/10).^2;   % [‰]
    W_lauf_kN = w_L .* m_ges_t * 9.81 / 1000;  % [kN]
end

