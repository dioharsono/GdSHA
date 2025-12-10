function BW_kN = brakeForceWagon(v_kmh)
% brakeForceWagon Bremskraft eines Wagens in kN
% v_kmh: Velocity in km/h

    BW_kN = 25 ./ (0.1 .* v_kmh + 1) ...
          + 65 ./ (0.04 .* sqrt(v_kmh) + 1);
end