function Z_kN = tractionForceTrainSet(v_kmh)
% tractionForceTrainSet Lokzugkraft Z(v) in kN
% v_kmh: velocity in km/h

    % determine Leistungseckpunkt (numeric)
    persistent v_corner;
    if isempty(v_corner)
        v_corner = fzero(@(v) (300 - 0.25*v) - 6400./v, 60);  % use start value of 60 km/h
    end

    Z_kN = zeros(size(v_kmh));
    
    idx_low  = v_kmh <= v_corner;
    idx_high = v_kmh >  v_corner;

    Z_kN(idx_low)  = 300 - 0.25 .* v_kmh(idx_low);
    Z_kN(idx_high) = 6400 ./ v_kmh(idx_high);
end