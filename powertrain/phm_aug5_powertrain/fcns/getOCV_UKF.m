function ocv = getOCV_UKF(soc, model)
    for s = 1:length(soc)
        if soc(s) < 0.0
            soc(s) = 0;
        elseif soc(s) > 1.0
            soc(s) = 1.0;
        end
    end

    idx = ceil(soc*100);

    for i=1:length(idx)
        if idx(i) > 101
            idx(i) = 101;
        elseif idx(i) < 1
            idx(i) = 1;
        end
    end
    ocv = model.soc_ocv(idx);
end