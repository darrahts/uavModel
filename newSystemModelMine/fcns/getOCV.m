function ocv = getOCV(soc, model)
    if soc < 0.0
        soc = 0;
    elseif soc > 1.0
        soc = 1.0;
    end
    
    idx = ceil(soc*100);
    
    if idx > 101
        idx = 101;
    elseif idx < 1
        idx = 1;
    end
    ocv = model.soc_ocv(idx);
end