function soc = getSOC(voltage, model)
    soc = find(model.soc_ocv >= voltage, 1);
end