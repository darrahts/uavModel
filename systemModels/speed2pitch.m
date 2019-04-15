function [pitch] = speed2pitch(speed)

    pitch = (2.10471847e-03)*speed + (5.35884537e-05)*speed^2;
    pitch = pitch - pitch*.27;
    pitch = round(pitch, 3);
end

