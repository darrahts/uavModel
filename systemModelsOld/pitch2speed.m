function [speed] = pitch2speed(pitch)

    speed = 693.69555308*pitch - 11826.75247458*pitch^2 + 145800.25449591*pitch^3;
    speed = round(speed,3);
end