function gusts = gust_generator(period, amplitude, enabled, t)

gusts = [0,0,0];
idx = 0;

if enabled == 1
    % general wind
    for i = 1:3
        gusts(i) = randn;
    end

    if mod(t,period) == 0
        if randn > 0
            select = rand;
            if select < .3333
                idx = 1;
            elseif select > .6666
                idx = 3;
            else
                idx = 2;
            end % create a gust
            gusts(idx) = normrnd(0, amplitude);
        end
    end
end

