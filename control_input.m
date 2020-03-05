function input = control_input(data,time,dt)
    if length(data) ~= length(time)
        error('Data and time series should be of equal length');
    else
        t = [time(1):dt:time(end)];
        input = zeros(length(t),1);
        idx = 1;
        for i=1:length(time)-1
            k = find(abs(t-round(time(i+1)))<0.1);
            input(idx:k) = data(i);
            idx = k;
        end
    end
end