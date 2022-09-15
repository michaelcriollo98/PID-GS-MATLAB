function new_gain = CreateNeighbor(gain,range_gains)
Kpmin = range_gains(1);
Kpmax = range_gains(2);
Kdmin = range_gains(3);
Kdmax = range_gains(4);
Kimin = range_gains(5);
Kimax = range_gains(6);
new_gain = gain;
pKp = 0.2;
pKi = 0.5;
pKd = 1-pKp-pKi;
r = rand;
c = cumsum([pKp pKi pKd]);
variacion  = find(r <= c, 1, 'first');
    switch variacion
        case 1
            Kp = Kpmin + (Kpmax-Kpmin)*rand;
            new_gain(1) = Kp;
        case 2
            Ki = Kimin + (Kimax-Kimin)*rand;
            new_gain(2) = Ki;
        case 3
            Kd = Kdmin + (Kdmax-Kdmin)*rand;
            new_gain(3) = Kd; 
    end 
end