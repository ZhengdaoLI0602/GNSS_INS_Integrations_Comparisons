function results = AdaptiveR (GnssT, floatClass, receiver)

    this_uct = floatClass(find(floatClass(:,1) == GnssT), 2);
    if receiver == "novatel"
        min_uct = 1;
        max_uct = 45; 
    elseif receiver == "ublox" 
        min_uct = 1.5;
        max_uct = 65; 
    elseif receiver == "xiaomi"
        min_uct = 4;
        max_uct = 130;
    end
    
    if ~isempty(this_uct)  % has ml results %

        if this_uct < min_uct
              this_uct = min_uct;
        end
        results = this_uct;
    else                   % has no ml results %
        results = max_uct;
    end
end




