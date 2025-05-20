%% Created by Egco 20/05/2025 
% reference https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html

function distance = LineToPointDistance(line_end,line_begin,point)
numerator = abs(cross((line_begin-line_end),(line_begin-point)));
denominator = abs((point-line_end));
distance = numerator / denominator;
return;
end