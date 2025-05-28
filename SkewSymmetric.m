%% Created by Egco 21/05/2025 

function S = SkewSymmetric(vector)
vector = vector(:);
    S = [  0   -vector(3)  vector(2);
          vector(3)   0   -vector(1);
         -vector(2)  vector(1)   0  ];
end