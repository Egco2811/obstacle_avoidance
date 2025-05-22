%% Created by Egco 21/05/2025 

function RotationMatrix = RotateBy(axis,radian)
    axis = axis(:) / norm(axis);
    khat = SkewSymmetric(axis);
    RotationMatrix = eye(3) + sin(radian)*khat + (1-cos(radian))*(khat*khat);
end