function [X, Y, Z] = capsule_surface(p1, p2, radius, n_axial, n_radial)
%CAPSULE_SURFACE  Surface mesh of a capsule, ready for SURF.
%
%   [X, Y, Z] = capsule_surface (P1, P2, RADIUS) builds the surface of the set
%   of points whose distance to the segment P1-P2 is RADIUS: a cylinder of
%   radius RADIUS capped with two hemispheres.  The mesh is closed, so it can
%   be drawn with SURF without holes at the ends.
%
%   [X, Y, Z] = capsule_surface (P1, P2, RADIUS, N_AXIAL, N_RADIAL) also sets
%   the mesh resolution.  Defaults are 12 axial and 24 radial divisions.
%
%   See also SPHERE, SURF, CAPSULE_OBSTACLE.

if nargin < 4 || isempty(n_axial)
    n_axial = 12;
end
if nargin < 5 || isempty(n_radial)
    n_radial = 24;
end

p1 = p1(:);
p2 = p2(:);
axis_vector = p2 - p1;
length_ = norm(axis_vector);

if length_ < eps
    axis_vector = [0; 0; 1];
else
    axis_vector = axis_vector / length_;
end

% Orthonormal basis of the plane perpendicular to the capsule axis.
reference = [0; 0; 1];
if abs(axis_vector.' * reference) > 0.9
    reference = [1; 0; 0];
end
e1 = cross(axis_vector, reference);
e1 = e1 / norm(e1);
e2 = cross(axis_vector, e1);

% Axial coordinate running from one spherical cap to the other.
s = linspace(-radius, length_ + radius, 2 * n_axial + 1);
profile = radius * ones(size(s));
cap_start = s < 0;
profile(cap_start) = sqrt(max(radius^2 - s(cap_start).^2, 0));
cap_end = s > length_;
profile(cap_end) = sqrt(max(radius^2 - (s(cap_end) - length_).^2, 0));

% The caps are centred on the segment ends, so the axial coordinate S runs
% past both ends and the profile above closes to zero exactly at the poles.
base = p1 + s .* axis_vector;

angle = linspace(0, 2 * pi, n_radial + 1).';
cos_angle = cos(angle) * profile;
sin_angle = sin(angle) * profile;

X = repmat(base(1, :), numel(angle), 1) + cos_angle * e1(1) + sin_angle * e2(1);
Y = repmat(base(2, :), numel(angle), 1) + cos_angle * e1(2) + sin_angle * e2(2);
Z = repmat(base(3, :), numel(angle), 1) + cos_angle * e1(3) + sin_angle * e2(3);

end

%!test
%! % the mesh really lies on the capsule: every vertex is RADIUS away from
%! % the axis segment
%! p1 = [0.1; -0.2; 0.3];
%! p2 = [0.4; 0.5; 0.7];
%! radius = 0.07;
%! [X, Y, Z] = capsule_surface(p1, p2, radius);
%! for k = 1:numel(X)
%!     point = [X(k); Y(k); Z(k)];
%!     [~, ~, ~, closest, ~] = segment_segment_distance(p1, p2, point, point);
%!     assert(abs(norm(point - closest) - radius) < 1e-9);
%! end

%!test
%! % a degenerate capsule is a sphere
%! [X, Y, Z] = capsule_surface([0;0;0], [0;0;0], 0.5);
%! assert(max(sqrt(X(:).^2 + Y(:).^2 + Z(:).^2)), 0.5, 1e-9);
