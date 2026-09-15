function [X, Y, Z] = sphere_surface(center, radius, n_axial, n_radial)
%SPHERE_SURFACE  Surface mesh of a sphere, ready for SURF.
%
%   [X, Y, Z] = sphere_surface (CENTER, RADIUS) is a latitude/longitude mesh of
%   a sphere.  It is a thin wrapper around CAPSULE_SURFACE with a degenerate
%   axis, which keeps every obstacle mesh in this project on the same code path
%   and the same vertex ordering.
%
%   The wrapper also exists for an Octave specific reason: the mesh returned by
%   the built in SPHERE function is not rendered by the FLTK print path, which
%   is what a plain Octave installation uses to write PNG files.  The mesh
%   built here renders correctly.
%
%   [X, Y, Z] = sphere_surface (CENTER, RADIUS, N_AXIAL, N_RADIAL) also sets
%   the resolution.  Defaults are 12 axial and 24 radial divisions.
%
%   See also CAPSULE_SURFACE, SURF, SPHERE_OBSTACLE.

if nargin < 3 || isempty(n_axial)
    n_axial = 12;
end
if nargin < 4 || isempty(n_radial)
    n_radial = 24;
end

center = center(:);
[X, Y, Z] = capsule_surface(center, center, radius, n_axial, n_radial);

end

%!test
%! % every vertex is exactly RADIUS away from the centre
%! center = [0.1; -0.2; 0.3];
%! radius = 0.075;
%! [X, Y, Z] = sphere_surface(center, radius);
%! distance = sqrt((X - center(1)).^2 + (Y - center(2)).^2 + (Z - center(3)).^2);
%! assert(max(abs(distance(:) - radius)), 0, 1e-9);

%!test
%! % the mesh is closed: it reaches the north and south poles
%! [X, Y, Z] = sphere_surface([0; 0; 0], 1);
%! assert(max(Z(:)), 1, 1e-9);
%! assert(min(Z(:)), -1, 1e-9);
