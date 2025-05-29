%% Created by Egco 20/05/2025
function distance = LineToPointDistance(line_end, line_begin, point)
line_begin = line_begin(:);
line_end = line_end(:);
point = point(:);
segment_vec = line_end - line_begin;
begin_to_point_vec = point - line_begin;
segment_len_sq = dot(segment_vec, segment_vec);
if segment_len_sq < eps

    distance = norm(begin_to_point_vec);
    return;
end
t = dot(begin_to_point_vec, segment_vec) / segment_len_sq;
if t < 0
    distance = norm(begin_to_point_vec);
elseif t > 1
    end_to_point_vec = point - line_end;
    distance = norm(end_to_point_vec);
else
    numerator_magnitude = norm(cross(segment_vec, begin_to_point_vec));
    denominator_magnitude = sqrt(segment_len_sq);
    distance = numerator_magnitude / denominator_magnitude;
end
end
