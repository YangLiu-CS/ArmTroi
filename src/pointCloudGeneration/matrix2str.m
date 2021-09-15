function [result_str] = matrix2str(data)
% translate the matrix type to string type for point cloud key

result_str = '[';
for i = 1 : size(data, 1)
    row_str = sprintf('%4f,', data(i, :));
    result_str = strcat(result_str, row_str(1:end-1), ';');
end
result_str = result_str(1: end-1);
result_str = strcat(result_str, ']');

end