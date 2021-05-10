function num_parameter = str2intarray(str)
    parameter = split(str,";");
    parameter = transpose(parameter);
    for i = 1:length(parameter)
        num_parameter(i) = str2num(parameter(i));
    end
end