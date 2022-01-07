function args = readJson(ds)

    if ds == 0
        fileName = 'kitti.json'
    elseif ds == 1
        fileName = 'malaga.json'
    else
        fileName = 'parking.json'
    end


    fid = fopen(fileName); % Opening the file
    raw = fread(fid,inf); % Reading the contents
    str = char(raw'); % Transformation
    fclose(fid); % Closing the file
    args = jsondecode(str); % Using the jsondecode function to parse JSON from string

end
