function geom = loadCustomMesh(objFile, scaleFactor, position, rotationMatrix)
    % Check if file exists
    if ~exist(objFile, 'file')
        error('File does not exist: %s', objFile);
    end
    
    try
        % Read the OBJ file
        fid = fopen(objFile, 'r');
        if fid == -1
            error('Cannot open OBJ file: %s', objFile);
        end
        
        vertices = [];
        faces = [];
        
        tline = fgetl(fid);
        while ischar(tline)
            if startsWith(tline, 'v ')
                % Read vertex data (x, y, z)
                data = sscanf(tline, 'v %f %f %f');
                if numel(data) >= 3
                    vertices = [vertices; data(1:3)'];
                end
            elseif startsWith(tline, 'f ')
                % Parse face data (handles both "f v1 v2 v3" and "f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3" formats)
                if contains(tline, '/')
                    % Format with texture/normal indices
                    parts = regexp(tline, '\d+', 'match');
                    if length(parts) >= 3
                        f = [str2double(parts{1}), str2double(parts{4}), str2double(parts{7})];
                        faces = [faces; f];
                    end
                else
                    % Simple format
                    parts = sscanf(tline, 'f %d %d %d');
                    if length(parts) >= 3
                        faces = [faces; parts(1:3)'];
                    end
                end
            end
            tline = fgetl(fid);
        end
        fclose(fid);
        
        % Apply transformations
        % First scale
        vertices = vertices * scaleFactor;
        
        % Then rotate (use the rotationMatrix)
        vertices = (rotationMatrix * vertices')';
        
        % Then translate
        vertices = vertices + repmat(position, size(vertices, 1), 1);
        
        % Ensure faces are using 1-based indexing (OBJ can use 0-based)
        if ~isempty(faces) && min(faces(:)) == 0
            faces = faces + 1;
        end
        
        % Create the output structure in the correct format for updateMesh
        geom.vertices = vertices;
        geom.faces = faces;
        
    catch e
        fclose(fid);
        error('Error processing OBJ file: %s - %s', objFile, e.message);
    end
end