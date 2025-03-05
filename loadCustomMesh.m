function h = loadCustomMesh(objFile, scaleFactor, position, rotationMatrix)
% loadCustomMesh - Load an OBJ file and apply scaling, rotation, and translation.
%
%   h = loadCustomMesh(objFile, scaleFactor, position, rotationMatrix)
%
%   Inputs:
%       objFile       - String, path to the OBJ file (e.g., 'drawings/building.obj').
%       scaleFactor   - Scalar to scale the vertex coordinates.
%       position      - 1x3 vector specifying the translation.
%       rotationMatrix- 3x3 rotation matrix. Pass eye(3) if no rotation needed.
%
%   Output:
%       h             - Handle to the patch object displaying the mesh.
%
% Note: This is a basic OBJ importer that reads vertices and faces.
%       For more robust support (e.g., texture coordinates), consider
%       using a File Exchange function like readObj.m.

    % Open the file.
    fid = fopen(objFile, 'r');
    if fid == -1
        error('Cannot open OBJ file: %s', objFile);
    end

    vertices = [];
    faces = [];
    % Read file line-by-line
    tline = fgetl(fid);
    while ischar(tline)
        if startsWith(tline, 'v ')
            % Parse a vertex line, e.g., "v 1.0 2.0 3.0"
            parts = sscanf(tline, 'v %f %f %f');
            vertices = [vertices; parts'];
        elseif startsWith(tline, 'f ')
            % Parse a face line. Faces may include slashes; we remove them.
            % Example face line: "f 1/1/1 2/2/2 3/3/3"
            parts = regexp(tline, 'f\s+([\d]+)/?[\d]*\s+([\d]+)/?[\d]*\s+([\d]+)/?[\d]*', 'tokens');
            if ~isempty(parts)
                f = str2double(parts{1});
                faces = [faces; f];
            else
                % If the above fails, try a simpler approach.
                parts = sscanf(tline, 'f %d %d %d');
                faces = [faces; parts'];
            end
        end
        tline = fgetl(fid);
    end
    fclose(fid);

    % Apply scaling
    vertices = vertices * scaleFactor;
    % Apply rotation
    vertices = (rotationMatrix * vertices')';
    % Apply translation
    vertices = vertices + repmat(position, size(vertices, 1), 1);

    % Create the patch object
    h = patch('Vertices', vertices, 'Faces', faces, ...
              'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none');
    % Set up material and lighting for realism
    material shiny;
    camlight('headlight');
    lighting gouraud;
end