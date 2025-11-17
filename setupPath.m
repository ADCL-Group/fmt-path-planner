function setupPath()
    % Get the path of this function
    thisDir = fileparts(mfilename('fullpath'));

    % List of subfolders to add
    folders = {'data', 'examples', 'src'};

    % Add each folder to the MATLAB path
    for i = 1:length(folders)
        folderPath = fullfile(thisDir, folders{i});
        if exist(folderPath, 'dir')
            addpath(genpath(folderPath));
        else
            fprintf('Warning: Folder not found: %s\n', folderPath);
        end
    end
end
