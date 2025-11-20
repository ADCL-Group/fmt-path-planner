function result = buildOccupancyMap(filename, resolution, loPadding, hiPadding)
% buildOccupancyMap - Creates a 3D occupancy map from OpenStreetMap building data.
% Credits:
%   John Stephen Clardy (clardyj@my.erau.edu)
%   Edison Martinez (martie14@my.erau.edu)
%   Embry-Riddle Aeronautical University, 2025

[osmFolder, baseName, ~] = fileparts(filename);     % e.g. "data/osm", "boston"
dataRoot = fileparts(osmFolder);                    % e.g. "data"

if isempty(dataRoot)
    dataRoot = osmFolder;
end

% Cache and bt directories under the data root
cacheDir  = fullfile(dataRoot, 'mat');              % e.g. "data/mat"
outputDir = fullfile(dataRoot, 'bt');               % e.g. "data/bt"

if ~exist(cacheDir, 'dir')
    mkdir(cacheDir);
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

cacheFile = fullfile(cacheDir, baseName + ".mat");

% funcFolder = fileparts( mfilename('fullpath') );
% cacheDir = fullfile(funcFolder, 'mat');
% if ~exist(cacheDir, 'dir')
%     mkdir(cacheDir);
% end
% 
% [~, baseName, ~] = fileparts(filename);
% cacheFile = fullfile(cacheDir, baseName + ".mat");

if isfile(cacheFile)
    S = load(cacheFile, 'result');
    if isfield(S, 'result') && isfield(S.result, 'padding')
        padsCached = S.result.padding;  % [0, lo, hi]
        if numel(padsCached) == 3 && padsCached(2) == loPadding && padsCached(3) == hiPadding
            % padding matches, return cached
            result = S.result;
            return;
        end
    end
end

% If we don't have the save mat file, build everything
% Step 1: Read Buildings from OpenStreetMap File
buildings = readgeotable(filename, Layer="buildingparts");

% Step 2: Convert geospatial table to regular table with Lat and Lon columns
buildingTable = geotable2table(buildings, ["Lat", "Lon"]);

% Step 3: Compute Map Origin from All Coordinates
allLats = []; allLons = [];
for i = 1:height(buildingTable)
    lat = buildingTable.Lat{i}; lon = buildingTable.Lon{i};
    allLats = [allLats; lat(:)];  %#ok<AGROW>
    allLons = [allLons; lon(:)];  %#ok<AGROW>
end
allLats = allLats(~isnan(allLats));
allLons = allLons(~isnan(allLons));
origin = [mean(allLats), mean(allLons), 0];

% Step 4: Create 3D Occupancy Map
pads = [0, loPadding, hiPadding];
nPad = numel(pads);

result.origin = origin;
result.padding = pads;
result.maps = cell(1,nPad);
result.occ = cell(1,nPad);

% create empty maps
for k=1:nPad
    result.maps{k} = occupancyMap3D(resolution);
    result.occ{k}  = [];
end

% Step 5: Extrude Buildings and Populate Map
for i = 1:height(buildingTable)
    % Extract building footprint coordinates
    lat = buildingTable.Lat{i}; lon = buildingTable.Lon{i};

    % Remove NaNs
    valid = ~isnan(lat) & ~isnan(lon);
    lat = lat(valid); lon = lon(valid);

    if isempty(lat), continue; end

    % Convert to local Cartesian coordinates
    [x, y, ~] = latlon2local(lat, lon, zeros(size(lat)), origin);
    
    % Define building height (modify as needed)
    % Try to get real building height from the attribute
    % Use MaxHeight from the table if available, otherwise fallback
    if ~isempty(buildingTable.MaxHeight(i)) && ~isnan(buildingTable.MaxHeight(i))
        h = buildingTable.MaxHeight(i);
    elseif ~isempty(buildingTable.MinHeight(i)) && ~isnan(buildingTable.MinHeight(i))
        h = buildingTable.MinHeight(i) + 5;  % Estimate
    else
        h = 10;  % Default fallback
    end

    % loop over pad levels
    for pIdx = 1:nPad
        pad = pads(pIdx);

        % expand the grid extents by pad in x&y
        xMin = floor(min(x)) - pad;
        xMax = ceil (max(x)) + pad;
        yMin = floor(min(y)) - pad;
        yMax = ceil (max(y)) + pad;
        zMax = h + pad;

        % define the 2D grid for this building
        xi = xMin:1/resolution:xMax;
        yi = yMin:1/resolution:yMax;
        [X2, Y2] = meshgrid(xi, yi);
        if pad>0
            coords = unique([x(:), y(:)], 'rows', 'stable');
            coords = uniquetol(coords, 1e-6, 'ByRows', true);

            oldState = warning('off','MATLAB:polyshape:repairedBySimplify');
            shp = polyshape(coords(:,1), coords(:,2), 'Simplify', true);
            warning(oldState);

            padShp = polybuffer(shp, pad);
            [bx,by] = boundary(padShp);
            mask2d = inpolygon(X2, Y2, bx, by);
        else
            mask2d = inpolygon(X2, Y2, x, y);
        end

        pts2D = [ X2(mask2d), Y2(mask2d) ];
        if isempty(pts2D), continue; end

        zLevels   = 0:1/resolution:zMax;
        
        % preallocate total
        n2D   = size(pts2D,1);
        nZ    = numel(zLevels);
        occPts = zeros(n2D * nZ, 3);
        
        idx = 1;
        for zi = 1:nZ
          zval = zLevels(zi);
          range = idx:idx + n2D - 1;
          occPts(range, :) = [pts2D, zval*ones(n2D,1)];
          idx = idx + n2D;
        end
        
        % now set only the points
        try
            % No need to set occupancy here as we will do it later
            % thisMap= result.maps{pIdx};
            % setOccupancy(thisMap, occPts, 1);
            result.occ{pIdx} = [result.occ{pIdx}; occPts];
        catch
            disp(i);
        end

    end
end

% Compute world‐limits
origOcc = result.occ{1};
result.mapLimits = [ floor(min(origOcc(:,1))), ceil(max(origOcc(:,1)));
                     floor(min(origOcc(:,2))), ceil(max(origOcc(:,2)));
                     floor(min(origOcc(:,3))), ceil(max(origOcc(:,3)))+150 ];

xMin = result.mapLimits(1,1);
xMax = result.mapLimits(1,2);
yMin = result.mapLimits(2,1);
yMax = result.mapLimits(2,2);
zMin = result.mapLimits(3,1);
zMax = result.mapLimits(3,2);

% For each padding-version of the map, fill all voxels as FREE
for pIdx = 1:nPad
    map3D  = result.maps{pIdx};
    occPts = result.occ{pIdx};

    % Build vectors along x, y, z
    xVec = xMin : 1/resolution : xMax;
    yVec = yMin : 1/resolution : yMax;
    zVec = zMin : 1/resolution : zMax;

    [X, Y, Z] = meshgrid(xVec, yVec, zVec);
    allPts = [X(:), Y(:), Z(:)];

    % Mark all of those voxels as free
    setOccupancy(map3D, allPts, 0);

    % Re‐mark building voxels as occupied
    if ~isempty(occPts)
        setOccupancy(map3D, occPts, 1);
    end

    result.maps{pIdx} = map3D;
end

btFile = fullfile(outputDir, baseName + ".bt");

try
    exportOccupancyMap3D(result.maps{1}, btFile);
catch
    disp("You must change 'outputfilename' or delete the existing file to save changes");
end

try
    save(cacheFile, 'result');
catch
    warning('Could not save cache to %s', cacheFile);
end

% outputDir = fullfile(funcFolder, 'bt');
% if ~exist(outputDir, 'dir')
%     mkdir(outputDir);
% end
% 
% btFile = fullfile(outputDir, baseName+".bt");
% 
% try
%     exportOccupancyMap3D(result.maps{1}, btFile);
% catch
%     disp("You must change 'outputfilename' or delete the existing file to save changes");
% end
% 
% try
%     save(cacheFile, 'result');
% catch
%     warning('Could not save cache to %s', cacheFile);
% end


end