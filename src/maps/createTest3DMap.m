function [origMap, lowPadMap, highPadMap, mapLimits] = createTest3DMap(loPadding, hiPadding)
    % Create Map 
    % Draw the map based on each geometrical obstacle that has different
    % altitude (z-coordinate).

    % Build 3D map with defined Occupancy
    n = 70;                   % samples per axis
    resolution = 0.1;
    worldSize  = 700;
    
    origMap    = occupancyMap3D(resolution);
    lowPadMap  = occupancyMap3D(resolution);
    highPadMap = occupancyMap3D(resolution);

    x = linspace(0, worldSize, n)';
    y = linspace(0, worldSize, n)';
    z = linspace(0, worldSize, n)';

    nx = numel(x); ny = numel(y); nz = numel(z);

    % voxels to pad
    loIdx = ceil(loPadding * origMap.Resolution);
    hiIdx = ceil(hiPadding * origMap.Resolution);

    % occupancy arrays
    occTrue  = false(ny, nx, nz);
    occLo    = false(ny, nx, nz);
    occHi    = false(ny, nx, nz);

    % Coordinates are read in this format: occ3D(Y,X,Z)
    % Define buildings by world limits [Xmin Xmax Ymin Ymax Zmax]
    blocks = [ ...
       60 160 60 160 440; % Building #1
      100 300 200 400 190; % Building #2
      10 90 470 650 170; % Building #3
      90 130 530 590 190; % Building #3
      200 260 540 610 300; % Building #4
      260 320 540 610 240; % Building #4
      260 320 480 540 240; % Building #4
      430 560 560 690 100; % Building #5
      400 460 270 390 80; % Building #6
      380 410 270 390 40; % Building #6
      460 480 350 390 70; % Building #6
      460 480 270 310 70; % Building #6
      600 650 120 220 400; % Building #7
      570 600 140 220 400; % Building #7
      500 570 120 240 400 % Building #7
    ];

    % Define the occupancy of each building
    for k = 1:size(blocks,1)
        X0 = blocks(k,1); X1 = blocks(k,2);
        Y0 = blocks(k,3); Y1 = blocks(k,4);
        Z1 = blocks(k,5);

        % convert world‐limits to index ranges
        ix = find(x >= X0,1,'first') : find(x <= X1,1,'last');
        iy = find(y >= Y0,1,'first') : find(y <= Y1,1,'last');
        iz = 1 : find(z <= Z1,1,'last');  % ground to Z1

        % true occupancy
        occTrue(iy, ix, iz) = true;

        % Low padding indices (clamped)
        ix0 = max(1, ix(1)-loIdx);
        ix1 = min(nx, ix(end)+loIdx);
        iy0 = max(1, iy(1)-loIdx);
        iy1 = min(ny, iy(end)+loIdx);
        iz0 = 1;
        iz1 = min(nz, iz(end)+loIdx);
        occLo(iy0:iy1, ix0:ix1, iz0:iz1) = true;

        % High padding indices (clamped)
        ix0h = max(1, ix(1)-hiIdx);
        ix1h = min(nx, ix(end)+hiIdx);
        iy0h = max(1, iy(1)-hiIdx);
        iy1h = min(ny, iy(end)+hiIdx);
        iz0h = 1;
        iz1h = min(nz, iz(end)+hiIdx);
        occHi(iy0h:iy1h, ix0h:ix1h, iz0h:iz1h) = true;
    end

    % Ground
    occTrue(:,:,1) = true;
    occLo(:,:,1)   = true;
    occHi(:,:,1)   = true;

    % Flatten and set occupancy
    [Xg,Yg,Zg] = ndgrid(x,y,z);
    pts = [Xg(:), Yg(:), Zg(:)];

    setOccupancy(origMap,    pts, double(occTrue(:)));
    setOccupancy(lowPadMap,  pts, double(occLo(:)));
    setOccupancy(highPadMap, pts, double(occHi(:)));

    % Map limits
    mapLimits = [ x(1), x(end);
                  y(1), y(end);
                  z(1), z(end) ];
end
