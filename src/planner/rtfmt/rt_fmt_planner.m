function S = rt_fmt_planner(map, limits, start, goal, rn, varargin)

p = inputParser;
p.FunctionName = mfilename;

% Required
addRequired(p, 'map');
addRequired(p, 'limits');
addRequired(p, 'start');
addRequired(p, 'goal');
addRequired(p, 'rn', @(x) isscalar(x) && x > 0 && isfinite(x));

% Optional name-value params
addParameter(p, 'N', [], @(x) isempty(x) || (isscalar(x) && x >= 0 && isfinite(x)));
addParameter(p, 'w', 0, @(x) isscalar(x) && isfinite(x));
addParameter(p, 'goalRadius', NaN, @(x) isscalar(x) && (isnan(x) || (x >= 0 && isfinite(x))));
addParameter(p, 'expandTreeRate', 32, @(x) isscalar(x) && x > 0 && isfinite(x));
addParameter(p, 'safeRadiusDObstacle', NaN, @(x) isscalar(x) && (isnan(x) || (x >= 0 && isfinite(x))));
addParameter(p, 'flightParams', [], ...
    @(x) isempty(x) || (isnumeric(x) && isvector(x) && numel(x) >= 4));

parse(p, map, limits, start, goal, rn, varargin{:});
opts = p.Results;

if isempty(opts.N)
    opts.N = computeSamples(opts.w, limits);   % default N depends on w and limits
end

S = struct();

if ~isempty(opts.flightParams)
    % Create the Dubins connector
    fp = opts.flightParams;
    S.conn = uavDubinsConnection( ...
        'MaxRollAngle',        fp(1), ...
        'AirSpeed',            fp(2), ...
        'FlightPathAngleLimit', fp(3:4));
else
    S.conn = [];
end

% Planner params
S.map            = map;
S.limits         = limits;
S.start          = start(:).';
S.goal           = goal(:).';
S.w              = opts.w;
S.N              = opts.N + 2;   % start/goal added inside sampler
S.expandTreeRate = opts.expandTreeRate;

% Sample free space
S.V  = sampleFree(S.map, S.limits, S.start, S.goal, opts.N, []);                                         
S.rn = opts.rn;

if isnan(opts.goalRadius)
    S.goalRadius = S.rn/2;
else
    S.goalRadius = opts.goalRadius;
end

if isnan(opts.safeRadiusDObstacle)
    S.safeRadiusDObstacle = S.rn;
else
    S.safeRadiusDObstacle = opts.safeRadiusDObstacle;
end

% Indices
allIdx        = (1:size(S.V,1))';
S.startIdx    = find(ismember(S.V, S.start, 'rows'), 1);
S.goalIdx     = find(ismember(S.V, S.goal,  'rows'), 1);
S.rootIdx     = S.startIdx;

% Goal region
S.goalRegionIdx = unique([S.goalIdx; near(S.V, S.V(S.goalIdx,:), S.goalRadius, S.w)]);
     
% Sets and flags
% States: 0=Undefined, 1=Open, 2=Unvisited, 3=Closed
S.parent  = zeros(size(S.V,1),1);
S.cost    = inf(size(S.V,1),1);
S.state   = 2*ones(size(S.V,1),1); % Unvisited
S.state(S.startIdx) = 1;           % Start in Open

S.W       = setdiff(allIdx, S.startIdx);   % Unvisited
S.isOpen  = false(size(S.V,1),1);          % Open set mask
S.isOpen(S.startIdx) = true;
S.z       = S.startIdx;                    % current node
S.cost(S.startIdx) = 0;                                                            

% Lists used by RT updates (blocked nodes, checked path, etc.)
S.blocked        = false(size(S.V,1),1);   % dynamic obstacle mask
S.openNew        = false(size(S.V,1),1);
S.closedToOpen   = false(size(S.V,1),1);
S.checkedPath    = false(size(S.V,1),1);
S.checkedPathCandidates = [];

end