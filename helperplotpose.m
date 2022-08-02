function hh = helperplotpose(varargin)
%HELPERPLOTPOSE 3-D pose plot.
%
%   This function is for internal use only. It may be removed in the 
%   future.
%
%   HELPERPLOTPOSE(Q) plots the pose specified by the quaternion Q. The
%   axes are set to the ENU frame. 
%
%   HELPERPLOTPOSE(R) plots the pose specified by the rotation matrix R.
%
%   HELPERPLOTPOSE(..., POS) plots the pose with the position specified by
%   the 3-element vector POS.
%
%   HELPERPLOTPOSE(T) plots the pose specified by the homogeneous transform 
%   matrix T.
%
%   HELPERPLOTPOSE(..., F) plots the pose in the navigation frame specified
%   by F. F can be 'ENU' or 'NED' for the East-North-Up and North-East-Down
%   frames, respectively. The default value is 'ENU'.
%
%   HELPERPLOTPOSE(AX,...) plots into the axes with handle AX.
%
%   HELPERPLOTPOSE returns a handle to a HelperPosePatch object.
%
%   Examples:
%       % Visualize quaternion.
%       q = quaternion([30 20 10], 'eulerd', 'ZYX', 'frame');
%       helperplotpose(q)
% 
%       % Visualize pose matrix.
%       R = rotmat(quaternion([4 52 9], 'eulerd', 'ZYX', 'frame'), ...
%           'frame');
%       helperplotpose(R)     
%
%       % Visualize pose between two quaternions.
%       q = quaternion([80 20 10; 20 2 5], 'eulerd', 'ZYX', 'frame');
%       r = helperplotpose(q(1));
%       for t = 0:0.01:1
%           qs = slerp(q(1), q(2), t);
%           set(r, 'Orientation', qs);
%           drawnow
%       end
%
%   See also surf.

%   Copyright 2019 The MathWorks, Inc.

[parent, args] = axescheck(varargin{:});

[navframe, args, nargs] = navframecheck(args{:});

if (nargs > 2)
    error(message('MATLAB:narginchk:tooManyInputs'));
end

orient = quaternion.ones;
pos = [0 0 0];
if (nargs == 2) % plotpose(q, pos) or plotpose(R, pos)
    orient = validateorientation(args{1});
    pos = validateposition(args{2});
elseif (nargs == 1)
    NUM_TRANSFORM_ELEMENTS = 16;
    arg = args{1};
    if ( (isa(arg, 'double') || isa(arg, 'single')) ...
            && (numel(arg) == NUM_TRANSFORM_ELEMENTS) ) % plotpose(T)
        [orient, pos] = validatetransform(arg);
    else % plotpose(q) or plotpose(R)
        orient = validateorientation(arg);
    end
end

if isempty(parent) || ishghandle(parent, 'axes')
    ax = newplot(parent);
else
    ax = ancestor(parent, 'axes');
end

h = HelperPosePatch('Parent', ax, ...
    'Orientation', orient, 'Position', pos);

if any(strcmpi(ax.NextPlot, {'replace', 'replaceall'}))
    configureaxes(ax, navframe);
end

% Remove object from figure code generation.
if ~isempty(h)
    if ~isdeployed
        makemcode('RegisterHandle', h.Parent, 'IgnoreHandle', h, ...
            'FunctionName', 'plotpose');
    end
end

if (nargout > 0) 
    hh = h;
end

end

function configureaxes(ax, frame)
%CONFIGUREAXES Setup axes for PosePatch plotting.

axis(ax, 'equal');

if strcmp(frame, 'NED') % Navigation frame is NED.
    ax.YDir = 'reverse';
    ax.ZDir = 'reverse';
end

grid(ax, 'on');
grid(ax, 'minor');

% Default 3-D viewing angle.
view(ax, 3);
end

function [navframe, args, nargs] = navframecheck(varargin)
%NAVFRAMECHECK Check if the last input argument is a navigation frame 
%   string.

navframe = 'ENU';
args = varargin;
nargs = nargin;
if (nargs > 0)
    lastArg = args{end};
    if isa(lastArg, 'string') || isa(lastArg, 'char')
        navframe = validatestring(lastArg,  {'ENU', 'NED'}, ...
            'plotpose', 'navframe');
        nargs = nargs - 1;
        args = args(1:end-1);
    end
end
end

function orient = validateorientation(val)

if isa(val, 'quaternion')
    validateattributes(val, {'quaternion'}, {'scalar'});
else
    validateattributes(val, {'double', 'single'}, ...
        {'real', '2d', 'nrows', 3, 'ncols', 3});
end

orient = val;
end

function pos = validateposition(val)

validateattributes(val, {'double', 'single'}, ...
    {'real', 'vector', 'numel', 3});

pos = val(:).';
end

function [orient, pos] = validatetransform(val)

validateattributes(val, {'double', 'single'}, ...
    {'square', 'nrows', 4, 'ncols', 4, '2d', 'real'});

orient = val(1:3,1:3);
pos = val(1:3,4).';
end
