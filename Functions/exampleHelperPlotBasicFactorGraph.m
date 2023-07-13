function exampleHelperPlotBasicFactorGraph
%EXAMPLEHELPERPLOTBASICFACTORGRAPH Visualize basic factor graph
%   
%   This function is for internal use only. It may be removed in the
%   future.

%   Copyright 2021-2022 The MathWorks, Inc.

% Create graph with factor and state nodes.
g = graph([0 1 0 0 0 0 0 0; ...
    1 0 1 0 0 0 0 0; ...
    0 1 0 1 0 0 0 0; ...
    0 0 1 0 1 1 0 0; ...
    0 0 0 1 0 0 0 0; ...
    0 0 0 1 0 0 1 0; ...
    0 0 0 0 0 1 0 1; ...
    0 0 0 0 0 0 1 0], ...
    ["GPS_{t0}", "Pose_{t0}", "IMU_{t0-t1}", "Pose_{t1}", "GPS_{t1}", ...
    "IMU_{t1-t2}", "Pose_{t2}", "GPS_{t2}"]);
% For each node, specify whether it is a factor or a state node.
isFactor = [true; false; true; false; true; true; false; true];

% Specify maker and color for factor and state nodes.
factorShape = "square";
stateShape = "o";
colorOrd = colororder;
factorColor = colorOrd(1,:);
stateColor = colorOrd(2,:);

% Set the marker shapes and colors depending on whether they are factor or
% state nodes.
markers = strings(numel(isFactor), 1);
markers(isFactor) = factorShape;
markers(~isFactor) = stateShape;
colors = NaN(numel(isFactor), 3);
colors(isFactor,:) = factorColor .* ones(nnz(isFactor), 3);
colors(~isFactor,:) = stateColor .* ones(nnz(~isFactor), 3);

% Plot graph with factors on top of state nodes.
h = plot(g, XData=[0 0 1 2 2 3 4 4], YData=0.5*[1 0 1 0 1 1 0 1]);
h.Marker = markers;
h.MarkerSize = 7;
h.NodeColor = colors;

% Improve visuals by centering the graph and removing the axis ticks. 
axis equal
xlim(xlim + 0.1)
xticks([])
yticks([])
end