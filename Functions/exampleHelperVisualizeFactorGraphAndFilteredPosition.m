classdef exampleHelperVisualizeFactorGraphAndFilteredPosition < handle
%EXAMPLEHELPERVISUALIZEFACTORGRAPHANDFILTEREDPOSITION Plot factor graph and
%   logged filtered position estimates.
%   
%   This class is for internal use only. It may be removed in the future.

%   Copyright 2021-2022 The MathWorks, Inc.

    properties (Access = private)
        GeoPlotLines
    end

    methods
        function obj = exampleHelperVisualizeFactorGraphAndFilteredPosition
            geoPlotLines = geoplot(NaN, NaN, NaN, NaN, LineWidth=1.5);
            geobasemap satellite
            legend("Factor graph", "Ground Turth")
            obj.GeoPlotLines = geoPlotLines;
        end

        function FGO_LLH = updatePlot(obj, ii, G, poseIDs, posLLH, lla0, numIMUPerGPS)
            geoPlotLines = obj.GeoPlotLines;
            P = zeros(ii,7);
            for jj = 1:ii
                P(jj,:) = nodeState(G, poseIDs(jj));
            end
            P = P(2:end,:);
            estPos = P(:,1:3);
            estPosLLA = enu2lla(estPos, lla0, "ellipsoid");
            FGO_LLH(:,1:3) = estPosLLA;
%             colors = colororder;
%             set(geoPlotLines(1), LatitudeData=estPosLLA(:,1), ...
%                 LongitudeData=estPosLLA(:,2), Color=colors(3,:));
%             imuSamplesPerGPS = numIMUPerGPS(1:ii);
%             filteredPosIdx = 1:sum(imuSamplesPerGPS(:));
%             set(geoPlotLines(2), LatitudeData=posLLA(filteredPosIdx,1), ...
%                 LongitudeData=posLLA(filteredPosIdx,2), Color=colors(2,:));
            colors = colororder;
            set(geoPlotLines(1), LatitudeData=estPosLLA(:,1), ...
            LongitudeData=estPosLLA(:,2), Color=colors(3,:));
%             imuSamplesPerGPS = numIMUPerGPS(1:ii);
            filteredPosIdx = 1:ii;
            set(geoPlotLines(2), LatitudeData=posLLH(filteredPosIdx,1), ...
            LongitudeData=posLLH(filteredPosIdx,2), Color=colors(2,:));
                                            % GT trajetory

        end
    end

end