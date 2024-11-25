% computeRanges.m
% 주어진 앵커와 정확한 위치에 대한 거리 측정 계산 함수

function [rangingfromAnchor1, rangingfromAnchor2, rangingfromAnchor3, rangingfromAnchor4] = ...
    computeRanges(Anchor1Pos, Anchor2Pos, Anchor3Pos, Anchor4Pos, exactPos, noiseVar)

    % 각 앵커로부터의 거리 계산
    rangingfromAnchor1 = norm(Anchor1Pos - exactPos) + noiseVar * randn;
    rangingfromAnchor2 = norm(Anchor2Pos - exactPos) + noiseVar * randn;
    rangingfromAnchor3 = norm(Anchor3Pos - exactPos) + noiseVar * randn;
    rangingfromAnchor4 = norm(Anchor4Pos - exactPos) + noiseVar * randn;
end
