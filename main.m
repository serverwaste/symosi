% Main.m
% Monte-Carlo 시뮬레이션으로 Time of Arrival (ToA) 알고리즘 기반 위치 정확도 계산

clear all;
format long e;

% 주요 파라미터
Anchor1Pos = [0 10];
Anchor2Pos = [0 0];
Anchor3Pos = [10 0];
Anchor4Pos = [10 10];
dt = 0.1;
MaxNumforPositioningAccuracy = 1e3;
NoiseVar = [sqrt(0.01) sqrt(0.1) sqrt(1) sqrt(10) sqrt(100)];
alpha = [9.900000000000000e-01 9.399999999999999e-01 7.700000000000000e-01 4.700000000000000e-01 1.400000000000000e-01]; 

AC = zeros(5, 1);
AC_LPF = zeros(5, 1);
Error_Predition = zeros(5, MaxNumforPositioningAccuracy);

for numforNoiseVar = 1:5
    Error = 0;
    Error_LPF = 0;
    alpha_Var = alpha(numforNoiseVar);

    for numforPositioningAccuracy = 1:MaxNumforPositioningAccuracy
        estimatedPos_LPF = [0 0]; % LPF 위치 초기화

        for i = 1:11
            exactPos = [i-1 i-1];

            % 거리 측정
            [rangingfromAnchor1, rangingfromAnchor2, rangingfromAnchor3, rangingfromAnchor4] = ...
                computeRanges(Anchor1Pos, Anchor2Pos, Anchor3Pos, Anchor4Pos, exactPos, NoiseVar(numforNoiseVar));

            % ToA 알고리즘 실행
            [estimatedPosX, estimatedPosY] = ToAalg(rangingfromAnchor1, rangingfromAnchor2, rangingfromAnchor3, rangingfromAnchor4);

            % 위치 오차 계산
            Position_Error_ToA = norm(exactPos - [estimatedPosX, estimatedPosY]);
            estimatedPos_LPF = LPF(estimatedPos_LPF, alpha_Var, [estimatedPosX, estimatedPosY]);
            Position_Error_LPF = norm(exactPos - estimatedPos_LPF);

            % 에러 누적
            Error = Error + Position_Error_ToA;
            Error_LPF = Error_LPF + Position_Error_LPF;
            randn;
        end
    end

    % 정확도 계산
    AC(numforNoiseVar) = Error / (MaxNumforPositioningAccuracy * 11);
    AC_LPF(numforNoiseVar) = Error_LPF / (MaxNumforPositioningAccuracy * 11);
end

% 예측 오차 계산
AC_Predition = mean(Error_Predition, [2,3]);

% 결과 플로팅
semilogx([0.01 0.1 1 10 100], AC, '-o', 'DisplayName', 'ToAalg');
hold on;
semilogx([0.01 0.1 1 10 100], AC_LPF, '-s', 'DisplayName', 'LPF');
semilogx([0.01 0.1 1 10 100], AC_Predition, '-s', 'DisplayName', 'Predition');
xlabel('Noise Var');
ylabel('Accuracy');
legend show;
grid on;
