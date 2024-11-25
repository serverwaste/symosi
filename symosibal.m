% Main.m
% Monte-Carlo 시뮬레이션으로 Time of Arrival (ToA) 알고리즘 기반 위치 정확도 계산

clear all;
format long e;

% 주요 파라미터
Anchor1Pos = [0 10];
Anchor2Pos = [0 0];
Anchor3Pos = [10 0];
Anchor4Pos = [10 10];
dt=0.1;
MaxNumforPositioningAccuracy = 1e3;
NoiseVar = [sqrt(0.01) sqrt(0.1) sqrt(1) sqrt(10) sqrt(100)];
%ToA
AC = zeros(5, 1);
%LPF
alpha = [9.900000000000000e-01 9.399999999999999e-01 7.700000000000000e-01 4.700000000000000e-01 1.400000000000000e-01]; 
AC_LPF = zeros(5, 1);
estimatedPos_LPF =zeros(5,1);
%predition_KF
A = [1 0; 0 1];
W = [0.000254480252705510 0.000740363780069335 -0.0111507988397592 0.0289433355873671 -0.0402898777163043;
    -6.09103567779639e-05 0.00177544753616439 0.00395764994184721 -0.0221450907603877 -0.0931219597185202];
Error_Predition = zeros(5,MaxNumforPositioningAccuracy);
% AC_Predition = zeros(5,1);
for numforNoiseVar = 1:5
    Error = 0;
    Error_LPF = 0;
    alpha_Var = alpha(numforNoiseVar);
        for numforPositioningAccuracy = 1:MaxNumforPositioningAccuracy
            estimatedPos_LPF = [0 0]; % LPF 위치 초기화
            for i = 1:11
                exactPosX = i-1;
                exactPosY = i-1;
                exactPos = [exactPosX exactPosY];
                if i <= 1
                rangingfromAnchor1 = norm(Anchor1Pos - exactPos) + NoiseVar(numforNoiseVar) * randn;
                rangingfromAnchor2 = norm(Anchor2Pos - exactPos) + NoiseVar(numforNoiseVar) * randn;
                rangingfromAnchor3 = norm(Anchor3Pos - exactPos) + NoiseVar(numforNoiseVar) * randn;
                rangingfromAnchor4 = norm(Anchor4Pos - exactPos) + NoiseVar(numforNoiseVar) * randn;
                [estimatedPosX, estimatedPosY] = ToAalg(rangingfromAnchor1, rangingfromAnchor2, rangingfromAnchor3, rangingfromAnchor4);
                prevPos = [estimatedPosX;estimatedPosY];
                Error_Predition(numforNoiseVar,numforPositioningAccuracy,i) = norm(exactPos - [estimatedPosX estimatedPosY]);
                elseif i == 2
                
                rangingfromAnchor1 = norm(Anchor1Pos - exactPos) + NoiseVar(numforNoiseVar) * randn;
                rangingfromAnchor2 = norm(Anchor2Pos - exactPos) + NoiseVar(numforNoiseVar) * randn;
                rangingfromAnchor3 = norm(Anchor3Pos - exactPos) + NoiseVar(numforNoiseVar) * randn;
                rangingfromAnchor4 = norm(Anchor4Pos - exactPos) + NoiseVar(numforNoiseVar) * randn;
                [estimatedPosX, estimatedPosY] = ToAalg(rangingfromAnchor1, rangingfromAnchor2, rangingfromAnchor3, rangingfromAnchor4);
                Error_Predition(numforNoiseVar,numforPositioningAccuracy,i) = norm(exactPos - [estimatedPosX estimatedPosY]);
                %속도추정
                VX = ([estimatedPosX;estimatedPosY] - prevPos)/dt;
                predition_KF = A*[estimatedPosX;estimatedPosY] + VX*dt + [W(1,numforNoiseVar); W(2,numforNoiseVar)];
                prevPos = [estimatedPosX;estimatedPosY];   
                else
                rangingfromAnchor1 = norm(Anchor1Pos - exactPos) + NoiseVar(numforNoiseVar) * randn;
                rangingfromAnchor2 = norm(Anchor2Pos - exactPos) + NoiseVar(numforNoiseVar) * randn;
                rangingfromAnchor3 = norm(Anchor3Pos - exactPos) + NoiseVar(numforNoiseVar) * randn;
                rangingfromAnchor4 = norm(Anchor4Pos - exactPos) + NoiseVar(numforNoiseVar) * randn;
                [estimatedPosX, estimatedPosY] = ToAalg(rangingfromAnchor1, rangingfromAnchor2, rangingfromAnchor3, rangingfromAnchor4);
                prevPos = [estimatedPosX;estimatedPosY];
                predition_KF = A*predition_KF + VX*dt + [W(1,numforNoiseVar); W(2,numforNoiseVar)];
                VX = (predition_KF-prevPos)/dt;
                Error_Predition(numforNoiseVar,numforPositioningAccuracy,i) = norm(exactPos - predition_KF);
                end
                % ToAalg
                [estimatedPosX, estimatedPosY] = ToAalg(rangingfromAnchor1, rangingfromAnchor2, rangingfromAnchor3, rangingfromAnchor4);
                Position_Error_ToA = norm([exactPosX exactPosY] - [estimatedPosX estimatedPosY]); % 위치 오차
                % LPF
                estimatedPos_LPF = (1 - alpha_Var) * estimatedPos_LPF + alpha_Var * [estimatedPosX, estimatedPosY];
                Position_Error_LPF = norm([exactPosX exactPosY] - estimatedPos_LPF); % LPF 위치 오차

                Error = Error + Position_Error_ToA; % 누적 ToAalg 오차
                Error_LPF = Error_LPF + Position_Error_LPF; % 누적 LPF 오차
                
            end
        end
    AC(numforNoiseVar) = Error / (MaxNumforPositioningAccuracy * 11); % ToAalg 정확도
    AC_LPF(numforNoiseVar) = Error_LPF / (MaxNumforPositioningAccuracy*11); % LPF 정확도
    
end
AC_Predition =  mean(Error_Predition,[2,3]);

semilogx([0.01 0.1 1 10 100], AC, '-o', 'DisplayName', 'ToAalg');
hold on;
semilogx([0.01 0.1 1 10 100], AC_LPF, '-s', 'DisplayName', 'LPF');
semilogx([0.01 0.1 1 10 100], AC_Predition, '-s', 'DisplayName', 'Predition');
xlabel('Noise Var');
ylabel('Accuracy');
legend show;
grid on;
