% LPF.m
% Low Pass Filter (LPF) 함수

function estimatedPos_LPF = LPF(estimatedPos_LPF, alpha_Var, newPos)
    % LPF 위치 업데이트
    estimatedPos_LPF = (1 - alpha_Var) * estimatedPos_LPF + alpha_Var * newPos;
end
