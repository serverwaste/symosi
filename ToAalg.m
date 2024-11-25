function [estimatedPosX,estimatedPosY] = ToAalg(rangingfromAnchor1,rangingfromAnchor2,rangingfromAnchor3,rangingfromAnchor4)


H = [0, -20;
    20, -20;
    20, 0;
    20, 0;
    20, 20;
    0, 20];

D = [rangingfromAnchor1^2-rangingfromAnchor2^2-100;
    rangingfromAnchor1^2-rangingfromAnchor3^2;
    rangingfromAnchor1^2-rangingfromAnchor4^2+100;
    rangingfromAnchor2^2-rangingfromAnchor3^2+100;
    rangingfromAnchor2^2-rangingfromAnchor4^2+200;
    rangingfromAnchor3^2-rangingfromAnchor4^2+100];

H_T = transpose(H);
S = (H_T*H)^(-1)*H_T*D;
estimatedPosX = S(1);
estimatedPosY = S(2);


end
