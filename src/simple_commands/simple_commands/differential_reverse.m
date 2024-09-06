function [thetae,d] = differential_reverse(thetae,d)
%DIFFERENTIAL_REVERSE Summary of this function goes here
%   Detailed explanation goes here
    if abs(thetae) > pi/2
        thetae = thetae - sign(thetae)* pi;
        d = -d;
    end
end

