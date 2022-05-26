clear all;

unitXvec = [1; 0; 0];
unitYvec = [0; 1; 0];
unitZvec = [0; 0; 1];

AccelMagTumble;

meanAccelX = mean(AccelX); % -116.1178
meanAccelY = mean(AccelY); % -121.7046
meanAccelZ = mean(AccelZ); % 159.0758
meanMagX = mean(MagX); % -554.7605
meanMagY = mean(MagY); % 1.0618e+03
meanMagZ = mean(MagZ); % 297.4232

AccelScaledX = (AccelX ./ 16384) ./ 1000; % these are taken directly from Lab3
AccelScaledY = (AccelY ./ 16384) ./ 1000;
AccelScaledZ = (AccelZ ./ 16384) ./ 1000;

MagScaledX = ((MagX .* 0.15) ./ 47.507) .* 1000; % these are also taken directly from Lab3
MagScaledY = ((MagY .* 0.15) ./ 47.507) .* 1000;
MagScaledZ = ((MagZ .* 0.15) ./ 47.507) .* 1000;

AccelData = [AccelScaledX, AccelScaledY, AccelScaledZ]; % put in nx3 matrix
MagData = [MagScaledX, MagScaledY, MagScaledZ];

SC_B_field = [22691.8, 5220.7, 41297.5]; % taken from "https://www.ngdc.noaa.gov/"

pi = [0,0,-1]; % setup Primary initial position
si = SC_B_field ./ norm(SC_B_field); % setup Secondary initial condition 

Pb = AccelData ./ norm(AccelData); % primary body, normalized
Sbmeas = MagData ./ norm(MagData); % secondary body, normalized

[Rmis, Pbody] = AlignPrimarySecondary(Pb', Sbmeas', pi', si', eye(3), 1)

% calc average of each axis and normalize that vector 
% pi = average of data / norm of average (mean(pi)/norm(mean(pi)))
% si = average of data / "..."