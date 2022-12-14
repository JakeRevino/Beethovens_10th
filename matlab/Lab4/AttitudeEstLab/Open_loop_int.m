clc
clear all
close all

% create trajectory data 
[A, M, G, E] = CreateTrajectoryData(0.02 , true);

% convert euler angles to radians
E = deg2rad(E);

% gyro input rates set at 60 degrees
p=deg2rad(60);
q=deg2rad(60);
r=deg2rad(60);
gyroInput=[p;q;r];

% average bias 
gbx = mean(deg2rad(G(1:50, 1)));
gby = mean(deg2rad(G(1:50, 2)));
gbz = mean(deg2rad(G(1:50, 3)));

% gyro input with bias


% dcm matrix created from euler angles
% C = eul2dcm(E);
C = [1 0 0; 0 1 0; 0 0 1];
E1 = dcm2eul(C);

% integrate with forward integration and matrix exponential
for i = 1:20,
    gyroInputWithBias=gyroInput+[gbx;gby;gbz];
    Int1 = IntegrateOpenLoop(C, gyroInputWithBias, 0.02, 0);
    Int2 = IntegrateOpenLoop(C, gyroInputWithBias, 0.02, 1);
end


% obtain euler angles from dcm matrices
E2 = dcm2eul(Int1);
E3 = dcm2eul(Int2);

% norms of the data using both methods
Int1_1 = norm(Int1(:, 1)) % yaw norm with forward integration
Int1_2 = norm(Int1(:, 2)) % pitch norm with forward integration
Int1_3 = norm(Int1(:, 3)) % roll norm with forward integration
Int2_1 = norm(Int2(:, 1)) % yaw norm with exponential
Int2_2 = norm(Int2(:, 2)) % pitch norm with exponential
Int2_3 = norm(Int2(:, 3)) % roll norm with exponential

% obtain euler angles
E1 = rad2deg(E1)
E2 = rad2deg(E2)
E3 = rad2deg(E3)

m1 = mean(sqrt((E2-E1).^2)) % mean using forward integration
m2 = mean(sqrt((E3-E1).^2)) % mean using matrix exponential

std1 = std(sqrt((E2-E1).^2)) % std using forward integration
std2 = std(sqrt((E3-E1).^2)) % std using matrix exponential

% draw histograms to compare
histfit(sqrt((E2-E1).^2))
figure()
histfit(sqrt((E3-E1).^2))


function C=eul2dcm(eul)
%----------------------------------------------------------------
% function C=eul2dcm(eul)
%
%   This functions determines the direction cosine matrix C
%   that transforms a vector in a reference axis system at time k
%   to one the same axis sytem at time k+1.  The input argument to
%   this function is a vector of the Euler angles in the following
%   order: eul = [yaw,pitch,roll]. (i.e., 3-2-1 rotation convention).  
%
%-----------------------------------------------------------------  

ps=eul(1); th=eul(2); ph=eul(3);

C1=[1 0 0; 0 cos(ph) sin(ph); 0 -sin(ph) cos(ph)];
C2=[cos(th) 0 -sin(th); 0 1 0; sin(th) 0 cos(th)];
C3=[cos(ps) sin(ps) 0; -sin(ps) cos(ps) 0; 0 0 1];

C=C1*C2*C3;
end

function C=dcm2eul(eul)

psi = atan2(eul(1,2), eul(1,1));
phi = atan2(eul(2,3), eul(3,3));
theta = -asin(eul(1,3));

C=[psi; theta; phi];
end

