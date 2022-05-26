%% Plot the attitude of simulated noisy data calibrated through 
% closed loop integration against true attitude. Also plot bias error as it
% converges to true bias

%% Part 5 initialization

clear all;
close all;

%Get noisy simulated trajectory data
[Acc,Mag,wGyro,Eul] = CreateTrajectoryData(.02,1);
numSteps = 3330;
summed_wgyro= [0 0 0].';

%convert wGyro to rads
wGyro = deg2rad(wGyro);

for i = 1:50
    summed_wgyro = summed_wgyro + wGyro(i, :).';  %true bias
end

%Find the true bias by taking the average value of the 1st second of data
biasTerms = summed_wgyro./50;

%NED vectors
nvector=[1;0;0];
evector=[0;-1;0];
dvector=[0;0;-1];

%inertial reference frames for accelerometer and magnetometer
accelInertial=[0;0;1];
magInertial= [22770;5329;41510.2]/1000;

%% Part 5 proportional accelerometer

% PI values for accelerometer and magnetometer components

%accelerometer PI values
Kp_a=70;
Ki_a=0;

%magnetometer PI values
Kp_m=0;
Ki_m=0;

%Use the identity matrix as initial
R=eye(3);

%Gyro bias
biasEstimate(:,1)=biasTerms;

%Integrate gyroscope angles using feedback from accelerometer
time(1)=0;
for i=1:numSteps,
    gyroInputWithBias=wGyro(i ,:).'-biasTerms;
    [Ro,biasEstimate(:,i+1)] = IntegrateClosedLoop(R, ...
        biasEstimate(:,i),gyroInputWithBias, ...
        Mag(i,:).',Acc(i,:).',magInertial,accelInertial, Kp_a, Ki_a, Kp_m, ...
        Ki_m, .02);
    R = Ro;

    %convert DCM of noisy data to euler angles
    theta_noisy(i) = -asind(R(1,3));
    phi_noisy(i) = atan2d(R(2,3),R(3,3));
    psi_noisy(i) = atan2d(R(1,2),R(1,1));
    time(i+1)=time(i)+.02;
end

%ideal euler angles
theta_ideal = Eul(:,2);
phi_ideal = Eul(:,3);
psi_ideal = Eul(:,1);

%Plot noisy euler angles vs ideal
figure()
subplot(3,1,1)
plot(time(1:numSteps), theta_noisy )
hold on;
plot(time(1:numSteps), theta_ideal)
legend("noisy", "ideal")
ylabel("Pitch")

subplot(3,1,2)
plot(time(1:numSteps), phi_noisy )
hold on;
plot(time(1:numSteps), phi_ideal  )
legend("noisy", "ideal")
ylabel("Roll")


subplot(3,1,3)
plot(time(1:numSteps), psi_noisy )
hold on;
plot(time(1:numSteps), psi_ideal )
legend("noisy", "ideal")
ylabel("Yaw")
sgtitle("Noisy vs. Ideal Attitude Proportional Accelerometer")

%Plot bias error
figure()
subplot(3,1,3)
yaw_bias = biasEstimate(1,:);
plot(time(1:numSteps), yaw_bias(1:numSteps) - biasTerms(1))
ylabel("Yaw")


subplot(3,1,1)
pitch_bias = biasEstimate(2,:);
plot(time(1:numSteps), pitch_bias(1:numSteps) - biasTerms(2))
ylabel("Pitch")

subplot(3,1,2)
roll_bias = biasEstimate(3,:);
plot(time(1:numSteps), roll_bias(1:numSteps) - biasTerms(3))
ylabel("Roll")
sgtitle("Bias Error Proportional Accelerometer")

%% Part 5 proportional integral accelerometer
close all;

% PI values for accelerometer and magnetometer components

%accelerometer PI values
Kp_a=70;
Ki_a=7;

%magnetometer PI values
Kp_m=0;
Ki_m=0;

%Use the identity matrix as initial
R=eye(3);

%Gyro bias
biasEstimate(:,1)=biasTerms;

%Integrate gyroscope angles using feedback from accelerometer
time(1)=0;
for i=1:numSteps,
    gyroInputWithBias=wGyro(i ,:).'-biasTerms;
    [Ro,biasEstimate(:,i+1)] = IntegrateClosedLoop(R, ...
        biasEstimate(:,i),gyroInputWithBias, ...
        Mag(i,:).',Acc(i,:).',magInertial,accelInertial, Kp_a, Ki_a, Kp_m, ...
        Ki_m, .02);
    R = Ro;

    %convert DCM of noisy data to euler angles
    theta_noisy(i) = -asind(R(1,3));
    phi_noisy(i) = atan2d(R(2,3),R(3,3));
    psi_noisy(i) = atan2d(R(1,2),R(1,1));
    time(i+1)=time(i)+.02;
end

%ideal euler angles
theta_ideal = Eul(:,2);
phi_ideal = Eul(:,3);
psi_ideal = Eul(:,1);

%Plot noisy euler angles vs ideal
figure()
subplot(3,1,1)
plot(time(1:numSteps), theta_noisy )
hold on;
plot(time(1:numSteps), theta_ideal)
legend("noisy", "ideal")
ylabel("Pitch")

subplot(3,1,2)
plot(time(1:numSteps), phi_noisy )
hold on;
plot(time(1:numSteps), phi_ideal  )
legend("noisy", "ideal")
ylabel("Roll")


subplot(3,1,3)
plot(time(1:numSteps), psi_noisy )
hold on;
plot(time(1:numSteps), psi_ideal )
legend("noisy", "ideal")
ylabel("Yaw")
sgtitle("Noisy vs. Ideal Attitude PI Accelerometer")

%Plot bias error
figure()
subplot(3,1,3)
yaw_bias = biasEstimate(1,:);
plot(time(1:numSteps), yaw_bias(1:numSteps) - biasTerms(1))
ylabel("Yaw")


subplot(3,1,1)
pitch_bias = biasEstimate(2,:);
plot(time(1:numSteps), pitch_bias(1:numSteps) - biasTerms(2))
ylabel("Pitch")

subplot(3,1,2)
roll_bias = biasEstimate(3,:);
plot(time(1:numSteps), roll_bias(1:numSteps) - biasTerms(3))
ylabel("Roll")
sgtitle("Bias Error PI Accelerometer")

%% Part 5 proportional magnetometer
% PI values for accelerometer and magnetometer components

%accelerometer PI values
Kp_a=0;
Ki_a=0;

%magnetometer PI values
Kp_m=35;
Ki_m=0;

%Use the identity matrix as initial
R=eye(3);

%Gyro bias
biasEstimate(:,1)=biasTerms;

%Integrate gyroscope angles using feedback from magnetometer
time(1)=0;
for i=1:numSteps,
    gyroInputWithBias=wGyro(i ,:).'-biasTerms;
    [Ro,biasEstimate(:,i+1)] = IntegrateClosedLoop(R, ...
        biasEstimate(:,i),gyroInputWithBias, ...
        Mag(i,:).',Acc(i,:).',magInertial,accelInertial, Kp_a, Ki_a, Kp_m, ...
        Ki_m, .02);
    R = Ro;

    %convert DCM of noisy data to euler angles
    theta_noisy(i) = -asind(R(1,3));
    phi_noisy(i) = atan2d(R(2,3),R(3,3));
    psi_noisy(i) = atan2d(R(1,2),R(1,1));
    time(i+1)=time(i)+.02;
end

%ideal euler angles
theta_ideal = Eul(:,2);
phi_ideal = Eul(:,3);
psi_ideal = Eul(:,1);

%Plot noisy euler angles vs ideal
figure()
subplot(3,1,1)
plot(time(1:numSteps), theta_noisy )
hold on;
plot(time(1:numSteps), theta_ideal)
legend("noisy", "ideal")
ylabel("Pitch")

subplot(3,1,2)
plot(time(1:numSteps), phi_noisy )
hold on;
plot(time(1:numSteps), phi_ideal  )
legend("noisy", "ideal")
ylabel("Roll")


subplot(3,1,3)
plot(time(1:numSteps), psi_noisy )
hold on;
plot(time(1:numSteps), psi_ideal )
legend("noisy", "ideal")
ylabel("Yaw")
sgtitle("Noisy vs. Ideal Attitude Proportional Magnetometer")

%Plot bias error
figure()
subplot(3,1,3)
yaw_bias = biasEstimate(1,:);
plot(time(1:numSteps), yaw_bias(1:numSteps) - biasTerms(1))
ylabel("Yaw")


subplot(3,1,1)
pitch_bias = biasEstimate(2,:);
plot(time(1:numSteps), pitch_bias(1:numSteps) - biasTerms(2))
ylabel("Pitch")

subplot(3,1,2)
roll_bias = biasEstimate(3,:);
plot(time(1:numSteps), roll_bias(1:numSteps) - biasTerms(3))
ylabel("Roll")
sgtitle("Bias Error Proportional Magnetometer")

%% Part 5 PI magnetometer

% PI values for accelerometer and magnetometer components

%accelerometer PI values
Kp_a=0;
Ki_a=0;

%magnetometer PI values
Kp_m=35;
Ki_m=3.5;

%Use the identity matrix as initial
R=eye(3);

%Gyro bias
biasEstimate(:,1)=biasTerms;

%Integrate gyroscope angles using feedback from magnetometer
time(1)=0;
for i=1:numSteps,
    gyroInputWithBias=wGyro(i ,:).'-biasTerms;
    [Ro,biasEstimate(:,i+1)] = IntegrateClosedLoop(R, ...
        biasEstimate(:,i),gyroInputWithBias, ...
        Mag(i,:).',Acc(i,:).',magInertial,accelInertial, Kp_a, Ki_a, Kp_m, ...
        Ki_m, .02);
    R = Ro;

    %convert DCM of noisy data to euler angles
    theta_noisy(i) = -asind(R(1,3));
    phi_noisy(i) = atan2d(R(2,3),R(3,3));
    psi_noisy(i) = atan2d(R(1,2),R(1,1));
    time(i+1)=time(i)+.02;
end

%ideal euler angles
theta_ideal = Eul(:,2);
phi_ideal = Eul(:,3);
psi_ideal = Eul(:,1);

%Plot noisy euler angles vs ideal
figure()
subplot(3,1,1)
plot(time(1:numSteps), theta_noisy )
hold on;
plot(time(1:numSteps), theta_ideal)
legend("noisy", "ideal")
ylabel("Pitch")

subplot(3,1,2)
plot(time(1:numSteps), phi_noisy )
hold on;
plot(time(1:numSteps), phi_ideal  )
legend("noisy", "ideal")
ylabel("Roll")


subplot(3,1,3)
plot(time(1:numSteps), psi_noisy )
hold on;
plot(time(1:numSteps), psi_ideal )
legend("noisy", "ideal")
ylabel("Yaw")
sgtitle("Noisy vs. Ideal Attitude PI Magnetometer")

%Plot bias error
figure()
subplot(3,1,3)
yaw_bias = biasEstimate(1,:);
plot(time(1:numSteps), yaw_bias(1:numSteps) - biasTerms(1))
ylabel("Yaw")


subplot(3,1,1)
pitch_bias = biasEstimate(2,:);
plot(time(1:numSteps), pitch_bias(1:numSteps) - biasTerms(2))
ylabel("Pitch")

subplot(3,1,2)
roll_bias = biasEstimate(3,:);
plot(time(1:numSteps), roll_bias(1:numSteps) - biasTerms(3))
ylabel("Roll")
sgtitle("Bias Error PI Magnetometer")

%% Part 5 proportional magnetometer and accelerometer
% PI values for accelerometer and magnetometer components

%accelerometer PI values
Kp_a=35;
Ki_a=0;

%magnetometer PI values
Kp_m=35;
Ki_m=0;

%Use the identity matrix as initial
R=eye(3);

%Gyro bias
biasEstimate(:,1)=biasTerms;

%Integrate gyroscope angles using feedback from magnetometer and accelerometer
time(1)=0;
for i=1:numSteps,
    gyroInputWithBias=wGyro(i ,:).'-biasTerms;
    [Ro,biasEstimate(:,i+1)] = IntegrateClosedLoop(R, ...
        biasEstimate(:,i),gyroInputWithBias, ...
        Mag(i,:).',Acc(i,:).',magInertial,accelInertial, Kp_a, Ki_a, Kp_m, ...
        Ki_m, .02);
    R = Ro;

    %convert DCM of noisy data to euler angles
    theta_noisy(i) = -asind(R(1,3));
    phi_noisy(i) = atan2d(R(2,3),R(3,3));
    psi_noisy(i) = atan2d(R(1,2),R(1,1));
    time(i+1)=time(i)+.02;
end

%ideal euler angles
theta_ideal = Eul(:,2);
phi_ideal = Eul(:,3);
psi_ideal = Eul(:,1);

%Plot noisy euler angles vs ideal
figure()
subplot(3,1,1)
plot(time(1:numSteps), theta_noisy )
hold on;
plot(time(1:numSteps), theta_ideal)
legend("noisy", "ideal")
ylabel("Pitch")

subplot(3,1,2)
plot(time(1:numSteps), phi_noisy )
hold on;
plot(time(1:numSteps), phi_ideal  )
legend("noisy", "ideal")
ylabel("Roll")


subplot(3,1,3)
plot(time(1:numSteps), psi_noisy )
hold on;
plot(time(1:numSteps), psi_ideal )
legend("noisy", "ideal")
ylabel("Yaw")
sgtitle("Attitude-- Proportional Magnetometer and Accelerometer")

%Plot bias error
figure()
subplot(3,1,3)
yaw_bias = biasEstimate(1,:);
plot(time(1:numSteps), yaw_bias(1:numSteps) - biasTerms(1))
ylabel("Yaw")


subplot(3,1,1)
pitch_bias = biasEstimate(2,:);
plot(time(1:numSteps), pitch_bias(1:numSteps) - biasTerms(2))
ylabel("Pitch")

subplot(3,1,2)
roll_bias = biasEstimate(3,:);
plot(time(1:numSteps), roll_bias(1:numSteps) - biasTerms(3))
ylabel("Roll")
sgtitle("Bias Error Proportional Magnetometer and Accelerometer")


%% Part 5 PI magnetometer and accelerometer
% PI values for accelerometer and magnetometer components

%accelerometer PI values
Kp_a=35;
Ki_a=3.5;

%magnetometer PI values
Kp_m=35;
Ki_m=3.5;

%Use the identity matrix as initial
R=eye(3);

%Gyro bias
biasEstimate(:,1)=biasTerms;

%Integrate gyroscope angles using feedback from magnetometer and accelerometer
time(1)=0;
for i=1:numSteps,
    gyroInputWithBias=wGyro(i ,:).'-biasTerms;
    [Ro,biasEstimate(:,i+1)] = IntegrateClosedLoop(R, ...
        biasEstimate(:,i),gyroInputWithBias, ...
        Mag(i,:).',Acc(i,:).',magInertial,accelInertial, Kp_a, Ki_a, Kp_m, ...
        Ki_m, .02);
    R = Ro;

    %convert DCM of noisy data to euler angles
    theta_noisy(i) = -asind(R(1,3));
    phi_noisy(i) = atan2d(R(2,3),R(3,3));
    psi_noisy(i) = atan2d(R(1,2),R(1,1));
    time(i+1)=time(i)+.02;
end

%ideal euler angles
theta_ideal = Eul(:,2);
phi_ideal = Eul(:,3);
psi_ideal = Eul(:,1);

%Plot noisy euler angles vs ideal
figure()
subplot(3,1,1)
plot(time(1:numSteps), theta_noisy )
hold on;
plot(time(1:numSteps), theta_ideal)
legend("noisy", "ideal")
ylabel("Pitch")

subplot(3,1,2)
plot(time(1:numSteps), phi_noisy )
hold on;
plot(time(1:numSteps), phi_ideal  )
legend("noisy", "ideal")
ylabel("Roll")


subplot(3,1,3)
plot(time(1:numSteps), psi_noisy )
hold on;
plot(time(1:numSteps), psi_ideal )
legend("noisy", "ideal")
ylabel("Yaw")
sgtitle("Attitude-- PI Magnetometer and Accelerometer")

%Plot bias error
figure()
subplot(3,1,3)
yaw_bias = biasEstimate(1,:);
plot(time(1:numSteps), yaw_bias(1:numSteps) - biasTerms(1))
ylabel("Yaw")


subplot(3,1,1)
pitch_bias = biasEstimate(2,:);
plot(time(1:numSteps), pitch_bias(1:numSteps) - biasTerms(2))
ylabel("Pitch")

subplot(3,1,2)
roll_bias = biasEstimate(3,:);
plot(time(1:numSteps), roll_bias(1:numSteps) - biasTerms(3))
ylabel("Roll")
sgtitle("Bias Error PI Magnetometer and Accelerometer")

