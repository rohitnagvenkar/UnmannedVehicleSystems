% obstacles : Matrix 3Xn, n is the number of obstacles
% obstacles(1,:): x-position of obstacles [m]
% obstacles(2,:): y-position of obstacles [m]
% obstacles(3,:): radious of obstacles [m]

%{
obstaclesArray = [2.5     7.5;
                   0    0.2;
                   0.2  0.2];

%}

obstaclesArray = [4      2    2;
                  2     -2    2;
                  0.2   0.2   0.5];
              

% Sonar sensor specifications
sonarInfo.as = 0.6; % coverage width [m]
sonarInfo.bs = 7.5; % coverage length [m]
sonarInfo.hs = 0.2; % sensor location on the platform (in body frame) 
sonar.Ts = 1/10; % sonar sensor sampling period
