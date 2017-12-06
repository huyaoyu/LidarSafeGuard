clear;
clc;
close all;

N = 360;

angleMin = 0;
angleMax = 2*pi;
angleInc = ( angleMax - angleMin ) / N;

r          = 1.0; % The radius of the cylinder.
ecc        = 0.9; % The eccentricity.
noiseRatio = 0.1;
noiseAmp   = r * noiseRatio; % The amplitute of the random noise.

angles = (1:1:N) / 360.0 * 2 * pi + angleMin;
angles = angles';

dArray01 = zeros(N,1);
dArray02 = dArray01;
x = dArray01;
y = x;

c = ecc^2 - r^2;

for I = 1:1:N
    alpha = angles(I);
    
    a = 1.0;
    b = -2.0 * ecc * cos( pi - alpha );
    
    dArray01(I) = ( -b + sqrt( b^2 - 4*a*c ) ) / ( 2*a );
    dArray02(I) = ( -b - sqrt( b^2 - 4*a*c ) ) / ( 2*a );

    x(I) = dArray01(I) * cos(alpha) + ecc;
    y(I) = dArray01(I) * sin(alpha);
end % I

figure('Name', 'Perfect ranges', 'NumberTitle', 'off');
plot(x, y);

hold on;

for I = 1:1:N
    plot( [ecc, x(I)], [0, y(I)] );
end % I

hold off;

title('Perfect ranges');

% Add some random noise into dArray.

randValues = rand(N, 1) - 0.5;
maxRandValue = max(randValues);
randValues = randValues / maxRandValue;

dArrayWithNoise = dArray01 + noiseAmp * randValues;

% Plot the curve with noise.

for I = 1:1:N
    alpha = angles(I);

    x(I) = dArrayWithNoise(I) * cos(alpha) + ecc;
    y(I) = dArrayWithNoise(I) * sin(alpha);
end % I

titleString = sprintf('Ranges with random noise of %.2f%% FS', noiseRatio*100);
figure('Name', titleString, 'NumberTitle', 'off');
plot(x, y);
title(titleString);

% Save the data into CSV files.
csvwrite('RangesPerfect.csv', [angles, dArray01]);

fnBase = 'RangesWithNoise';
fn = sprintf('%s_r%.2f_ecc%.2f_noiseAmp%.2f.csv', fnBase, r, ecc, noiseAmp);

csvwrite(fn, [angles, dArrayWithNoise]);
