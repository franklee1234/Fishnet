% Define the plane size and number of points
n = 5;  % x Number
m = 5;  % y Number
numPoints = n * m;  % total Number

% the coordinates of the plane
x = repmat(linspace(0, 1, n), m, 1);
y = repmat(linspace(0, 1, m)', 1, n);
z = zeros(m, n);

% Select indices of points for periodic motion
% We can select the points within 5*5
movingPointIndices = [1, 5, 6, 9, 15, 19, 22];

% the time sequence
t = 0:0.1:5;

% save GIF
filename = 'periodic_check.gif';


figure (1)
for i = 1:length(t)
    % calculate the new position
    newX = x;
    newY = y;
    newZ = z;
    
    % Update the positions of points undergoing periodic motion
    for j = 1:length(movingPointIndices)
        idx = movingPointIndices(j);
        amplitude = 0.05; % we can change to any amplitude
        newX(idx) = x(idx) + amplitude * cos(2*pi*t(i));
        newY(idx) = y(idx) + amplitude * sin(2*pi*t(i));
    end
    
    % if distance is less than 0.5
    distances = zeros(numPoints, numPoints);  % save distances
    for j = 1:numPoints
        for k = j+1:numPoints
            distances(j, k) = sqrt((newX(j) - newX(k))^2 + (newY(j) - newY(k))^2 + (newZ(j) - newZ(k))^2);
            distances(k, j) = distances(j, k);
        end
    end    
    
    % show the distance between adjacent points
    disp('Distances between adjacent points:');
    disp(distances);
    

    % realtime position
    surf(newX, newY, newZ, 'FaceColor', 'interp', 'EdgeColor', 'none');
    hold on;
    scatter3(newX(:), newY(:), newZ(:), 50, 'filled','r');
    hold on;
    
    % draw the connecting line between adjacent two points and 
    % check if the distance is less than 0.5
    for j = 1:m
        for k = 1:n
            idx = (j-1)*n + k; % calculate the index of each point in the plane
            if k < n
                line([newX(j, k), newX(j, k+1)], [newY(j, k), newY(j, k+1)], [newZ(j, k), newZ(j, k+1)], 'Color', 'b');
                % check if the distance between two neighbors is less than 0.5
                distance = sqrt((newX(j, k) - newX(j, k+1))^2 + (newY(j, k) - newY(j, k+1))^2 + (newZ(j, k) - newZ(j, k+1))^2);
                if distance >= 0.5
                    disp(['Distance between points (' num2str(j) ',' num2str(k) ') and (' num2str(j) ',' num2str(k+1) ') is greater than or equal to 0.5']);
                end
            end
            if j < m
                line([newX(j, k), newX(j+1, k)], [newY(j, k), newY(j+1, k)], [newZ(j, k), newZ(j+1, k)], 'Color', 'b');
                % check if the distance between two neighbors is less than 0.5
                distance = sqrt((newX(j, k) - newX(j+1, k))^2 + (newY(j, k) - newY(j+1, k))^2 + (newZ(j, k) - newZ(j+1, k))^2);
                if distance >= 0.5
                    disp(['Distance between points (' num2str(j) ',' num2str(k) ') and (' num2str(j+1) ',' num2str(k) ') is greater than or equal to 0.5']);
                end
            end
        end
    end
    
    % Set the figure
    title(['t = ' num2str(t(i))]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    view(3);
    
    % save gif
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if i == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
    
    hold off;
    pause(0.1);
end
