% Example usage coeffs [x^2, x^1, const]:
% coeffs = [2 1 -15]; 
% vehiclex = 5;
% vehicley = -1;
% w = 2;  % Vehicle width
% l = 4;  % Vehicle length
% dist = findDistanceToLane(coeffs, vehiclex, vehicley, w, l);
% disp(['The distance to the lane is: ', num2str(dist)]);
function dist = distanceFinding(coeffs, vehiclex, vehicley, w, l)
    % Define xmin and xmax based on the length of the car
    xmin = vehiclex - l / 2;  % Starting point based on vehicle length
    xmax = vehiclex + l / 2;  % End point based on vehicle length
    
    % Define the desired x range
    x_min = xmin;
    x_max = xmax;
    
    % Generate a set of x values within the range
    x_values = linspace(x_min, x_max, 100);  % 100 points between xmin and xmax
    
    % Calculate corresponding y values for each x using the polynomial equation
    y_values = polyval(coeffs, x_values);
    
    % Create the matrix P of points on the polynomial
    P = [x_values' y_values'];  % Each row is a point [x, y]
    
    % Adjust the query point to the front driver side corner of BB
    PQ = [vehiclex - l/2, vehicley - w / 2];
    
    % Find the closest point in P to PQ using dsearchn
    k = dsearchn(P, PQ);  % Index of the closest point in P
    
    % Retrieve the closest point's coordinates
    closest_point = P(k, :);
    
    % Compute the Euclidean distance between the query point and the closest point
    dist = sqrt((closest_point(1) - PQ(1))^2 + (closest_point(2) - PQ(2))^2);
    
    % % Optionally, plot the polynomial points, the closest point, and the vehicle position
    % figure(158995488);
    % plot(x_values, y_values, 'o'); hold on;  % Plot polynomial points
    % plot(closest_point(1), closest_point(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);  % Plot closest point
    % plot(PQ(1), PQ(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);  % Plot query point (vehicle position adjusted for width)
    % legend('Polynomial Points', 'Closest Point', 'Front Driver Side Corner of Vehicle');
    % xlabel('x');
    % ylabel('y');
    % title('Closest Point on Polynomial');
    % hold off
end