% Initialize webcam
c = webcam(2);
converter=0;

% Define the figure
figure;

% Variable to control grid spacing
gridSpacing = 200; % Adjust this value to change the grid size

% Initialize variables for X and Y coordinates
xCoord = 0;
yCoord = 0;

while true
    % Capture image from webcam
    img = snapshot(c);
    
    % Get image dimensions
    [imgHeight, imgWidth, ~] = size(img);
    
    % Convert RGB image to HSV color space
    hsvImg = rgb2hsv(img);
    
    % Define thresholds for blue color in HSV space
    blueMin = [0.55, 0.4, 0.2]; % Adjust these thresholds as needed
    blueMax = [0.75, 1, 1];
    
    % Define wider thresholds for yellow color in HSV space to increase sensitivity
    yellowMin = [0.1, 0.3, 0.3]; % Lowered saturation and value minimums
    yellowMax = [0.2, 1, 1];      % Same upper limits

    % Create binary masks for blue and yellow regions
    blueMask = (hsvImg(:,:,1) >= blueMin(1)) & (hsvImg(:,:,1) <= blueMax(1)) & ...
               (hsvImg(:,:,2) >= blueMin(2)) & (hsvImg(:,:,2) <= blueMax(2)) & ...
               (hsvImg(:,:,3) >= blueMin(3)) & (hsvImg(:,:,3) <= blueMax(3));

    yellowMask = (hsvImg(:,:,1) >= yellowMin(1)) & (hsvImg(:,:,1) <= yellowMax(1)) & ...
                 (hsvImg(:,:,2) >= yellowMin(2)) & (hsvImg(:,:,2) <= yellowMax(2)) & ...
                 (hsvImg(:,:,3) >= yellowMin(3)) & (hsvImg(:,:,3) <= yellowMax(3));
    
    % Perform morphological operations to remove noise
    blueMask = imopen(blueMask, strel('disk', 5));
    blueMask = imclose(blueMask, strel('disk', 5));
    
    yellowMask = imopen(yellowMask, strel('disk', 5));
    yellowMask = imclose(yellowMask, strel('disk', 5));
    
    % Find connected components in the binary masks
    blueStats = regionprops(blueMask, 'BoundingBox', 'Centroid');
    yellowStats = regionprops(yellowMask, 'BoundingBox', 'Centroid');
    
    % Display the original image
    imshow(img);
    hold on;
    
    % Loop through detected blue objects
    for i = 1:length(blueStats)
        % Get the bounding box and centroid
        bb = blueStats(i).BoundingBox;
        centroid = blueStats(i).Centroid;
        
        % Draw a rectangle around the detected blue object
        rectangle('Position', bb, 'EdgeColor', 'r', 'LineWidth', 2);
        
        % Plot the centroid
        plot(centroid(1), centroid(2), 'g+', 'MarkerSize', 10);
        
        % Draw axes centered at the centroid
        line([centroid(1), imgWidth], [centroid(2), centroid(2)], 'Color', 'g', 'LineWidth', 2); % Positive X-axis
        line([centroid(1), 0], [centroid(2), centroid(2)], 'Color', 'g', 'LineWidth', 2); % Negative X-axis
        line([centroid(1), centroid(1)], [centroid(2), imgHeight], 'Color', 'g', 'LineWidth', 2); % Positive Y-axis
        line([centroid(1), centroid(1)], [centroid(2), 0], 'Color', 'g', 'LineWidth', 2); % Negative Y-axis
        
        % Draw grid lines and labels along positive and negative X and Y axes
        for xPos = centroid(1):-gridSpacing:0
            line([xPos, xPos], [0, imgHeight], 'Color', 'g', 'LineStyle', '--');
            text(xPos, centroid(2) - 10, num2str(round(xPos - centroid(1))), 'Color', 'g', 'FontSize', 8, 'HorizontalAlignment', 'center');
        end
        for xPos = centroid(1):gridSpacing:imgWidth
            line([xPos, xPos], [0, imgHeight], 'Color', 'g', 'LineStyle', '--');
            text(xPos, centroid(2) - 10, num2str(round(xPos - centroid(1))), 'Color', 'g', 'FontSize', 8, 'HorizontalAlignment', 'center');
        end
        for yPos = centroid(2):-gridSpacing:0
            line([0, imgWidth], [yPos, yPos], 'Color', 'g', 'LineStyle', '--');
            text(centroid(1) - 10, yPos, num2str(round(yPos - centroid(2))), 'Color', 'g', 'FontSize', 8, 'HorizontalAlignment', 'right');
        end
        for yPos = centroid(2):gridSpacing:imgHeight
            line([0, imgWidth], [yPos, yPos], 'Color', 'g', 'LineStyle', '--');
            text(centroid(1) - 10, yPos, num2str(round(yPos - centroid(2))), 'Color', 'g', 'FontSize', 8, 'HorizontalAlignment', 'right');
        end
    end
    
    % Detect yellow rectangles and draw lines between them
    if length(yellowStats) >= 2
        centroids = cat(1, yellowStats.Centroid);
        % Draw rectangles and centroids
        for i = 1:length(yellowStats)
            bb = yellowStats(i).BoundingBox;
            centroid = yellowStats(i).Centroid;
            rectangle('Position', bb, 'EdgeColor', 'b', 'LineWidth', 2);
            plot(centroid(1), centroid(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        end
        % Draw line between the centroids of two yellow objects
        line([centroids(1,1) centroids(2,1)], [centroids(1,2) centroids(2,2)], 'Color', 'b', 'LineWidth', 2);
        % Calculate and plot the midpoint
        midPoint = mean(centroids, 1);
        plot(midPoint(1), midPoint(2), 'mo', 'MarkerSize', 10, 'LineWidth', 2);

        % Update X and Y coordinates in real-time
        if ~isempty(blueStats)
            blueCentroid = blueStats(1).Centroid; % Assuming there is at least one blue object
            converter=pix2m(blueCentroid(1));
            xCoord = abs(midPoint(1) - blueCentroid(1))*converter;
            yCoord = abs(midPoint(2) - blueCentroid(2))*converter;
            
            % Display the X and Y coordinates
            text((midPoint(1) + blueCentroid(1))/2, (midPoint(2) + blueCentroid(2))/2, ...
                sprintf('X: %.2f, Y: %.2f', xCoord, yCoord), 'Color', 'w', 'FontSize', 12, 'BackgroundColor', 'black');
        end
    end
    fprintf("%f, %f\n",xCoord,yCoord);
    hold off;
    pause(0.1); % Adjust to control the refresh rate
end

function m=pix2m (large)
    m=0.35/large;
end
