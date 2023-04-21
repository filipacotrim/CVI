clear all; close all;

% Global variables
path = 'data\\View_001\\frame_%.4d.jpg';
frameIdComp = 4;
str = ['%s%.', num2str(frameIdComp), 'd.%s'];
seqLength = 794; %hardcoded (nr of files)
baseNum = 0;

% --------------------- 1: Plot Ground Truth -------------------------- %

fid = fopen('GT.txt', 'r'); 

frame = []; id = []; box_l = []; box_t = []; box_w = []; box_h = []; 
conf = []; x = [];y = []; z = [];
while ~feof(fid)
    y1 = fscanf(fid, '%f,'); 

    % GT data as a table with colums frame, id, box_l, box_t, box_w, box_h
    frame = cat(1,frame,y1(1,1));
    id = cat(1,id,y1(2,1));
    box_l = cat(1,box_l,y1(3,1));
    box_t = cat(1,box_t,y1(4,1));
    box_w = cat(1,box_w,y1(5,1));
    box_h = cat(1,box_h,y1(6,1));
end

GT_table = table(frame,id,box_l,box_t,box_w,box_h);
%GT_table
fclose(fid);

figure;

% for i = 0 : seqLength
%     imgfr = imread(sprintf(path,baseNum+i));
%     imshow(imgfr);
%     if i ~= 0 %frame 0 doesn't exist
% 
%         %finds all rows whose frame id is View id and saves it in a table
%         row = GT_table.frame == i;
%         frames = GT_table(row,:);
% 
%         %frames
%         for j = 0: height(frames)-1
% 
%             %for each row (each pedestrian) gets bounding box measures
%             box = frames(j+1,:);
%             b_l = box.box_l; b_t = box.box_t; b_w = box.box_w; b_h = box.box_h;
%             bounding_box = [b_l b_t b_w b_h]
%             rectangle('Position', bounding_box, 'EdgeColor',[1,1,0], ...
%                 'linewidth',2);
%             %j
%             
%         end
%     end
%     drawnow;
% end

% ----------------- 2.1: Background estimation ------------------------- %

step = 4; i = 1;
for k = 1 : step : seqLength
    imgfr = imread(sprintf(path,baseNum+i));

    % We create a 4D arrays because the image is 3D (L*C*RGB) and we have a stack
    % of images so we have (Lines,Collumn,Z,Time) where Z is the number of RGB channels          
    vid4D(:,:,:,i)=imgfr;
    % imshow(img); drawnow
    i = i+1;
end

% We use the median to supress the pedestrians
% Creates a 3D array of size (L, C, Z) where each one is the median pixel
% value all frames for that pixel location and color channel
imgbk = median(vid4D, 4);
figure, imshow(uint8(imgbk));

% ---------------- 2.2 Detector Algorithm + Bounding Boxes ---------------%

thr = 35;
minArea = 20;

se = strel('rectangle', [5 5]);

trajectoryList = {}
distance = 0
lastCentroid = 0

for i=0:seqLength
    
    % Obtain current frame (image frame)
    imgfr = imread(sprintf(path,baseNum+i));
    
    % Obtain a binary image (img difference) -> (current frame - background) > threshold
    % 1, 2, 3 because of RGB
    imgdif = (abs(double(imgbk(:,:,1))-double(imgfr(:,:,1)))>thr) | ...
        (abs(double(imgbk(:,:,2))-double(imgfr(:,:,2)))>thr) | ...
        (abs(double(imgbk(:,:,3))-double(imgfr(:,:,3)))>thr);

    % Obtain binary image with morphologic operations
    bw = imclose(imgdif, se);
    bw = imerode(bw, se);

    % Labels the connected components in binary image, compute region
    % properties 
    [lb num] = bwlabel(bw);
    regionProps = regionprops(lb, 'area', 'FilledImage', 'Centroid');
    inds = find([regionProps.Area] > minArea);
    regnum = length(inds);

    subplot(2,2,1); imshow(imgfr);
    subplot(2,2,2); imshow(imgdif);
    subplot(2,2,3); imshow(bw);

    % Compute bounding boxes
    if regnum
        for j=1:regnum
            
            [lin col] = find(lb == inds(j));
            
            % y x
            upLPoint = min([lin col]);
            % h w
            dWindow = max([lin col]) - upLPoint + 1;

            position = [fliplr(upLPoint) fliplr(dWindow)];

            % val1 val2
            centroid = regionProps(j).Centroid;
            
            % add position to trajetoryList
            centroid
            j
            trajectoryList
            lastCentroid
            [distance, trajectoryList, lastCentroid] = updateTrajectory(centroid, j, trajectoryList);
            % call a function to update the trajectory and draw it
            
            % position x y h w
            rectangle('Position', position, ...
                'EdgeColor', [1 0 0], 'LineWidth', 2);

            if ~(distance==0)
%                 line('Position', [centroid(1) lastCentroid(2)], [centroid(2) lastCentroid(1)], ...
%                     'EdgeColor', [0 0 1], 'LineWidth', 1);
                hold on;
                plot(centroid(1),centroid(2), 'g+', 'MarkerSize', 3, 'LineWidth', 2);
            end

            % 
        end
    end
    drawnow
end


% ---------------------   3 Plot Trajectories       --------------------%

function [distance, trajectoryList, lastCentroid] = updateTrajectory(newCentroid, label, trajectoryList)
    label
    newCentroid
    distance = 0
    lastCentroid = 0
    if isempty(trajectoryList)
        trajectoryList{label} = newCentroid;
    elseif length(trajectoryList) < label
        trajectoryList{label} = newCentroid;
    else
        lastCentroid = trajectoryList{label}(end,:); 

        % Euclidean distance
        distance = sqrt(sum((newCentroid(1) - lastCentroid(1)).^2 + (newCentroid(2) - lastCentroid(2)).^2));
    end
end
    


%function = updateTracjectory (newPosition){
    % see if there is anything in that position
    % if not, add that
    % if yes, calculate the difference between the last position and new
    % position (compare it with the centroides
    % euclidian distance?
    % add the new position to the list
    % returns the difference -> for it to traces the trajectory

    % we have to handle occlusions
% end

% ---------------------   4 Consistent Labels       --------------------%

% --------------------------   5 Heatmap       -----------------------%

% --------------------------   6 EM Algorithm    -----------------------%

% ----------------  7 Evaluation Perfomance Algorithm  ------------------%
