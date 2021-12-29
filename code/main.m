

%% Setup
clear all;
close all;
clc;
ds = 0; % 0: KITTI, 1: Malaga, 2: parking
parking_path = 'data/parking'
kitti_path = 'data/kitti05/kitti'
malaga_path = 'G:/malaga-urban-dataset-extract-07'
plot_ground_truth = false;

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;

    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    
    % Path containing images, depths and all...
%     assert(exist('datasets/parking', 'dir') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end


%% Bootstrap
% need to set bootstrap_frames
bootstrap_frames = [1 3]
if ds == 0
    img0 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));

else
    assert(false);
end

%Load tuning parameters
args = readJson(ds).TuningParameters;

%K input (better way for this, will think about it, prob integrate in config)
args.K = K;

%Feature Detection
keypoints = detectKeypoints(img0,args)

%Initialise KLT 
pointTracker = vision.PointTracker('MaxBidirectionalError',1);
initialize(pointTracker,keypoints',img0);
keypoints_ini = keypoints

for i = 1:bootstrap_frames(2)
    if ds == 0
        img = imread([kitti_path '/05/image_0/' ...
            sprintf('%06d.png',i)]);
    elseif ds == 1
        img = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        img = rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)]));
    else
        assert(false);
    end
  
    [points,point_validity] = pointTracker(img);

end

keypoints_end = points';
keypoints_end= flipud(keypoints_end);
keypoints_ini = flipud(keypoints_ini);

%Plot the image with the keypoints alone
plotKeypoints(img1,keypoints_end, keypoints_ini);

matched_keypoints_0 = keypoints_ini(:, point_validity);
matched_keypoints_1 = keypoints_end(:, point_validity);

%% Triangulate and determine final pose 

p1 = [matched_keypoints_0(2,:); matched_keypoints_0(1,:); ones(1, length(matched_keypoints_0))];
p2 = [matched_keypoints_1(2,:); matched_keypoints_1(1,:); ones(1, length(matched_keypoints_1))];

% Compute the essential matrix from the fundamental matrix given K
[E, mask, cameraParams] = determineEssentialMatrix(p1,p2,K);

%Determine final pose (with RANSAC)
[R_C2_W, T_C2_W, T_W_C2, R_W_C2,inlier_mask,p1_mask, p2_mask, P] = extractFinalPose(p1,p2,mask,E,K,matched_keypoints_0,matched_keypoints_1,cameraParams)

%Print for testing
% sum(inlier_mask)/length(inlier_mask)
% T_C2_W
% T_W_C2
% P;

%Visualise the 3D scene
visualise3DScene(img0, img1, P, R_C2_W, T_C2_W, p1, p2)


%% Continuous operation
img_prev = img1;
S_i.P = [];
S_i.X = [];
S_i.C = [];
S_i.F = [];
S_i.T = [];
S_i.D= [];

fh = figure(20)
    fh.WindowState = 'maximized';
pause(10)

pose_hist = [zeros(3,1)];
n_tracked_hist = [0];


for i = 1:size(p2_mask, 2)
    S_i.C = [S_i.C p1_mask(1:2,i)];
    S_i.F = [S_i.F p1_mask(1:2,i)];
    S_i.T = [S_i.T reshape(eye(3,4), [12,1])];
end

images = cell(bootstrap_frames(2)-1,1);
 for i = 1:bootstrap_frames(2)
     if ds == 0
        images{i} = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        images{i} = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        images{i} = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
 
 end
 
S_i = trackLandmarksKLT(S_i, images, [R_W_C2 T_W_C2], args,0.1);
plotCO(S_i, images{1}, pose_hist, n_tracked_hist, P,0,ground_truth,plot_ground_truth);
T_W_C2
R_W_C2
size(images, 1)
range = (bootstrap_frames(2)+1):last_frame;
for i = range
    iter = i;
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
        image_prev = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i-1)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
        image_prev = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i-1).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
        image_prev = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i-1)])));
    else
        assert(false);
    end

    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    S_prev = S_i;

    [S_i, T_WC_i] = processFrame(image, image_prev, S_i, args);
    images = cell(2,1);
    images{1} = image_prev;
    images{2} = image;

    S_i = trackLandmarksKLT(S_i, images, T_WC_i, args,0.5);

    pose_hist = [pose_hist T_WC_i(:,4)];
    n_tracked = length(S_i.X);
    n_tracked_hist = [n_tracked_hist n_tracked];

     %S_i = trackLandmarks(S_i, image, T_WC_i, args);
        plotCO(S_i, image, pose_hist, n_tracked_hist, P,i, ground_truth,plot_ground_truth);
    pause(.01);
    
end