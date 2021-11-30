

%% Setup
clear all;
close all;
clc;
ds = 0; % 0: KITTI, 1: Malaga, 2: parking
parking_path = 'data/parking'
kitti_path = 'data/kitti'

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
    assert(exist('datasets/parking', 'dir') ~= 0);
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
bootstrap_frames(1)
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


args.corner_patch_size =9;
args.harris_kappa = 0.08;
args.num_keypoints = 1000;
args.nonmaximum_supression_radius = 8;
args.descriptor_radius = 9;
args.match_lambda = 5;
args.K = K;

harris_scores_0 = harris(img0, args);
keypoints_0 = selectKeypoints(...
    harris_scores_0, args);
descriptors_0 = describeKeypoints(img0, keypoints_0, args);

harris_scores_1 = harris(img1, args);
keypoints_1 = selectKeypoints(...
    harris_scores_1, args);
descriptors_1 = describeKeypoints(img1, keypoints_1, args);


matches = matchDescriptors(descriptors_1, descriptors_0, args);




%% Load outlier-free point correspondences

[~, query_indices, match_indices] = find(matches);

matched_keypoints_1 = keypoints_1(:, query_indices);
matched_keypoints_0 = keypoints_0(:, match_indices);


figure(4);
imshow(img1);
hold on;
plot(matched_keypoints_1(2, :), matched_keypoints_1(1, :), 'rx', 'Linewidth', 2);

plotMatches(matches, keypoints_1, keypoints_0);
%% Estimate the essential matrix E using the 8-point algorithm


F = estimateFundamentalMatrix(matched_keypoints_0', matched_keypoints_1');
p1 = [matched_keypoints_0(2,:); matched_keypoints_0(1,:); ones(1, length(matched_keypoints_0))];
p2 = [matched_keypoints_1(2,:); matched_keypoints_1(1,:); ones(1, length(matched_keypoints_1))];


% Compute the essential matrix from the fundamental matrix given K
E1 = K'*F*K

% Option 2: Recycle code from exercise, but no RANSAC implemented
E2 = estimateEssentialMatrix(p1,p2,K,K)

%% Extract the relative camera positions (R,T) from the essential matrix

% Obtain extrinsic parameters (R,t) from E
[Rots,u3] = decomposeEssentialMatrix(E1);

% Disambiguate among the four possible configurations
[R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p1,p2,K,K);

% Triangulate a point cloud using the final transformation (R,T)
M1 = K * eye(3,4);
M2 = K * [R_C2_W, T_C2_W];
P = linearTriangulation(p1,p2,M1,M2)

%% Visualize the 3-D scene
figure(1),
subplot(1,3,1)
% R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]

% P is a [4xN] matrix containing the triangulated point cloud (in
% homogeneous coordinates), given by the function linearTriangulation
plot3(P(1,:), P(2,:), P(3,:), 'o');

% Display camera pose

plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

center_cam2_W = -R_C2_W'*T_C2_W;
plotCoordinateFrame(R_C2_W',center_cam2_W, 0.8);
text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

axis equal
rotate3d on;
grid

% Display matched points
subplot(1,3,2)
imshow(img0,[]);
hold on
plot(p1(1,:), p1(2,:), 'ys');
title('Image 1')

subplot(1,3,3)
imshow(img1,[]);
hold on
plot(p2(1,:), p2(2,:), 'ys');
title('Image 2')



%% Continuous operation
img_prev = img1;
S_prev.P = [];
S_prev.X = [];
S_prev.C = [];
S_prev.F = [];
S_prev.T = [];
S_prev.D = [];


S_i = trackLandmarks(S_prev, img0, eye(3,4), args);
S_prev = S_i;
S_i = trackLandmarks(S_prev, img1, [R_C2_W, T_C2_W], args);

pose_hist = [];
n_tracked_hist = []

range = (bootstrap_frames(2)+1):last_frame;
for i = range
    iter = i
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    S_prev = S_i;




    [S_i, T_WC_i] = processFrame(image, img_prev, S_prev, args);
    size(T_WC_i)
    pose_hist = [pose_hist T_WC_i(:,4)];

     figure(10);
    imshow(image);
    hold on;
    plot(S_i.C(1, :), S_i.C(2, :), 'rx', 'Linewidth', 2);

    plot(S_i.P(1, :), S_i.P(2, :), 'gx', 'Linewidth', 2);
    
    figure(3)
    plot(pose_hist(1, :), pose_hist(3, :));
    hold on
    plot(S_i.X(1,:), S_i.X(3,:), 'x')
    hold off

    figure(5)
    [~, n_tracked] = size(S_i.P);
    n_tracked_hist = [n_tracked_hist n_tracked]
  
    plot(n_tracked_hist);

    S_i = trackLandmarks(S_i, image, T_WC_i, args);

    img_prev = image;
end