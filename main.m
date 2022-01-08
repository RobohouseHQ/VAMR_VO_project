

%% Setup
clear all;
close all;
clc;
ds = 2; % 0: KITTI, 1: Malaga, 2: parking
parking_path = 'data/parking';
kitti_path = 'data/kitti';
plot_ground_truth = true;

% rng(0)

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
    ground_truth = [];
    plot_ground_truth = false;
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


args.corner_patch_size = 15;
args.harris_kappa = 0.04;
args.num_keypoints = 1000;
args.nonmaximum_supression_radius = 11;
args.descriptor_radius = 11;
args.match_lambda = 7;
args.K = K;


harris_scores_0 = harris(img0, args);
keypoints_0 = selectKeypoints(...
    harris_scores_0, args);

keypoints = keypoints_0 ;
keypoints= flipud(keypoints)

pointTracker = vision.PointTracker
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
figure(4);
imshow(img1);
hold on;
plot(keypoints_end(2, :), keypoints_end(1, :), 'rx', 'Linewidth', 2);

plotMatches(1:size(keypoints_end, 2), keypoints_end, keypoints_ini);

matched_keypoints_0 = keypoints_ini(:, point_validity);
matched_keypoints_1 = keypoints_end(:, point_validity);

%% Estimate the essential matrix E using the 8-point algorithm


p1 = [matched_keypoints_0(2,:); matched_keypoints_0(1,:); ones(1, length(matched_keypoints_0))];
p2 = [matched_keypoints_1(2,:); matched_keypoints_1(1,:); ones(1, length(matched_keypoints_1))];

% Compute the essential matrix from the fundamental matrix given K


% Option 1: Use Matlab built in function
[F, mask] = estimateFundamentalMatrix(p1(1:2,:)', p2(1:2,:)');
IntrinsicMatrix = K';
radialDistortion = [0 0]; 
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix); 
[E1, mask] = estimateEssentialMatrix(p1(1:2,:)', p2(1:2,:)', cameraParams);

% E1 = K'*F*K;
% Option 2: Recycle code from exercise, but no RANSAC implemented
E2 = estimateEssentialMatrixx(p1,p2,K,K);

% Extract the relative camera positions (R,T) from the essential matrix

p1_mask = p1(:, mask);
p2_mask = p2(:, mask);

% Obtain extrinsic parameters (R,t) from E
[Rots,u3] = decomposeEssentialMatrix(E1);

% Disambiguate among the four possible configurations
[R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p1_mask,p2_mask,K,K)

% Triangulate a point cloud using the final transformation (R,T)
M1 = K * eye(3,4);
M2 = K * [R_C2_W, T_C2_W];

P = triangulate(p1_mask(1:2,:)',p2_mask(1:2,:)',M1',M2');
P = P';
matched_keypoints_1_mask = matched_keypoints_1(:, mask);
matched_keypoints_0_mask = matched_keypoints_0(:, mask);

% perform RANSAC to find best Pose and inliers
 [R_C2_W, T_C2_W, inlier_mask, max_num_inliers_history, num_iteration_history] = ...
         ransacLocalization(matched_keypoints_1_mask, P(1:3,:), K);
 
 [R_W_C2,T_W_C2, inlier_mask] = estimateWorldCameraPose(flipud(matched_keypoints_1_mask)',P(1:3,:)',cameraParams);

 T_W_C2 = T_W_C2';
 R_W_C2 = R_W_C2';
 

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
S_i.P = [];
S_i.X = [];
S_i.C = [];
S_i.F = [];
S_i.T = [];
S_i.D= [];

fh = figure(20)

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
plotCO(S_i, images{1}, pose_hist, n_tracked_hist,ground_truth, plot_ground_truth);

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
    plotCO(S_i, image, pose_hist, n_tracked_hist, ground_truth, plot_ground_truth);
    pause(.01);
    


end