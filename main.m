%% Setup
clear;
close all;
clc;
path(pathdef); % Reset paths
addpath(genpath('src')); % Source code


%% User settings
ds = 2; % 0: KITTI, 1: Malaga, 2: parking
parking_path = 'data/parking';
kitti_path = 'data/kitti';
malaga_path = 'data/malaga-urban-dataset-extract-07';
%custom_ds_path = 'data/undistorted_recording_7';
custom_ds_path = 'data/undistorted_front_walk';
plot_ground_truth = false;

rng(0)

%% Dataset loading
if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [4 8 12])';
    last_frame = 4540;

elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    plot_ground_truth = false;
    ground_truth = []; % gives GPS data

elseif ds == 2
    % Path containing images, depths and all...
    %     assert(exist('datasets/parking', 'dir') ~= 0);
    last_frame = 598;
    % K = load([parking_path '/K.txt']);

    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [4 8 12])';
elseif ds == 3
    assert(exist('custom_ds_path', 'var') ~= 0);
    last_frame = 729; 
    ground_truth =[];
else
    assert(false);
end

%Load tuning parameters for Initialization
initArgs = readJson(ds).Init;

%% Bootstrap
%set bootstrap_frames
bootstrap_frames = [10 13];

if ds == 0
    img0 = imread([kitti_path '/05/image_0/' ...
                    sprintf('%06d.png', bootstrap_frames(1))]);
    img1 = imread([kitti_path '/05/image_0/' ...
                    sprintf('%06d.png', bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
                            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                            left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
                            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                            left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
                            sprintf('/images/img_%05d.png', bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
                            sprintf('/images/img_%05d.png', bootstrap_frames(2))]));
elseif ds == 3
    img0 = imread([custom_ds_path '/images/' ...
                    sprintf('%06d.png', bootstrap_frames(1))]);
    img1 = imread([custom_ds_path '/images/' ...
                    sprintf('%06d.png', bootstrap_frames(2))]);
else
    assert(false);
end

%Feature Detection
keypoints = detectKeypoints(img0, initArgs);

%Initialise KLT
pointTracker = vision.PointTracker('MaxBidirectionalError', initArgs.max_bidir_err);
initialize(pointTracker, keypoints', img0);
keypoints_ini = keypoints;

for i = 1:bootstrap_frames(2)

    if ds == 0
        img = imread([kitti_path '/05/image_0/' ...
                    sprintf('%06d.png', i)]);
    elseif ds == 1
        img = rgb2gray(imread([malaga_path ...
                                '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                                left_images(i).name]));
    elseif ds == 2
        img = rgb2gray(imread([parking_path ...
                                sprintf('/images/img_%05d.png', i)]));
    elseif ds == 3
        img = imread([custom_ds_path '/images/' ...
                    sprintf('%06d.png', i)]);
    else
        assert(false);
    end

    [points, point_validity] = pointTracker(img);

end

keypoints_end = points';
keypoints_end = flipud(keypoints_end);
keypoints_ini = flipud(keypoints_ini);

%Plot the image with the keypoints alone
plotKeypoints(img1, keypoints_end, keypoints_ini);

matched_keypoints_0 = keypoints_ini(:, point_validity);
matched_keypoints_1 = keypoints_end(:, point_validity);

%% Triangulate and determine final pose

p1 = [matched_keypoints_0(2, :); matched_keypoints_0(1, :); ones(1, length(matched_keypoints_0))];
p2 = [matched_keypoints_1(2, :); matched_keypoints_1(1, :); ones(1, length(matched_keypoints_1))];

% Compute the essential matrix from the fundamental matrix given K
[E, mask, cameraParams] = determineEssentialMatrix(p1, p2, initArgs.K);

%Determine final pose (with RANSAC)
[R_C2_W, t_C2_W, t_W_C2, R_W_C2, p1_mask, p2_mask, P] = extractFinalPose (p1, p2, mask, E, initArgs.K);

%Visualise the 3D scene
% visualise3DScene(img0, img1, P, R_C2_W, t_C2_W, p1, p2)

%% Continuous operation
img_prev = img1;
S_i.P = [];
S_i.X = [];
S_i.C = [];
S_i.F = [];
S_i.T = [];
S_i.D = [];

fh = figure(20);

% 3xN array of camera positions
pos_hist = [t_W_C2];
n_tracked_hist = [];

% Hidden state for BA
hidden_state.twists = HomogMatrix2twist([[R_W_C2 t_C2_W]; zeros(1, 3) 1]);
hidden_state.landmarks = P;

% Observation struct for BA
observations.n = size(hidden_state.twists, 2); % number of frames/poses
observations.m = size(hidden_state.landmarks, 2); % number of landmarks
assert(size(p2_mask, 2) == size(P, 2), "number of keypoints in frame 2 and num of triangulated landmarks does not match");
% k is the number of landmarks observed in the frame, ...
% p are the keypoints in the image frame that correspond to ...
% the landmarks in index positions l in the hidden_state.landmarks array
observations.O = [struct('k', size(p2_mask, 2), 'p', p2_mask, 'l', 1:observations.m)]; % struct array

hidden_state_flat = [reshape(hidden_state.twists, 1, []), reshape(hidden_state.landmarks, 1, [])]';

% Plot BA
% hidden_state_refined = runBA(hidden_state_flat, observations, initArgs.K);
% plotBAMap(hidden_state_flat, hidden_state_refined, observations, [0 40 -10 10]);

for i = 1:size(p1_mask, 2)
    S_i.C = [S_i.C p1_mask(1:2, i)];
    S_i.F = [S_i.F p1_mask(1:2, i)];
    S_i.T = [S_i.T reshape(eye(3, 4), [12, 1])];
end

images = cell(bootstrap_frames(2) - 1, 1);

for i = 1:bootstrap_frames(2)

    if ds == 0
        images{i} = imread([kitti_path '/05/image_0/' sprintf('%06d.png', i)]);
    elseif ds == 1
        images{i} = rgb2gray(imread([malaga_path ...
                                    '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                                    left_images(i).name]));
    elseif ds == 2
        images{i} = im2uint8(rgb2gray(imread([parking_path ...
                                            sprintf('/images/img_%05d.png', i)])));
    elseif ds == 3
        images{i} = imread([custom_ds_path '/images/' sprintf('%06d.png', i)]);
    else
        assert(false);
    end

end

S_i = trackLandmarksKLT(S_i, images, [R_W_C2 t_W_C2], initArgs);
plotCO(S_i, images{1}, pos_hist, n_tracked_hist, 1, ground_truth, plot_ground_truth);

%Load tuning parameters for CO
continuousArgs = readJson(ds).CO;


% last_frame = 20; % Testing
range = (bootstrap_frames(2) + 1):last_frame;

for i = (bootstrap_frames(2)+1):1: last_frame
    %fprintf('\n\nProcessing frame %d\n=====================\n', i);

    if ds == 0
        image = imread([kitti_path '/05/image_0/' sprintf('%06d.png', i)]);
        image_prev = imread([kitti_path '/05/image_0/' sprintf('%06d.png', i - 1)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
                                '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                                left_images(i).name]));
        image_prev = rgb2gray(imread([malaga_path ...
                                    '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                                    left_images(i - 1).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
                                        sprintf('/images/img_%05d.png', i)])));
        image_prev = im2uint8(rgb2gray(imread([parking_path ...
                                                sprintf('/images/img_%05d.png', i - 1)])));
    elseif ds == 3
        image = imread([custom_ds_path '/images/' sprintf('%06d.png', i)]);
        image_prev = imread([custom_ds_path '/images/' sprintf('%06d.png', i - 1)]);
    else
        assert(false);
    end

    S_prev = S_i;

    [S_i, T_WC_i] = processFrame(image, image_prev, S_i, continuousArgs);
    images = cell(2, 1);
    images{1} = image_prev;
    images{2} = image;

    S_i = trackLandmarksKLT(S_i, images, T_WC_i, continuousArgs);

    pos_hist = [pos_hist T_WC_i(:, 4)];
    n_tracked = length(S_i.X);
    n_tracked_hist = [n_tracked_hist n_tracked];

    plotCO(S_i, image, pos_hist, n_tracked_hist, i + 1, ground_truth, plot_ground_truth);
    pause(.01);

end

pp_G_C = ground_truth(:, 1:length(pos_hist));

p_G_C = alignEstimateToGroundTruth(pp_G_C, pos_hist);

figure(2);
plot(pp_G_C(3, :), -pp_G_C(1, :));
hold on;
plot(pos_hist(3, :), -pos_hist(1, :));
plot(p_G_C(3, :), -p_G_C(1, :));
hold off;
axis equal;
% axis([-5 95 -30 10]);
legend('Ground truth', 'Original estimate', 'Aligned estimate', ...
'Location', 'SouthWest');
