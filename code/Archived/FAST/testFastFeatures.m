%% main
test_image = imread('test_im2.png');

test_image = rgb2gray(test_image);
[kp, kp_l] = computeFastFeatures(test_image);

%% Plotting the features

figure(1);
imshow(test_image,[]);
hold on;
plot(kp.selectStrongest(150));

%% functions

function [final_keypoints, keypoint_locations] = computeFastFeatures(I)

    final_keypoints = detectFASTFeatures(I);
    keypoint_locations = final_keypoints.Location;
    
end