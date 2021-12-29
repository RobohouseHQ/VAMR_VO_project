
function [final_keypoints, keypoint_locations] = computeFastFeatures(I)

    final_keypoints = detectFASTFeatures(I);
    keypoint_locations = final_keypoints.Location;
    
end