% function [R_C2_W, T_C2_W, T_W_C2, R_W_C2, inlier_mask,p1_mask, p2_mask,P] = extractFinalPose (p1,p2,mask,E,K,matched_keypoints_0,matched_keypoints_1,cameraParams)
% 
%     % Extract the relative camera positions (R,T) from the essential matrix
% 
%     p1_mask = p1(:, mask);
%     p2_mask = p2(:, mask);
% 
%     % Obtain extrinsic parameters (R,t) from E
%     [Rots,u3] = decomposeEssentialMatrix(E);
% 
%     % Disambiguate among the four possible configurations
%     [R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p1_mask,p2_mask,K,K)
% 
%     %Compute M
%     M1 = K * eye(3,4);
%     M2 = K * [R_C2_W, T_C2_W];
%     
%     %Triangulate new landmarks 
%     P = triangulate(p1_mask(1:2,:)',p2_mask(1:2,:)',M1',M2');
%     P = P';
%     
%     matched_keypoints_1_mask = matched_keypoints_1(:, mask);
%     matched_keypoints_0_mask = matched_keypoints_0(:, mask);
% 
% %     perform RANSAC to find best Pose and inliers
%         [R_C2_W, T_C2_W, inlier_mask, max_num_inliers_history, num_iteration_history] = ...
%          ransacLocalization(matched_keypoints_1_mask, P(1:3,:), K);
%  
%         [R_W_C2,T_W_C2, inlier_mask] = estimateWorldCameraPose(flipud(matched_keypoints_1_mask)',P(1:3,:)',cameraParams);
% 
%     T_W_C2 = T_W_C2';
%     R_W_C2 = R_W_C2';
%     
% %     T_final = inv([R_C2_W T_C2_W
% %                     0 0 0 1])
% %             
% %     T_W_C2 = T_final(1:3,4);
% %     R_W_C2 = T_final(1:3,1:3);
% 
%        
%     %inlier_mask = 0;
%       
% 
% end

function [R_C2_W, T_C2_W, T_W_C2, R_W_C2,p1_mask, p2_mask,P] = extractFinalPose (p1,p2,mask,E,K)

    % Extract the relative camera positions (R,T) from the essential matrix

    p1_mask = p1(:, mask);
    p2_mask = p2(:, mask);

    % Obtain extrinsic parameters (R,t) from E
    [Rots,u3] = decomposeEssentialMatrix(E);

    % Disambiguate among the four possible configurations
    [R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p1_mask,p2_mask,K,K);

    %Compute M
    M1 = K * eye(3,4);
    M2 = K * [R_C2_W, T_C2_W];
    
    %Triangulate new landmarks 
    P = triangulate(p1_mask(1:2,:)',p2_mask(1:2,:)',M1',M2');
    P = P';
%     matched_keypoints_1_mask = matched_keypoints_1(:, mask);
%     matched_keypoints_0_mask = matched_keypoints_0(:, mask);
% 
%     % perform RANSAC to find best Pose and inliers
%     [R_C2_W, T_C2_W, inlier_mask, max_num_inliers_history, num_iteration_history] = ...
%     ransacLocalization(matched_keypoints_1_mask, P(1:3,:), K);
% 
%     [R_W_C2,T_W_C2, inlier_mask] = estimateWorldCameraPose(flipud(matched_keypoints_1_mask)',P(1:3,:)',cameraParams);
    
    R_W_C2 = R_C2_W';
    T_W_C2 = -R_W_C2*T_C2_W;

end
  

