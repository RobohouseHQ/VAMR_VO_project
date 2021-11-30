function [S_i, T_WC_i] = processFrame(img_i, img_prev, S_prev, args)
% Reproject 3D points given a projection matrix
%
% P: [nx3] coordinates of the 3d points in the world frame
% M_tilde: [3x4] projection matrix
% K: [3x3] camera matrix
%
% p_reproj: [nx2] coordinates of the reprojected 2d points

    

    img_query = img_i;
    P_database = S_prev.X;
    p_database = S_prev.P;

    K = args.K;

    keypoints_database = [p_database(2,:); p_database(1,:)];
    img_database = img_prev;
    descriptors_database = describeKeypoints(img_database, keypoints_database, args);

    harris_scores_query = harris(img_query, args);
    keypoints_query = selectKeypoints(harris_scores_query, args);
    descriptors_query = describeKeypoints(img_query, keypoints_query, args);
    
    matches = matchDescriptors(descriptors_query, descriptors_database, args);

    
    [~, query_indices, match_indices] = find(matches);

    matched_keypoints_query = keypoints_query(:, query_indices);
    matched_p_query = [matched_keypoints_query(2,:); matched_keypoints_query(1,:); ones(1,length(matched_keypoints_query))];


    matched_keypoints_database = keypoints_database(:, match_indices);
    matched_p_database = p_database(:, match_indices);
    matched_P_database = P_database(:, match_indices);
    matched_P_database = matched_P_database(1:3, :);

    
    figure(4);
    imshow(img_query);
    hold on;
    plot(keypoints_query(2, :), keypoints_query(1, :), 'rx', 'Linewidth', 2);
    plotMatches(matches, keypoints_query, keypoints_database);


    % perform RANSAC to find best Pose and inliers
    [R_C_W, t_C_W, inlier_mask, max_num_inliers_history, num_iteration_history] = ...
        ransacLocalization(matched_keypoints_query, matched_P_database, K);

    inlier_mask
    
    disp('Found transformation T_C_W = ');
    disp([R_C_W t_C_W; zeros(1, 3) 1]);
    disp('Estimated inlier ratio is');
    disp(nnz(inlier_mask)/numel(inlier_mask));

    figure(1),
    subplot(1,3,1)
    % R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]
    
    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    plot3(matched_P_database(1,:), matched_P_database(2,:), matched_P_database(3,:), 'o');
    
    % Display camera pose
    
    plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
    text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
    
    center_cam2_W = -R_C_W'*t_C_W;
    plotCoordinateFrame(R_C_W',center_cam2_W, 0.8);
    text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
    
    axis equal
    rotate3d on;
    grid
    
    % Display matched points
    subplot(1,3,2)
    imshow(img_database,[]);
    hold on
    plot(matched_p_database(1,:), matched_p_database(2,:), 'ys');
    title('Image 1')
    
    subplot(1,3,3)
    imshow(img_query,[]);
    hold on
    plot(matched_p_query(1,:), matched_p_query(2,:), 'ys');
    title('Image 2')
    

    T_WC_i = [R_C_W t_C_W];
    S_i.P = matched_p_query(1:2,inlier_mask==1);
    S_i.X = matched_P_database(1:3,inlier_mask==1);
    S_i.C = S_prev.C;
    S_i.F = S_prev.F;
    S_i.T = S_prev.T;
    S_i.D = S_prev.D;


    % Makes sure that plots refresh.    
    pause(0.1);

end



