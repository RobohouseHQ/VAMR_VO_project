function [S_i, T_WC_i] = processFrame(img_i, img_prev, S_prev, args)
    % img_i: image to obtain pose from
    % img_prev: image used to match descriptors with img_i 
    % S_prev: previous state
    
    
    img_query = img_i;
    P_database = S_prev.X;
    p_database = S_prev.P;
    
    K = args.K;
    
   
    keypoints_database = p_database;    
    
    pointTracker = vision.PointTracker('MaxBidirectionalError',.5);
    initialize(pointTracker,keypoints_database',img_prev);
    [keypoints_query,point_validity] = pointTracker(img_i);
    matched_keypoints_query = keypoints_query';
    matched_keypoints_query = matched_keypoints_query(:, point_validity);
    matched_P_database = P_database(1:3, point_validity);
    matched_keypoints_database = keypoints_database(:, point_validity);

    % perform RANSAC to find best Pose and inliers
    [R_C_W, t_C_W, inlier_mask, max_num_inliers_history, num_iteration_history] = ...
        ransacLocalization(flipud(matched_keypoints_query), matched_P_database, K);
    IntrinsicMatrix = K';
    cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix); 
    [R_WC,t_WC, inlier_mask] = estimateWorldCameraPose(matched_keypoints_query',matched_P_database',cameraParams);
    t_WC = t_WC';
    R_WC = R_WC';
    T_WC_i = [R_WC t_WC];

    

    disp('Estimated inlier ratio is');
    disp(nnz(inlier_mask)/numel(inlier_mask));


    
    % TODO: compare keeping outliers vs removing them from P and X
    S_i.P = matched_keypoints_query(1:2,inlier_mask);
    S_i.X = matched_P_database(1:3,inlier_mask);
    S_i.C = S_prev.C;
    S_i.F = S_prev.F;
    S_i.T = S_prev.T;


 

end



