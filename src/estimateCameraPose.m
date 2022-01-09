function [S_i, T_WC_i] = estimateCameraPose(img_i, img_prev, S_prev, args)
    % update pose and pointcloud
    % img_i: image to obtain pose from
    % img_prev: image used to match descriptors with img_i
    % S_prev: previous state

    P_database = S_prev.X;
    keypoints_database = S_prev.P;

    pointTracker = vision.PointTracker('MaxBidirectionalError', args.max_bidir_err);
    initialize(pointTracker, keypoints_database', img_prev);
    [keypoints_query, point_validity] = pointTracker(img_i);
    matched_keypoints_query = keypoints_query';
    matched_keypoints_query = matched_keypoints_query(:, point_validity);
    matched_P_database = P_database(1:3, point_validity);
    %     matched_keypoints_database = keypoints_database(:, point_validity);

    %perform p3p + MSAC
    cameraParams = cameraParameters('IntrinsicMatrix', args.K');
    [R_WC, t_WC, inlier_mask] = estimateWorldCameraPose(matched_keypoints_query', matched_P_database', cameraParams);
    t_WC = t_WC';
    R_WC = R_WC';
    T_WC_i = [R_WC t_WC];

    disp('Estimated inlier ratio is');
    disp(nnz(inlier_mask) / numel(inlier_mask));

    % TODO: compare keeping outliers vs removing them from P and X
    S_i.P = matched_keypoints_query(1:2, inlier_mask);
    S_i.X = matched_P_database(1:3, inlier_mask);
    S_i.C = S_prev.C;
    S_i.F = S_prev.F;
    S_i.T = S_prev.T;

end
