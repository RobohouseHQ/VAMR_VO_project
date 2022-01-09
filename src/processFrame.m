function [S_i, T_WC_i, hidden_state, observations] = processFrame(img_i, img_prev, S_prev, hidden_state, observations, args)
    % update pose and pointcloud
    % img_i: image to obtain pose from
    % img_prev: image used to match descriptors with img_i
    % S_prev: previous state

    % Add new entry to observations struct
    observations.O = [observations.O struct('k', 0, 'p', [], 'l', [])];

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
    
    % Update hidden state twist history
    hidden_state.twists = [hidden_state.twists HomogMatrix2twist([T_WC_i; zeros(1, 3) 1])];
    observations.n = observations.n + 1;

    disp('Estimated inlier ratio is');
    disp(nnz(inlier_mask) / numel(inlier_mask));

    % TODO: compare keeping outliers vs removing them from P and X
    S_i.P = matched_keypoints_query(1:2, inlier_mask);
    S_i.X = matched_P_database(1:3, inlier_mask);
    S_i.C = S_prev.C;
    S_i.F = S_prev.F;
    S_i.T = S_prev.T;

    % number of landmarks observed in this query image/frame
    observations.O(end).k = size(S_i.P, 2);
    observations.O(end).p = S_i.P;

    if length(observations.O) < 2 % this might not be necessary if I properly implement the addition of new landmarks in trackLandmarksKLT
        observations.m = size(S_i.X, 2);
        observations.O(end).l = 1:size(S_i.X, 2); % initial landmark id's
        hidden_state.landmarks = S_i.X;
    else
        % Find the indices of the landmarks that were tracked by the keypoints in this frame (no new landmarks being added in this step)
        first_filter_idx = find(point_validity);
        second_filtered_idx = first_filter_idx(inlier_mask);

        % Make indices globally consistent by matching them on the landmark index of the previous frame
        %         final_filtered_idx = observations.O(end - 1).l(second_filtered_idx);
        final_filtered_idx = second_filtered_idx;
        assert(length(final_filtered_idx) == observations.O(end).k);
        observations.O(end).l = final_filtered_idx;

        % Number of landmarks does not change
        % observations.m = observations.m;
        % hidden_state.landmarks = hidden_state.landmarks;
        % end
    end
