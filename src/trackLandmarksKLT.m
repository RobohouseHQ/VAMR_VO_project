function [hidden_state, observations, S_i] = trackLandmarksKLT(S, images, T_WC_i, hidden_state, observations, args)
    % S -> Current state
    % image -> Image we want to use to add landmarks
    % T_WC_i -> pose of camera that took image

    % Get harris scores, keypoints and descriptors of the image we want to
    % use to add landmarks

    keypoints_img = detectKeypoints(images{size(images, 1)}, args);

    F = [];
    C = [];
    T = [];

    % If candidate keypoints array not empty, track keypoints and update their location in the last image frame, and if lost, delete its tracks from T and F
    if size(S.C, 2) ~= 0

        keypoints_ini = S.C;
        pointTracker = vision.PointTracker('MaxBidirectionalError', 0.5);
        initialize(pointTracker, keypoints_ini', images{1});

        for i = 2:size(images, 1)
            img = images{i};
            [keypoints_end, point_validity] = pointTracker(img);

        end

        keypoints_end = keypoints_end';

        C = keypoints_end(:, point_validity);
        F = S.F(:, point_validity);
        T = S.T(:, point_validity);

    end

    % Triangulate a point cloud using the final transformation (R,T)
    P = S.P;
    X = S.X;

    % For each keypoint in the track, triangulate landmark and check its quality.
    % If landmark is valid or "good" then add to real pool of keypoints and landmarks
    % (P and X) and discard from candidate pool
    n_candidates = size(C, 2);
    % Candidate keypoint filtering: should check that these candidate keypoints are not already in P before adding them
    for i = n_candidates:-1:1

        T_WC_first = reshape(T(:, i), [3, 4]);
        R_CW_first = T_WC_first(:, 1:3)';
        t_CW_first = -T_WC_first(:, 1:3)' * T_WC_first(:, 4);
        T_CW_first = [R_CW_first t_CW_first];
        M_first = args.K * T_CW_first;

        R_CW_i = T_WC_i(:, 1:3)';
        t_CW_i = -T_WC_i(:, 1:3)' * T_WC_i(:, 4);
        T_CW_i = [R_CW_i t_CW_i];
        M_i = args.K * T_CW_i;

        % Use matlab function, passing in point_2d_first, point_2d_current
        % and the respective projection matrices
        point_3d = triangulate(F(:, i)', C(:, i)',M_first', M_i')';

        v1 = point_3d(1:3) - T_WC_first(:, 4);
        v2 = point_3d(1:3) - T_WC_i(:, 4);
        alpha = atan2d(norm(cross(v1, v2)), dot(v1, v2)); % Angle in degrees

        isInFront_first = R_CW_first(3, :) * point_3d > -t_CW_first(3);
        isInFront_curr = R_CW_i(3, :) * point_3d > -t_CW_i(3);
        isCloseEnough = vecnorm(R_CW_i(3, :) * point_3d, 2) < args.max_dist_new_lmk;
        isFarEnough = vecnorm(R_CW_i(3, :) * point_3d, 2) > args.min_dist_new_lmk;
        % Add new landmark if it's valid
        if alpha > args.min_alpha_new_lmk %&& isCloseEnough && isFarEnough && isInFront_curr && isInFront_first
            % This is the place where new landmarks get added, so landmark labelling should be done here in the observation and hidden_state objects

            % % Find the existing landmark corresponding to this observed keypoint,
            % % if it doesn't match existing landmarks, then add a landmark and its
            % % index is observations.m + num_new_landmarks, which is unique?

            hidden_state.landmarks = [hidden_state.landmarks point_3d];
            observations.m = observations.m + 1;
            % observations.O(end).l = [observations.O(end).l observations.m];
            % % Only increase the number of new landmarks if this valid landmark is unique
            % num_new_landmarks = num_new_landmarks + 1;

            % % Enhance this check by comparing the 3D landmark
            % isDuplicate = ~isempty(P) && sum(abs(P(1, :) - C(1, i)) < 1 & abs(P(2, :) - C(2, i)) < 1) > 0;

            % add candidate keypoint to set of keypoints
            P = [P C(:, i)];
            % add landmark of keypoint to set of tracked landmarks
            X = [X point_3d];

            % remove keypoint from candidate keypoints
            C(:, i) = [];
            T(:, i) = [];
            F(:, i) = [];

        end

    end

    for i = 1:100
        r = randi([1 args.num_keypoints]);
        F = [F keypoints_img(:, r)];
        T = [T reshape(T_WC_i, [12, 1])];
        C = [C keypoints_img(:, r)];
    end

    % This approach makes more sense but it doesn't work as well or isn't
    % suitable for the set of tuned params
    % Ideally the number of added candidate keypoints is a function of
    % size(P,2) so that we don't run out of them
    %     for i = 1:args.num_keypoints
    %         % check that keypoints_img being added to C are not already in C
    %         isDuplicate = ~isempty(P) && sum(abs(C(1, :) - keypoints_img(1, i)) < 1 & abs(C(2, :) - keypoints_img(2, i)) < 1) > 0;
    %         if ~isDuplicate
    %             F = [F keypoints_img(:, i)];
    %             T = [T reshape(T_WC_i, [12, 1])];
    %             C = [C keypoints_img(:, i)];
    %         end
    %     end

    S_i.C = C;
    S_i.F = F;
    S_i.T = T;
    S_i.P = P;
    S_i.X = X;

end
