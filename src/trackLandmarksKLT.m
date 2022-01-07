function S_i = trackLandmarksKLT(S, images, T_WC_i, args, beta)
    % S -> Current state
    % image -> Image we want to use to add landmarks
    % T_WC_i -> pose of camera that took image

    % Get harris scores, keypoints and descriptors of the image we want to
    % use to add landmarks

    keypoints_img = detectKeypoints(images{size(images, 1)}, args);

    F = [];
    C = [];
    T = [];

    if size(S.C, 2) ~= 0

        keypoints_ini = S.C;
        pointTracker = vision.PointTracker('MaxBidirectionalError', .5);
        initialize(pointTracker, keypoints_ini', images{1});

        for i = 2:size(images, 1)
            img = images{i};
            [keypoints_end, point_validity] = pointTracker(img);

        end

        %         [keypoints_end,point_validity] = pointTracker(image);
        %         keypoints_ini = keypoints_ini(:, point_validity);
        keypoints_end = keypoints_end';
        keypoints_end = keypoints_end(:, point_validity);

        %         figure(4)
        %         imshow(images{size(images,1)});
        %         hold on;
        %         plotMatches(1:size(keypoints_end),2), keypoints_end), S.F(:, point_validity)));
        %         hold off;
        %         pause(0.01);
        C = keypoints_end;
        F = S.F(:, point_validity);
        T = S.T(:, point_validity);

    end

    % Triangulate a point cloud using the final transformation (R,T)
    P = S.P;
    X = S.X;

    % For each keypoint in the track, triangulate landmark and check alpha.
    % If alpha is "good" then add to real pool of keypoints and landmarks
    % (P and X) and discard from candidate pool
    n_candidates = size(C, 2);

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

        %         point_2d_first = [F(:,i); 1];
        %         point_2d_current = [C(:,i); 1];
        %         point_3d = linearTriangulation(point_2d_first,point_2d_current,M_first,M_i);
        %         point_3d = point_3d(1:3);

        % Use matlab function, passing in point_2d_first, point_2d_current
        % and the respective projection matrices
        point_3d = triangulate(F(:, i)', C(:, i)',M_first', M_i')';

        v1 = point_3d(1:3) - T_WC_first(:, 4);
        v2 = point_3d(1:3) - T_WC_i(:, 4);
        a = atan2d(norm(cross(v1, v2)), dot(v1, v2)); % Angle in degrees
            
        %TODO: add condition for distance of landmark
        if a > beta
            % add candidate keypoint to set of keypoints
            P = [P C(:,i)];
            % add landmark of keypoint to set of tracked landmarks
            X = [X point_3d];
            
            % remove keypoint from candidate keypoints
            C(:, i) = [];
            T(:, i) = [];
            F(:, i) = [];

        end

    end
    
    % Hacky, TODO: select with replacement to avoid duplicates
    for i = 1:100
        r = randi([1 args.num_keypoints]);
        F = [F keypoints_img(:, r)];
        T = [T reshape(T_WC_i, [12, 1])];
        C = [C keypoints_img(:, r)];
    end

    disp('Tracked  = ');

    S_i.C = C;
    S_i.F = F;
    S_i.T = T;
    S_i.P = P;
    S_i.X = X;

end
