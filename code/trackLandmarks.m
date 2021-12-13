function S_i = trackLandmarks(S,image, T_WC_i, args)
    % S -> Current state
    % image -> Image we want to use to add landmarks
    % T_WC_i -> pose of camera that took image

    % Get harris scores, keypoints and descriptors of the image we want to
    % use to add landmarks
    harris_scores_current = harris(image, args);
    keypoints_current = selectKeypoints(...
        harris_scores_current, args);
    descriptors_current = describeKeypoints(image, keypoints_current, args);

    [~, n_tracks] = size(S.C);
    [~, n_keypoints_current] = size(keypoints_current);

    if n_tracks == 0
        matches = zeros(n_keypoints_current);
    else
        matches = matchDescriptors(descriptors_current, S.D, args); 

        figure(4);
        imshow(image);
        hold on;
        plot(keypoints_current(2, :), keypoints_current(1, :), 'rx', 'Linewidth', 2);
        
        keypoints_first = [S.C(2,:); S.C(1,:)];
        plotMatches(matches, keypoints_current, keypoints_first);
    end
    F = [];
    C = [];
    T = [];
    D = [];

    for i=1:length(matches)
        idx_first = matches(i);
        if idx_first == 0
            % No match, create new track
            F = [F [keypoints_current(2, i); keypoints_current(1,i)]];
            T = [T reshape(T_WC_i,[12,1])];
            D = [D descriptors_current(:,i)];
        else
            % Match, update current keyframe (C)
            F = [F S.F(:, idx_first)];
            T = [T S.T(:, idx_first)];
            D = [D S.D(:, idx_first)];
        end
        C = [C [keypoints_current(2, i); keypoints_current(1,i)]];

    end

    % Triangulate a point cloud using the final transformation (R,T)
    P = S.P;
    X = S.X;
    
    % For each keypoint in the track, triangulate landmark and check alpha.
    % If alpha is "good" then add to real pool of keypoints and landmarks
    % (P and X) and discard from candidate pool
    for j = 1:args.num_keypoints
        i = args.num_keypoints-j+1;

        T_WC_first = reshape(T(:,i),[3,4]);
        R_CW_first = T_WC_first(:, 1:3)';
        t_CW_first = -T_WC_first(:,1:3)'*T_WC_first(:,4);
        T_CW_first = [R_CW_first t_CW_first];
        M_first = args.K * T_CW_first;

        R_CW_i = T_WC_i(:, 1:3)';
        t_CW_i = -T_WC_i(:,1:3)'*T_WC_i(:,4);
        T_CW_i = [R_CW_i t_CW_i];
        M_i = args.K * T_CW_i;


        point_2d_first = [F(:,i); 1];
        point_2d_current = [C(:,i); 1];
        point_3d = linearTriangulation(point_2d_first,point_2d_current,M_first,M_i);
        point_3d = point_3d(1:3);
%         point_3d = point_3d + T_WC_first(:,4);

        
        v1 = point_3d(1:3)-T_WC_first(:,4);
        v2 = point_3d(1:3)-T_WC_i(:,4);
        a = atan2d(norm(cross(v1,v2)),dot(v1,v2)); % Angle in degrees

        if a > .5 && point_3d(3) > 0 
            P = [P point_2d_current(1:2)];
            X = [X point_3d];
            C(:,i) = [];
            F(:,i) = [];
            T(:,i) = [];
            D(:,i) = [];
        end
    end

    disp('Tracked  = ');
    
    S_i.C = C;
    S_i.F = F;
    S_i.T = T;
    S_i.D = D;
    S_i.P = P;
    S_i.X = X;

    hold off



end