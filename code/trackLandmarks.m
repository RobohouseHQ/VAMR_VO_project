function S_i = trackLandmarks(S,image, T_WC_i, args)



    harris_scores_current = harris(image, args);
    keypoints_current = selectKeypoints(...
        harris_scores_current, args);
    descriptors_current = describeKeypoints(image, keypoints_current, args);

    [~, n_tracks] = size(S.C)
    [~, n_keypoints_current] = size(keypoints_current)

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

    length(matches)
    args.num_keypoints

    for i=1:length(matches)
        idx_first = matches(i);
        if idx_first == 0
            % No match, create new track
            F = [F [keypoints_current(2, i); keypoints_current(1,i)]];
            C = [C [keypoints_current(2, i); keypoints_current(1,i)]];
            T = [T reshape(T_WC_i,[12,1])];
            D = [D descriptors_current(:,i)];
        else
            % Match, update current keyframe
            F = [F S.F(:, idx_first)];
            C = [C [keypoints_current(2, i); keypoints_current(1,i)]];
            T = [T S.T(:, idx_first)];
            D = [D S.D(:, idx_first)];
        end
    end

    % Triangulate a point cloud using the final transformation (R,T)
    
    

    P = S.P;
    X = S.X;
    size(C)
    n_tracks
    
    for j = 1:args.num_keypoints
        i = args.num_keypoints-j+1;

        T_WC_first = reshape(T(:,i),[3,4]);
        M_first = args.K * T_WC_first;
        M_current = args.K * T_WC_i;
        point_2d_first = [F(:,i); 1];
        point_2d_current = [C(:,i); 1];
        point_3d = linearTriangulation(point_2d_first,point_2d_current,M_first,M_current);

    
        v1 = point_3d(1:3)-T_WC_first(:,4);
        v2 = point_3d(1:3)-T_WC_i(:,4);
        a = atan2d(norm(cross(v1,v2)),dot(v1,v2)) % Angle in degrees
        if a > .05 
            P = [P point_2d_current(1:2)];
            X = [X point_3d(1:3)];
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

    % Makes sure that plots refresh.    
    pause(0.1);


end