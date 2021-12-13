function processFrame(S_i, image, pose_hist, n_tracked_hist, P);

    % Plot
    height = 2; width = 4;
    figure(20);

    % Image with keypoints
    subplot(height,width,[1,2])
    imshow(image);
    hold on;
    plot(S_i.C(1, :), S_i.C(2, :), 'ro', 'Linewidth', 2);
    plot(S_i.P(1, :), S_i.P(2, :), 'gx', 'Linewidth', 2);    
    hold off
    daspect([1 1 1])


    % Trajectory + landmarks (top view)
    subplot(height,width,[3,4,7,8])
    plot(pose_hist(1, max(end-20,1):end), pose_hist(3, max(end-20,1):end));
    hold on
    plot(S_i.X(1,:), S_i.X(3,:), 'rx')
    hold off
            
    % Number of triangulated landmarks
    subplot(height,width,5)
    plot(n_tracked_hist);

    % Number of triangulated landmarks
    subplot(height,width,6)
    pbaspect([1 1 1])

    plot(pose_hist(1, :), pose_hist(3, :));
    daspect([1 1 1])
    pbaspect([1 1 1])


end