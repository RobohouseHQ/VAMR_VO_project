function plotCO(S_i, image, pose_hist, n_tracked_hist, P, frame_number,ground_truth,plot_truth);

    % Plot
    height = 2; width = 4;

    figure(20)

    % Image with keypoints
    subplot(height,width,[1,2,3,4])
    imshow(image);
    hold on;
    plot(S_i.C(1, :), S_i.C(2, :), 'ro', 'Linewidth', 2);
    plot(S_i.P(1, :), S_i.P(2, :), 'gx', 'Linewidth', 2);
    hold off
    daspect([1 1 1])
    pbaspect([1 1 1])
    
    title(['Frame number = ', num2str(frame_number)]);
     


    % Trajectory + landmarks (top view)
    subplot(height,width,8)
    plot(pose_hist(1, max(end-20,1):end), pose_hist(3, max(end-20,1):end));
    hold on
    plot(S_i.X(1,:), S_i.X(3,:), 'rx')
    hold off
    title('Trajectory of last 20 frames and landmarks') 

            
    % Number of triangulated landmarks
    subplot(height,width,[5,6])
    plot(n_tracked_hist);
    title('# tracked landmarks over the last 20 frames') 

    % Number of triangulated landmarks
    subplot(height,width,7)

    plot(pose_hist(1, :), pose_hist(3, :)),'b';
    hold on
    if plot_truth
        plot(ground_truth(1:length(pose_hist),1), ground_truth(1:length(pose_hist),2),'r');
    end
    hold off
    daspect([1 1 1])
    pbaspect([1 1 1])
    title('Full trajectory') 

    sgtitle('Visual Odometry Pipeline')


end