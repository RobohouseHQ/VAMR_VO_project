function plotKeypoints(img,keypoints_end, keypoints_ini)

    figure(4);
    imshow(img);
    hold on;
    plot(keypoints_end(2, :), keypoints_end(1, :), 'rx', 'Linewidth', 2);
    plotMatches(1:size(keypoints_end, 2), keypoints_end, keypoints_ini);

end

