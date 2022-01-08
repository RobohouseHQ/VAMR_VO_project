function plotBAMap(hidden_state, hidden_state_refined, observations, range)

    num_frames = observations.n;

    p_W_landmarks_before = getLandmarkPositions(hidden_state, num_frames);
    p_W_frames_before = getFramePositions(hidden_state, num_frames);

    p_W_landmarks_after = getLandmarkPositions(hidden_state_refined, num_frames);
    p_W_frames_after = getFramePositions(hidden_state_refined, num_frames);

    figure(15)
    axis equal;
    subplot(1, 2, 1)
    plot(p_W_landmarks_before(3, :), -p_W_landmarks_before(1, :), '.');
    hold on;
    plot(p_W_frames_before(3, :), -p_W_frames_before(1, :), 'rx', 'Linewidth', 3);
    hold off;
    title('Full problem before bundle adjustment');
    axis(range);

    subplot(1, 2, 2)
    plot(p_W_landmarks_after(3, :), -p_W_landmarks_after(1, :), '.');
    hold on;
    plot(p_W_frames_after(3, :), -p_W_frames_after(1, :), 'rx', 'Linewidth', 3);
    hold off;
    title('Full problem after bundle adjustment');
    axis(range);

end

function p_W_frames = getFramePositions(hidden_state, num_frames)
    T_W_frames = reshape(hidden_state(1:num_frames * 6), 6, []);

    p_W_frames = zeros(3, num_frames);

    for i = 1:num_frames
        T_W_frame = twist2HomogMatrix(T_W_frames(:, i));
        p_W_frames(:, i) = T_W_frame(1:3, end);
    end

end

function p_W_landmarks = getLandmarkPositions(hidden_state, num_frames)

    p_W_landmarks = reshape(hidden_state(num_frames * 6 + 1:end), 3, []);

end
