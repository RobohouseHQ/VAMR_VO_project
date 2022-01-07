function [E, mask, cameraParams] = determineEssentialMatrix(p1, p2, K)

    %[F, mask] = estimateFundamentalMatrix(p1(1:2,:)', p2(1:2,:)'); %Proobably this is not required
    cameraParams = cameraParameters('IntrinsicMatrix', K');
    [E, mask] = estimateEssentialMatrix(p1(1:2, :)', p2(1:2, :)', cameraParams);

end
