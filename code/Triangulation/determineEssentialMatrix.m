function [E,mask,cameraParams] = determineEssentialMatrix(p1,p2,K)

    %[F, mask] = estimateFundamentalMatrix(p1(1:2,:)', p2(1:2,:)'); %Proobably this is not required
    IntrinsicMatrix = K';
    radialDistortion = [0 0]; 
    cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix); 
    [E, mask] = estimateEssentialMatrix(p1(1:2,:)', p2(1:2,:)', cameraParams);

end