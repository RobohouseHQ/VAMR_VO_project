function [R_C2_W, T_C2_W, T_W_C2, R_W_C2,p1_mask, p2_mask,P] = extractFinalPose (p1,p2,mask,E,K)

    % Extract the relative camera positions (R,T) from the essential matrix

    p1_mask = p1(:, mask);
    p2_mask = p2(:, mask);

    % Obtain extrinsic parameters (R,t) from E
    [Rots,u3] = decomposeEssentialMatrix(E);

    % Disambiguate among the four possible configurations
    [R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p1_mask,p2_mask,K,K);

    %Compute M
    M1 = K * eye(3,4);
    M2 = K * [R_C2_W, T_C2_W];
    
    %Triangulate new landmarks 
    P = triangulate(p1_mask(1:2,:)',p2_mask(1:2,:)',M1',M2');
    P = P';
    
    R_W_C2 = R_C2_W';
    T_W_C2 = -R_W_C2*T_C2_W;

end
  

