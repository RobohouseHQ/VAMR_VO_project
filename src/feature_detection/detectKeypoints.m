function [keypoints] = detectKeypoints(img,args)

%Harris feature detector based on exercise code 

    harris_scores = harris(img, args);
    keypoints = selectKeypoints(...
        harris_scores, args);
    
    keypoints= flipud(keypoints);


end
