function [final_keypoints, keypoint_locations] = computeSURFFeatures(I,args)

    metric_threshold = args.metric_threshold;
    num_octaves = args.num_octaves; 
    num_scale_levels = args.num_scale_levels;
    
    final_keypoints = detectSURFFeatures(I,'MetricThreshold',metric_threshold,'NumOctaves',num_octaves,'NumScaleLevels', num_scale_levels);
    keypoint_locations = final_keypoints.Location;
    
end