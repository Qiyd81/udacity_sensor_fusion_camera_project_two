## FP.1 Match 3D Objects

    loop all the bounding boxes in previous frame
        loop all the matches 
            check if match.previous_keypoint inside the previous bounding box
                get the corresponding current_keypoint
                loop all the bounding boxes in current frame 
                    check if the currrent_keypoint inside the current bounding box
                        if inside, count the number of match pair :(prevbox, currbox)
        find the max matches box in current frame, restore the bounding box pair in the result map.


## FP.2 Compute Lidar-based TTC

First of all, we use Constant Velocity Model to compute TTC.

We will use the following method to avoid outlier Lidar points:  
1. First, we will sort the Lidar points by x-position.
2. Then, we will pick top N (N=10) points, to compute the average distance.
3. Check the first 3 points, if the (average_distance-distance) > 0.1 , don't use this point as nearest point, find the next one.


The TTC compute process:
1. First, we need to find the x-position of object in previous frame: distance_prev_x
2. Then, we need to find the x-position of object in current frame: distance_curr_x
3. We compute the distance the object move: distance_x=distance_prev_x - distance_curr_x
4. We compute the velocity of the object : velocity_x = distance_x / delta_time
5. We assume the velocity of the object is same, then compute the TTC = distance_curr_X / velocity_x;
Note: if the relative velocity_x is negative, the TTC should be maximum.


## FP.3 Associate Keypoint Correspondences with Bounding Boxes

    loop all the matches
        check if the matches.keypoint inside the bounding box
        if inside, add it to the vector

    compute the mean of matches euclidean distance.
    loop all the matches inside the vector
        compute the euclidean distance between the matches (previous keypoint and current keypoint)
        if the abs(mean-distance) > threshold (for example. 300)
            then remove this match. Because it's obviously wrong.


## FP.4 Compute Camera-based TTC

    out_loop each matches
        in_loop each matches
            compute the distance ratio for each match pair 
            save the distance ratio in a vector

    sort the distance ratio vector, find the median value of distance ratio.
    use the median distance ratio to compute the TTC.

# Performance Evaluation

To make it simple, we use constant velocity model, and the TTC only focus on the related two frames.
So the performance of the TTC compute may not good enough.
In the future, we may change to constant acceleration model, and use more frames to make the estimation more precise.

## FP.5 Performance Evaluation 1 _Lidar TTC Compute

For TTC estimation, I think there will be two aspects can influence the performance:
3.1 TTC computation estimation model (constant velocity, or constant acceleration)
3.2 The distance estimation of object (with lidar measurement)

I think for lidar points, the distance information is precise. 

If we don't consider estimation model, if the lidar points is correctly match to its original object, the TTC computation should be precise.

For lidar points, I think the main error may come in two ways:

- The lidar points match to the wrong object.
  It means lidar points of object1 match to object2.
  This problem is very common. Because now we use Yolov3 to get the object rectangle from the image.
  But the detection is not so precise. The object in real world may have a lot of shape, but in here we use rectangle to represent it.
  We can use semantic segmentation to get more precise object boundary.

- There is no object, but some points come.
  I think in real world, this problem may rise in sometime. Because we measure the lidar points by the light reflection. But for some object have big or small reflection feature, especially the mirror and water, there will be some ghost lidar points.
  It's hard to filter them out. 
  I think we should handle them by some software technology. For example, we can get more precise tracking of the object. Some sudden come points will not be counted.
  In one word, we don't trust the points in one frame. We should trust the object which appear in several frames.

Here i will mark some problems I found in the test:

1. In some cases, there are two bounding boxes found in front. 
   The main lidar points belong to the leading vehicle. But there are a few points of leading vehicle are mismatch to the right vehicle. 
   
   Understanding: 
   This error maybe caused by the shrink operation.
   In order to filter out the outlier, we make the bounding box small. So some of its points are filter out. But when there is another bounding box near by with some intersection of that box. Maybe the points will be mismatched. Then will get another bounding box.

   Solution: Maybe we can use smaller box to compute the lidar. To filter more boundary points out.
   I make the float shrinkFactor = 0.30; this error is fix.

   I think , in the future, maybe we can don't use bounding box to filter the lidar points, we may need instance segmentation to filter the outlier points. That's more precise.

   And further, the shrinkFactor can change depends on the object's distance. If we get closer, we can change the shrinkFactor to get smaller box. Because for closer object, we don't need so much lidar points to get the enough information. 

2. In some cases, there will be the front-front car's lidar came to the ROI of box.
   But it does not cause the error in TTC estimation.

3. When the front vehicle is stop, in some cases the lidar will measurement small movement. When we compute the TTC. The TTC will be very large.
   For this error, i guess it may cause by the lidar depth resolution or some lidar physical issue.

4. In some cases, the TTC compute will be very strange: the x-distance of previous frame will be very large. 
After checking the log, the reason is mismatch of bounding box. The bounding box has been match to another box in this case.But in most cases, the matching works well. See combination(SIFT, BRISK)




## FP.6 Performance Evaluation 2 _Camera TTC Compute

For Camera-based TTC estimation, i make the test of some combination and list some precise performance in the following table.
I find the suitable combination is very hard to choose. If we ignore the process time, the SIFT-detector will have good performance. But it will take more than 120ms. It is unacceptable. Since the image come every 100ms, we also have a lot of other job to perform, i think the time used for detector and descriptor must be less than 10ms. So i think only FAST can match our requirement. But FAST TTC estimation performance is not so good. Finally, I choose FAST+BRISK.

In the following table, i make the test TTC estimation performance by score (min:1 , max:3). (ignore time used)


|col:detector<br/>row:descriptor | SHITOMASI | HARRIS | FAST | BRISK | ORB | AKAZE | SIFT |
|-|-|-|-|-|-|-|-|
BRISK|2|2|2|1|2|3
BRIEF|2|2|2|1|2|3
ORB|2|2|2|1|2|_
FREAK|2|2|1|1|2|2
AKAZE|_|_|_|_|2|_
SIFT|2|2|1|1|2|3


1. I find the performance of camera based TTC estimation is very unstable. Depends on the combinations of detectors and descriptors, the performance change large. 

For one combination of the detector and descriptors, the TTC estimation may also change very fast from time to time. I think it may caused by the lightness/intensity change, or the object change in image (scale/rotation/affine transformation).

2. For camera TTC estimation, it can only compute the relative change in image. It can not measure the distance of the object. The TTC computation result is not stable.
Especially when the image change very fast, the TTC computation is not stable.
The reason maybe the matching between two images is hard, when the image change a lot. So the estimation is not good.

