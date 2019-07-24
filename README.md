## FP.1 Match 3D Objects

    loop all the bounding boxes in previous frame
        loop all the matches 
            check if match.previous_keypoint inside the previous bounding box
                get the corresponding current_keypoint
                loop all the bounding boxes in current frame 
                    check if the currrent_keypoint inside the current bounding box
                        if inside, count the number of match pair :(prevbox, currbox)
        find the max matches box in current frame, restore the bounding box pair in the result map.
    
    Note: in order to make the result stable, I make three steps :
    1. when I compute the max match box, I wil also try to compute the ratio = prev_box_size/ curr_box_size. If the ratio is less than threshold, it means the box size change to much in those two frame, it should be something wrong. I will filter this match box out.
    2. Besides the size of box, I will also compare the center position of the box, if the box position change too large (the euclidean distance is large than 100), I will also filter this box out.
    3. If the matches points number of two boxes is less than 5, I will filter this box out.



## FP.2 Compute Lidar-based TTC

First of all, we use Constant Velocity Model to compute TTC.

The TTC compute process:
1. First, we need to find the x-position of object back in previous frame: distance_prev_x
2. Then, we need to find the x-position of object back in current frame: distance_curr_x
3. We compute the distance the object move: distance_x=distance_prev_x - distance_curr_x
4. We compute the velocity of the object : velocity_x = distance_x / delta_time
5. We compute the curr_nearest_distance of the object. (use another average statistic method to avoid outlier to the distance computation)
6. We assume the velocity of the object is same, then compute the TTC = curr_nearest_distance / velocity_x;
Note: if the relative velocity_x is negative, the TTC should be maximum.

In order to make the system stable, I implmenet two method to avoid ghost points.

One is for velocity computation:
We will use the following method to avoid outlier Lidar points to the velocity computation:  
1. First, we will sort the Lidar points by x-position.
2. Then, we will pick top N (N=200) points.
3. We will use IQR method (the Q1=1/4 of top N points) to compute the center of the object back.
4. Finally, we use the average value of Q1-1, Q1, Q1+1 to compute the final value, as the center of object's back.

Another is for nearest distance computation
We will use the following method to avoid outlier Lidar points to the nearest distance computation:  
1. First, we will sort the Lidar points by x-position.
2. Then, we will pick top N (N=200) points.
3. We will use IQR method (the Q1=1/10 of top N points) as the standard value.
4. In order to avoid ghost point in near, We will pick top 5 points of the points. If the distance of the point between standard value is more than 5cm, we will remove the point.

## FP.3 Associate Keypoint Correspondences with Bounding Boxes

    loop all the matches
        check if the matches.keypoint inside the bounding box
        if inside, add it to the vector

In order to make the system stable, I implment two method to avoid wrong matches.

Method 1st:
    compute the mean of matches euclidean distance.
    loop all the matches inside the vector
        compute the euclidean distance between the matches (previous keypoint and current keypoint)
        if the abs(mean-distance) > threshold (for example. 300)
            then remove this match. Because it's obviously wrong.

Method 2nd:
    outer loop all the matches in the box
        get match point pair A
        inner loop all the matches in the box
            get match point pair B
            get the line from A to B
            compute the ratio of Previous and Current line's length; 
            restore the ratio
        compute the average ratio value start from A
    compute the average ratio value of all points

    loop all the matches in the box
        if the average ratio start from A - average ratio of all points > threshold
            remove this match



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

For TTC estimation, I think there will be two aspects can influence the estimation performance (ignore time used):
3.1 TTC computation estimation model (constant velocity, or constant acceleration)
3.2 The distance estimation of object (with lidar measurement)

I think for lidar points, the distance information is precise. 

If we don't consider estimation model, if the lidar points is correctly match to its original object, the TTC computation should be precise.

For lidar points, I think the main error may come in two ways:

- The lidar points match to the wrong object.
  It means lidar points of object1 match to object2.
  This problem is very common. Because now we use Yolov3 to get the object rectangle from the image.
  But the detection is not so precise. The object in real world may have a lot of shape, but in here we use rectangle to represent it.
  In the future, we can use semantic segmentation to get more precise object boundary.

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
After checking the log, the reason is mismatch of bounding boxes. The bounding box has been match to another wrong box.But in most cases, the matching works well. See combination(SIFT, BRISK)




## FP.6 Performance Evaluation 2 _Camera TTC Compute

For Camera-based TTC estimation, i make the tests of some combination and list performance frame by frame in the following table.
If we ignore the process time, the SIFT-detector will have good performance. But it will take more than 120ms. It is unacceptable. 
Since our job is collision avoid system, it has strong real-time requirement. And the image come every 100ms, we also have a lot of other job to perform, i think the time used for detector and descriptor must be less than 10ms. So i think only FAST can match our requirement.

I have computed all the combinations and store the result of them in the build/log/ directory. You will find them in that directory. 
In the following table, i have record all the combination and their result in test.
By the way, I use iamge from 25 to 55, image_step=2. So there is 14 frames in the table.


|col:the frame<br/>row:detector<br/>descriptor | 1 | 2| 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11| 12| 13 | 14 |
|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|
|AKAZE+AKAZE|8.88|9.01|9.89|10.44|8.70|7.43|8.76|9.02|7.02|8.50|9.41|12.17|18.67|59.29|
|AKAZE+BRIEF|9.20|9.26|9.96|10.79|8.48|7.43|9.09|8.34|6.98|8.57|9.65|12.33|19.66|73.81|
|AKAZE+BRISK|8.62|8.87|9.97|10.35|8.48|7.25|8.58|7.60|6.80|8.22|9.45|11.84|18.83|50.80|
|AKAZE+FREAK|8.80|9.03|10.01|10.33|8.82|7.49|8.04|7.79|7.69|8.41|9.53|11.95|19.51|56.49|
|AKAZE+ORB  |8.73|8.92|9.34|10.52|8.30|7.25|8.59|8.27|7.17|8.89|9.81|12.56|19.75|65.62|
|AKAZE+SIFT |8.98|8.95|9.91|10.78|8.42|7.64|8.56|8.73|7.02|8.79|9.86|11.92|18.93|61.28|
|BRISK+BRIEF| 10.51| 10.19| 11.40| 11.66| 9.39| 7.91| 9.57| 7.33| 6.78| 7.55| 9.56| 12.61| 17.36| 52.23|
|BRISK+BRISK| 11.04| 11.20| 14.78| 13.00| 9.12| 7.65| 8.66| 7.42| 6.87| 8.28| 9.01| 11.36| 18.49| 54.73|
|BRISK+FREAK| 10.90| 12.14| 12.23| 11.67| 9.88| 8.23| 8.27| 8.17| 6.67| 8.15| 9.93| 12.30| 19.13| 52.58|
|BRISK+ORB| 11.38| 11.73| 13.69| 11.63| 9.31| 8.07| 8.51| 7.45| 7.23| 8.40| 9.00| 11.66| 17.94| 55.47|
|BRISK+SIFT| 11.60| 12.31| 16.53| 15.62| 12.54| 7.70| 9.25| 8.97| 7.47| 7.95| 9.72| 12.41| 17.67| 90.56|
|FAST+BRIEF| 9.07| 10.64| 11.39| 12.78| 10.06| 7.56| 9.22| 7.11| 7.03| 8.22| 9.34| 12.03| 17.80| 53.56|
|FAST+BRISK| 10.26| 10.04| 11.71| 11.66| 9.74| 7.69| 8.29| 6.56| 6.94| 8.14| 8.92| 11.53| 17.45| 44.41|
|FAST+FREAK| 8.81| 12.65| 13.26| 11.66| 8.76| 7.41| 10.11| 6.98| 7.16| 8.24| 8.45| 11.42| 15.45| 48.89|
|FAST+SIFT| 10.76| 10.83| 11.75| 11.99| 9.42| 7.58| 10.30| 7.41| 7.20| 8.22| 10.00| 11.79| 18.58| 51.68|
|HARRIS+BRIEF| 9.77| 21.91| 13.60| 27.29| 14.67| 10.20| 22.37| 10.42| 11.11| 13.28| 24.49| 25.94| 46.00|-inf|
|HARRIS+BRISK| 9.88| 12.82| 10.91| 13.57| 9.20| 10.23| 11.24| 9.21| 10.37| 12.34| 42.87| 16.16| 86.53|-inf|
|HARRIS+FREAK| 7.73| 15.27| 12.46| 24.86| 6.72|104.66| 10.42| 11.40| 9.94| 16.55| 23.64| 52.77|-inf|-28.59|
|HARRIS+ORB| 9.53| 13.58| 13.30| 18.97| 13.33| 10.20| 27.72| 7.27| 10.83| 12.61| 24.49| 25.94| 39.96|-inf|
|HARRIS+SIFT| 9.53| 15.25| 13.60| 25.20| 13.33| 10.20| 27.72| 11.43| 12.10| 12.14| 24.49| 20.60| 91.67|-inf|
|ORB+BRIEF| 11.32| 13.20| 13.63| 12.55| 9.16| 8.39| 10.90| 14.70| 10.12| 12.28| 16.50| 28.48| 30.38|-inf|
|ORB+BRISK| 13.18| 15.70| 14.83| 16.49| 11.54| 8.60| 11.13| 12.20| 12.14| 15.55| 17.16| 40.22| 86.59|-inf|
|ORB+FREAK| 8.96| 13.83| 12.82| 17.80| 10.95| 8.82| 17.24| 19.83| 14.64| 21.67| 56.58|541.19|-inf|-inf|
|ORB+ORB| 11.23| 12.61| 15.47| 14.57| 13.05| 8.11| 11.44| 10.25| 10.34| 10.34| 12.97| 24.05| 28.61|-inf|
|ORB+SIFT| 16.00| 14.08| 20.85| 20.36| 17.05| 9.41| 10.02| 12.09| 12.93| 11.14| 16.97| 43.11| 29.21|-inf|
|SHITOMASI+BRIEF| 8.82| 10.11| 11.39| 12.08| 9.52| 8.13| 7.99| 7.16| 6.86| 7.74| 9.40| 10.23| 18.14| 47.26|
|SHITOMASI+BRISK| 9.11| 9.80| 9.81| 11.23| 8.68| 7.84| 8.12| 6.95| 6.62| 7.37| 9.00| 9.81| 17.30| 40.95|
|SHITOMASI+FREAK| 8.97| 9.46| 9.77| 10.74| 8.95| 7.63| 7.81| 6.93| 6.42| 7.52| 9.20| 9.80| 16.28| 43.50|
|SHITOMASI+ORB| 8.25| 9.87| 10.73| 11.74| 9.18| 8.01| 7.78| 6.91| 6.89| 7.66| 9.32| 10.29| 18.00| 43.99|
|SHITOMASI+SIFT| 8.86| 10.81| 11.33| 11.83| 9.03| 8.14| 8.56| 7.44| 6.91| 7.58| 9.41| 10.11| 18.01| 42.25|
|SIFT+BRIEF| 9.02| 8.47| 10.28| 11.19| 9.21| 7.76| 7.63| 7.18| 6.93| 7.35| 8.83| 10.68| 15.47| 40.27|
|SIFT+BRISK| 9.21| 8.71| 9.94| 10.40| 8.58| 7.25| 7.40| 6.77| 6.33| 6.88| 8.43| 10.16| 14.90| 40.30|
|SIFT+FREAK| 9.05| 8.54| 9.44| 10.16| 8.60| 7.13| 6.74| 7.27| 6.63| 6.90| 8.44| 10.15| 15.38| 40.12|
|LIDAR      |7.58|7.05|9.23|6.86 |9.17|6.82|6.70|6.18|6.73|6.11|9.07|10.77|14.58|23.36|

1. I find the performance of camera based TTC estimation is depends on the combinations of detectors and descriptors. I think the reason is : the different detector and descriptor is suitable for different scenarios. Maybe in real world, we should implement a lots of combination and test them in each situation. According to the currrent situation, choose the right one.

2. For one combination of the detector and descriptors, the TTC estimation may be change not stable from time to time. I think it may caused by the lightness/intensity change, or the object change in image (scale/rotation/affine transformation).

3. For camera TTC estimation, it can only compute the relative change in image. It can not measure the distance of the object. The TTC computation result is not precise.
Especially when the image change very fast, the TTC computation is not stable.
The reason maybe the matching between two images is hard, when the image change a lot. So the estimation is not good.

