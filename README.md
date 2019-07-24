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
By the way, I use iamge from 25 to 55, image_step=2. So there is 15 frames in the table.


|col:the frame<br/>row:detector<br/>descriptor | 1 | 2| 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11| 12| 13 | 14 | 15 |
|LIDAR      |7.58|7.05|9.23|6.86 |9.17|6.82|6.70|6.18|6.73|6.11|9.07|10.77|14.58|23.36|NAN|
|AKAZE+AKAZE|8.88|9.01|9.89|10.44|8.70|7.43|8.76|9.02|7.02|8.50|9.41|12.17|18.67|59.29|-109.05|
|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|
|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|
|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|
|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|
|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|
|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|

1. I find the performance of camera based TTC estimation is depends on the combinations of detectors and descriptors. I think the reason is : the different detector and descriptor is suitable for different scenarios. Maybe in real world, we should implement a lots of combination and test them in each situation. According to the currrent situation, choose the right one.

2. For one combination of the detector and descriptors, the TTC estimation may also change very fast from time to time. I think it may caused by the lightness/intensity change, or the object change in image (scale/rotation/affine transformation).

3. For camera TTC estimation, it can only compute the relative change in image. It can not measure the distance of the object. The TTC computation result is not stable.
Especially when the image change very fast, the TTC computation is not stable.
The reason maybe the matching between two images is hard, when the image change a lot. So the estimation is not good.

