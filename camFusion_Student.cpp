
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 2, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-10, bottom+50), cv::FONT_ITALIC, 0.5, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-10, bottom+100), cv::FONT_ITALIC, 0.5, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    //all the input data is from databuffer, now we need to compute the keypoints and matches in the ROI.

    std::vector<cv::DMatch> matches;

    //loop all the matches.
    for(auto itMatch = kptMatches.begin(); itMatch != kptMatches.end(); ++itMatch)
    {
        int prevKeypointId= itMatch->queryIdx; // descriptor index in previous image
        int currKeypointId= itMatch->trainIdx;  // descriptor index in current image

        //check if the corresponding keypoints inside the ROI.
        if(boundingBox.roi.contains(kptsPrev[prevKeypointId].pt) && boundingBox.roi.contains(kptsCurr[currKeypointId].pt))
        {
            //add the matches inside ROI to the vector
            matches.push_back(*itMatch);
        }
        
        
    }

    //remove outliers
    //compute mean
    double mean=0;
    auto it=matches.begin();
    for(it=matches.begin(); it!=matches.end(); ++it){
        cv::KeyPoint prevPoint=kptsPrev[it->queryIdx];
        cv::KeyPoint currPoint=kptsCurr[it->trainIdx];
        double distance= sqrt((prevPoint.pt.x-currPoint.pt.x)*(prevPoint.pt.x-currPoint.pt.x)+(prevPoint.pt.y-currPoint.pt.y)*(prevPoint.pt.y-currPoint.pt.y));
        mean+=distance;
    }
    mean= mean/matches.size();

    
    double threshold=50; //max eculidean distance between keypoint matches
    it=matches.begin();
    while(it!= matches.end())
    {
        cv::KeyPoint prevPoint=kptsPrev[it->queryIdx];
        cv::KeyPoint currPoint=kptsCurr[it->trainIdx];

        double distance= sqrt((prevPoint.pt.x-currPoint.pt.x)*(prevPoint.pt.x-currPoint.pt.x)+(prevPoint.pt.y-currPoint.pt.y)*(prevPoint.pt.y-currPoint.pt.y));

        if(abs(distance-mean)>threshold)
        {   //remove the match obviously wrong: match point too far away.
            it=matches.erase(it);
        }
        else
        {
            ++it;
        }
    }

    boundingBox.kptMatches=matches;
    //cout<<"BoundingBox id: "<<boundingBox.boxID <<", matches number: "<<matches.size()<<endl;
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI_2nd(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    //find the matches in the bounding box.
    std::vector<cv::DMatch> matches;

    //loop all the matches.
    for(auto itMatch = kptMatches.begin(); itMatch != kptMatches.end(); ++itMatch)
    {
        int prevKeypointId= itMatch->queryIdx; // descriptor index in previous image
        int currKeypointId= itMatch->trainIdx;  // descriptor index in current image

        //check if the corresponding keypoints inside the ROI.
        if(boundingBox.roi.contains(kptsPrev[prevKeypointId].pt) && boundingBox.roi.contains(kptsCurr[currKeypointId].pt))
        {
            //add the matches inside ROI to the vector
            matches.push_back(*itMatch);
        }
    }

    //try to filter the wrong matches out
    //use the distance_previous_mean and distance_current_mean value to filter the mismatch boxes.
    double threshold = 0.2;
    vector<double> ratiosGlobal;
    vector<double> ratiosLocal;
    for (auto it1 = matches.begin(); it1 != matches.end(); ++it1)
    {
        vector<double> ratiosTmp;   
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        double distCurr, distPrev;
        for (auto it2 = matches.begin(); it2 != matches.end(); ++it2)
        {
            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);
            double ratio=0;

            if(distCurr<0.01||distPrev<0.01){
                ratio=0;
            }
            else{
                ratio = distCurr/distPrev;
                ratiosGlobal.push_back(ratio);
                ratiosTmp.push_back(ratio);
            }
                
        }

        //get the average mean value of ratio for outer loop match points
        double localMean=accumulate(ratiosTmp.begin(), ratiosTmp.end(), 0.0) / ratiosTmp.size();
        ratiosLocal.push_back(localMean);
    }

    //compute the average mean value of ratio for all matches pair
    double globalMean=accumulate(ratiosGlobal.begin(), ratiosGlobal.end(), 0.0) / ratiosGlobal.size();

    //check every match , if the local mean is too far away from global mean.
    auto itMatch=matches.begin();
    int i=0;
    while(itMatch!=matches.end())
    {
        if(abs(globalMean-ratiosLocal.at(i))>threshold){
            itMatch=matches.erase(itMatch);

            //std::cout<<"Filter matches, PrevId "<<itMatch->queryIdx<<", CurrId "<<itMatch->trainIdx <<endl;
        }
        else{
            ++itMatch;
        }

        i++;
    }
    boundingBox.kptMatches=matches;
    //cout<<"BoundingBox id: "<<boundingBox.boxID <<", matches number: "<<matches.size()<<endl;
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { 
        // outer kpt. loop
        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { 
            // inner kpt.-loop
            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { 
                // avoid division by zero
                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        }// eof inner loop over all matched kpts
    }// eof outer loop over all matched kpts


    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    std::sort(distRatios.begin(), distRatios.end());

    //compute mean value of distance ratio.
    double medianDistRatio=0;
    int index=distRatios.size()/2;
    if(distRatios.size()%2==0)
    {
      medianDistRatio = (distRatios[index-1]+distRatios[index])/2;
    }
    else
    {
      medianDistRatio = distRatios[index];
    }

    double dT = 1 / frameRate;
    TTC = -  dT / (1 - medianDistRatio);
    cout<<"Distance Ratio Size: "<<distRatios.size()<<", MedianRatio:"<<medianDistRatio<<", TTC :"<<TTC<<endl;
}

bool compareFunction (double  i, double j) { return i<j; }

double computeLidarDis(const std::vector<LidarPoint> & lidarPoints)
{
    double dis=1e8;
    if(lidarPoints.size()==0)
        return dis;

    int N=200;
    //sort the distance, find top N.
    std::vector<double> distanceVector;
    for(auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    {
        distanceVector.push_back(it->x);
    }

    std::sort(distanceVector.begin(), distanceVector.end(), compareFunction);
    
    if(distanceVector.size()<=N)
        N=(distanceVector.size());

    int Q1_Index=distanceVector.size()/10; 
    double standardValue=distanceVector[Q1_Index];   

    double threshold=0.05;
    int nearestIndex=0;
    for(nearestIndex=0; nearestIndex<5 ; ++nearestIndex){
        if(standardValue-distanceVector[nearestIndex]>threshold)
        {
            std::cout<<"Skip point , standardValue: "<<standardValue<<", pointValue: "<<distanceVector[nearestIndex]<<endl;
            continue;
        }
        else{
            break;
        }
    }

    dis=distanceVector[nearestIndex];

    return dis;
    
}

//compute the object's center position by IQR.
double computeLidarObjectCenter_IQR(const std::vector<LidarPoint> & lidarPoints)
{
    double dis=1e8;
    if(lidarPoints.size()==0)
        return dis;

    int N=200;
    //sort the distance
    std::vector<double> distanceVector;
    int i=0;
    for(auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    {
        i++;
        distanceVector.push_back(it->x);

        if(i==N)
            break;
    }

    std::sort(distanceVector.begin(), distanceVector.end(), compareFunction);

    //use IQR to compute the distance
    //we will use the Q1=1/4 as the distance
    //we will compute the average value of Q1-1, Q1, Q1+1
    int Q1_Index=distanceVector.size()/4;
    dis=(distanceVector[Q1_Index-1]+distanceVector[Q1_Index]+distanceVector[Q1_Index+1])/3;

    return dis;
    
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    //compute previous lidar point -> object center
    double disPrev =  computeLidarObjectCenter_IQR(lidarPointsPrev);

    //compute current lidar point -> object center
    double disCurr =  computeLidarObjectCenter_IQR(lidarPointsCurr);
    double deltaTime = 1.0 / frameRate ; 

    //compute relative speeed of the object
    double velocity = (disPrev - disCurr) / deltaTime;

    //compute the current distance of object
    double disNear = computeLidarDis(lidarPointsCurr);

    if(velocity < 0 )
    {
        //object leave us , can't compute TTC, return max value;
        TTC = NAN;
    }
    else
    {
        TTC = disNear/ velocity ;
    }

    std::cout<< "Lidar TTC, disPrev: "<<disPrev <<", disCurr: "<<disCurr<<", velocity: "<<velocity<<", Near: "<<disNear<<", TTC: "<<TTC<<endl;
}


//Find the matching boundingbox from previous and current frame.
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // iterate all boundingbox in previous frame, find the corresponding matched box for every one.
    for( auto itPrevBox = prevFrame.boundingBoxes.begin(); itPrevBox!= prevFrame.boundingBoxes.end() ; ++itPrevBox)
    {
        //vector store the number of matchPoints with each box in current frame.
        map<int, int> matchPointNum; //first: current box id , second: current box matched point number.

        //loop all the matches, to find the matches inside the previous box
        for( auto itMatch= matches.begin(); itMatch!= matches.end(); ++itMatch)
        {
            int prevKeypointId= itMatch->queryIdx; // descriptor index in previous image
            int currKeypointId= itMatch->trainIdx;  // descriptor index in current image

            //the descriptor index is equal with the keypoint index
            cv::KeyPoint prevKeypoint= prevFrame.keypoints[prevKeypointId];
            cv::KeyPoint currKeypoint= currFrame.keypoints[currKeypointId];

            //check if the source keypoint inside the boundingbox
            float x=prevKeypoint.pt.x;
            float y=prevKeypoint.pt.y;

            if( x < itPrevBox->roi.x || itPrevBox->roi.x + itPrevBox->roi.width < x ||
                y < itPrevBox->roi.y || itPrevBox->roi.y + itPrevBox->roi.height < y ){
                    //skip the point not inside the roi
                    continue;
                }

            //loop all current boundingbox , check if the target point inside the box
            //if inside, count the number 
            float targetX=currKeypoint.pt.x;
            float targetY=currKeypoint.pt.y;
            for(auto itCurrBox = currFrame.boundingBoxes.begin(); itCurrBox!=currFrame.boundingBoxes.end(); ++itCurrBox)
            {
                if( targetX < itCurrBox->roi.x || itCurrBox->roi.x + itCurrBox->roi.width < targetX ||
                    targetY < itCurrBox->roi.y || itCurrBox->roi.y + itCurrBox->roi.height < targetY ){
                    //skip the point not inside the roi
                    continue;
                }

                //matchPointNum[itCurrBox->boxID]++;
                matchPointNum[itCurrBox->boxID] = matchPointNum[itCurrBox->boxID]+1;
            }
        }

        //find the boundingbox with max matches
        int max=-1;
        int maxId=-1;
        for(auto it = matchPointNum.begin(); it!=matchPointNum.end(); ++it)
        {
            int currBoxID= it->first;
            if(it->second > max){

                //compute the ratio of change to avoid mismatch
                double threshold_ratio = 0.7;
                double prevBoxSize = itPrevBox->roi.width*itPrevBox->roi.height;
                double currBoxSize = -1;
                double distance_threshold=100;
                double distance= 1000;
                for (auto itCurrBox = currFrame.boundingBoxes.begin(); itCurrBox!=currFrame.boundingBoxes.end(); ++itCurrBox)
                {
                     //compare the box center , if the center is too far away , we will filter this box out.
                    if(itCurrBox->boxID==it->first)
                    {
                        currBoxSize = itCurrBox->roi.width * itCurrBox->roi.height;
                        distance = sqrt((itCurrBox->roi.x-itPrevBox->roi.x)*(itCurrBox->roi.x-itPrevBox->roi.x)+(itPrevBox->roi.y-itCurrBox->roi.y)*(itPrevBox->roi.y-itCurrBox->roi.y));
                        break;
                    }
                }

                //compute the ratio of size, if the ratio is less than threshold, filter this box out.
                double ratio = prevBoxSize/currBoxSize;
                if(ratio>1)
                    ratio = 1/ratio;
                if(ratio<threshold_ratio)
                    continue;

                if(distance>distance_threshold)
                    continue;

                max=it->second;
                maxId=it->first;
            }
        }

        int threshold = 5; //a successfull match must have matches more than threshold
        //if find matches, put it into the result map.
        if(max > threshold)
        {
            bbBestMatches.insert(std::pair<int, int>(itPrevBox->boxID,maxId));
            //std::cout<<"Match: Previous Box ID: "<<itPrevBox->boxID<<",Size: "<<itPrevBox->roi<<", Current Box ID: "<<maxId<<",Size: "<<currFrame.boundingBoxes[maxId].roi<<endl;
        }
    }
}
