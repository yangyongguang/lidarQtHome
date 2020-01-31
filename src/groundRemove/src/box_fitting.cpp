#include <array>
#include <random>
#include <opencv2/opencv.hpp>
#include "box_fitting.h"

using namespace std;
using namespace cv;

// float picScale = 30;
float roiM = 120.0f;
float picScale = 900/roiM;
int ramPoints = 80;
int lSlopeDist = 1.5; // 大于 1.5 米的对角线车
// int lnumPoints = 200;
int lnumPoints = 80;

float sensorHeight = 0.1;
// float tHeightMin = 1.2;
float tHeightMin = 0.8;
float tHeightMax = 2.6;
// float tWidthMin = 0.5;
// float tWidthMin = 0.4;
float tWidthMin = 0.25;
float tWidthMax = 3.5;
float tLenMin = 0.5;
float tLenMax = 14.0;
float tAreaMax = 20.0;
float tRatioMin = 1.2;
float tRatioMax = 5.0;
float minLenRatio = 3.0;
float tPtPerM3 = 8;


void getPointsInPcFrame(Point2f rectPoints[], vector<Point2f>& pcPoints, int offsetX, int offsetY)
{
    // loop 4 rect points
    for (int pointI = 0; pointI < 4; pointI++){
        float picX = rectPoints[pointI].x;
        float picY = rectPoints[pointI].y;
        // reverse offset
        float rOffsetX = picX - offsetX;
        float rOffsetY = picY - offsetY;
        // reverse from image coordinate to eucledian coordinate
        float rX = rOffsetX;
        float rY = picScale*roiM - rOffsetY;
        // reverse to 30mx30m scale
        float rmX = rX/picScale;
        float rmY = rY/picScale;
        // reverse from (0 < x,y < 30) to (-15 < x,y < 15)
        float pcX = rmX - roiM/2;
        float pcY = rmY - roiM/2;
        Point2f point(pcX, pcY);
        pcPoints[pointI] = point;
    }
}

bool ruleBasedFilter(vector<Point2f> pcPoints, float maxZ, int numPoints)
{
    bool isPromising = false;
    //minnimam points thresh
    if(numPoints < 10) return isPromising;
    // length is longest side of the rectangle while width is the shorter side.
    float width, length, height, area, ratio;
    float mass;

    float x1 = pcPoints[0].x;
    float y1 = pcPoints[0].y;
    float x2 = pcPoints[1].x;
    float y2 = pcPoints[1].y;
    float x3 = pcPoints[2].x;
    float y3 = pcPoints[2].y;

    float dist1 = sqrt((x1-x2)*(x1-x2)+ (y1-y2)*(y1-y2));
    float dist2 = sqrt((x3-x2)*(x3-x2)+ (y3-y2)*(y3-y2));
    if(dist1 > dist2)
    {
        length = dist1;
        width = dist2;
    }
    else
    {
        length = dist2;
        width = dist1;
    }
    // assuming ground = sensor height
    height = maxZ + sensorHeight;
    // assuming right angle
    area = dist1*dist2;
    mass = area*height;
    ratio = length/width;

    //start rule based filtering
#if 0 //zhanghm 20180607 修改
    if(height > tHeightMin && height < tHeightMax){
        if(width > tWidthMin && width < tWidthMax){
            if(length > tLenMin && length < tLenMax){
                if(area < tAreaMax){
                    if(numPoints > mass*tPtPerM3){
                        if(length > minLenRatio){
                            if(ratio > tRatioMin && ratio < tRatioMax){
                                isPromising = true;
                                return isPromising;
                            }
                        }
                        else{
                            isPromising = true;
                            return isPromising;
                        }
                    }
                }
            }
        }
    }
    else 
        return isPromising;
#endif

    if(height > 0.5 && height < tHeightMax)
    {
      if(width > tWidthMin && width < tWidthMax)
      {
        if(length > tLenMin && length < tLenMax)
        {
          if(area < tAreaMax){
            if(ratio>1.1 && ratio < 4)
            {
              isPromising = true;
              return isPromising;
            }
            else
            {
              return false;
            }
          }
        }
      }
    }
    else
        return isPromising;
}

void getBoundingBox(const vector<Cloud::Ptr> & clusteredPoints,
                    vector<Cloud::Ptr>& bbPoints)
{
    for (size_t iCluster = 0; iCluster < clusteredPoints.size(); iCluster++)
    {//遍历每个物体
      // Check lidar points number
      if(clusteredPoints[iCluster]->size() == 0)
        continue;

        Mat m (picScale*roiM, picScale*roiM, CV_8UC1, Scalar(0));
        float initPX = (*clusteredPoints[iCluster])[0].x() + roiM/2;
        float initPY = (*clusteredPoints[iCluster])[0].y() + roiM/2;
        int initX = floor(initPX*picScale);
        int initY = floor(initPY*picScale);
        int initPicX = initX;
        int initPicY = picScale*roiM - initY;//图像像素坐标
        int offsetInitX = roiM*picScale/2 - initPicX;
        int offsetInitY = roiM*picScale/2 - initPicY;

        int numPoints = (*clusteredPoints[iCluster]).size();
        vector<Point> pointVec(numPoints);
        vector<Point2f> pcPoints(4);
        float minMx, minMy, maxMx, maxMy;
        float minM = 999; float maxM = -999; float maxZ = -99;
        // for center of gravity
        float sumX = 0; float sumY = 0;
        for (size_t iPoint = 0; iPoint < (*clusteredPoints[iCluster]).size(); iPoint++)
        {
            //遍历某个点云簇中的所有点
            float pX = (*clusteredPoints[iCluster])[iPoint].x();
            float pY = (*clusteredPoints[iCluster])[iPoint].y();
            float pZ = (*clusteredPoints[iCluster])[iPoint].z();
            // cast (-15 < x,y < 15) into (0 < x,y < 30)
            float roiX = pX + roiM/2;
            float roiY = pY + roiM/2;
            // cast 30mx30m into 900x900 scale
            int x = floor(roiX*picScale);
            int y = floor(roiY*picScale);
            // cast into image coordinate
            int picX = x;
            int picY = picScale*roiM - y;
            // offset so that the object would be locate at the center
            int offsetX = picX + offsetInitX;
            int offsetY = picY + offsetInitY;
            m.at<uchar>(offsetY, offsetX) = 255;
            pointVec[iPoint] = Point(offsetX, offsetY);
            // calculate min and max slope
            float m = pY/pX;
            if(m < minM) 
            {
                minM = m;
                minMx = pX;
                minMy = pY;
            }
            if(m > maxM) 
            {
                maxM = m;
                maxMx = pX;
                maxMy = pY;
            }

            //get maxZ
            if(pZ > maxZ) maxZ = pZ;

            sumX += offsetX;
            sumY += offsetY; 

        }
        // L shape fitting parameters
        float xDist = maxMx - minMx;
        float yDist = maxMy - minMy;
        float slopeDist = sqrt(xDist*xDist + yDist*yDist);//最大最小斜率对应的两点之间的距离
        float slope = (maxMy - minMy)/(maxMx - minMx);

        // random variable
        mt19937_64 mt(0);
        uniform_int_distribution<> randPoints(0, numPoints-1);

        // start l shape fitting for car like object
        // lSlopeDist = 30, lnumPoints = 300
        if(slopeDist > lSlopeDist && numPoints > lnumPoints)
        {
            float maxDist = 0;
            float maxDx, maxDy;

            // 80 random points, get max distance
            for(int i = 0; i < ramPoints; i++)
            {
                size_t pInd = randPoints(mt);
                assert(pInd >= 0 && pInd < (*clusteredPoints[iCluster]).size());
                float xI = (*clusteredPoints[iCluster])[pInd].x();
                float yI = (*clusteredPoints[iCluster])[pInd].y();

                // from equation of distance between line and point
                float dist = abs(slope*xI-1*yI+maxMy-slope*maxMx)/sqrt(slope*slope + 1);
                if(dist > maxDist) 
                {
                    maxDist = dist;
                    maxDx = xI;
                    maxDy = yI;
                }
            }

            // for center of gravity
            // maxDx = sumX/clusteredPoints[iCluster].size();
            // maxDy = sumY/clusteredPoints[iCluster].size();

            // vector adding
            float maxMvecX = maxMx - maxDx;
            float maxMvecY = maxMy - maxDy;
            float minMvecX = minMx - maxDx;
            float minMvecY = minMy - maxDy;
            float lastX = maxDx + maxMvecX + minMvecX;
            float lastY = maxDy + maxMvecY + minMvecY;

            pcPoints[0] = Point2f(minMx, minMy);
            pcPoints[1] = Point2f(maxDx, maxDy);
            pcPoints[2] = Point2f(maxMx, maxMy);
            pcPoints[3] = Point2f(lastX, lastY);
            // bool isPromising = ruleBasedFilter(pcPoints, maxZ, numPoints);
            // if(!isPromising) 
            //     continue;
        }
        else
        {
            //MAR fitting
            RotatedRect rectInfo = minAreaRect(pointVec);
            Point2f rectPoints[4]; 
            rectInfo.points(rectPoints);
            // covert points back to lidar coordinate
            getPointsInPcFrame(rectPoints, pcPoints, offsetInitX, offsetInitY);
        }

        // make pcl cloud for 3d bounding box
        Cloud::Ptr oneBboxPtr(new Cloud);
        for(int pclH = 0; pclH < 2; pclH++)
        {//底面四个点,上面四个点
            for(int pclP = 0; pclP < 4; pclP++)
            {
                point o;
                o.x() = pcPoints[pclP].x;
                o.y() = pcPoints[pclP].y;
                if(pclH == 0) 
                    o.z() = -1.73;//车体坐标系下点云,地面高度估计为0.1m
                else 
                    o.z() = maxZ;
                oneBboxPtr->emplace_back(o);
            }
        }
        bbPoints.emplace_back(oneBboxPtr);
    }
}

void getBBox(const vector<Cloud::Ptr> & clusteredPoints,
                    vector<Cloud::Ptr>& bbPoints)
{
    for (size_t iCluster = 0; iCluster < clusteredPoints.size(); iCluster++)
    {   
        //遍历每个物体
        // Check lidar points number
        if(clusteredPoints[iCluster]->size() == 0)
            continue;

        int numPoints = (*clusteredPoints[iCluster]).size();
        // vector<Point> pointVec(numPoints);
        vector<cv::Point2f> pointVec(numPoints);
        vector<Point2f> pcPoints(4);
        float minMx, minMy, maxMx, maxMy;
        float minM = 999; float maxM = -999; float maxZ = -99;
        // for center of gravity
        // float sumX = 0; float sumY = 0;
        for (size_t iPoint = 0; iPoint < (*clusteredPoints[iCluster]).size(); iPoint++)
        {
            //遍历某个点云簇中的所有点
            float pX = (*clusteredPoints[iCluster])[iPoint].x();
            float pY = (*clusteredPoints[iCluster])[iPoint].y();
            float pZ = (*clusteredPoints[iCluster])[iPoint].z();
            // cast (-15 < x,y < 15) into (0 < x,y < 30)          
            // float m = pY/pX;
            pointVec[iPoint].x = pX;
            pointVec[iPoint].y = pY;

            float m = atan2(pY, pX);
            if(m < minM) 
            {
                minM = m;
                minMx = pX;
                minMy = pY;
            }
            if(m > maxM) 
            {
                maxM = m;
                maxMx = pX;
                maxMy = pY;
            }

            //get maxZ
            if(pZ > maxZ) maxZ = pZ;

            // sumX += offsetX;
            // sumY += offsetY; 

        }
        // L shape fitting parameters
        float xDist = maxMx - minMx;
        float yDist = maxMy - minMy;
        float slopeDist = sqrt(xDist*xDist + yDist*yDist);//最大最小斜率对应的两点之间的距离
        float slope = (maxMy - minMy)/(maxMx - minMx);

        // random variable
        mt19937_64 mt(0);
        uniform_int_distribution<> randPoints(0, numPoints-1);

        // start l shape fitting for car like object
        // lSlopeDist = 30, lnumPoints = 300
        if(slopeDist > lSlopeDist && numPoints > lnumPoints)
        {
            float maxDist = 0;
            float maxDx, maxDy;

            // 80 random points, get max distance
            for(int i = 0; i < ramPoints; i++)
            {
                size_t pInd = randPoints(mt);
                assert(pInd >= 0 && pInd < (*clusteredPoints[iCluster]).size());
                float xI = (*clusteredPoints[iCluster])[pInd].x();
                float yI = (*clusteredPoints[iCluster])[pInd].y();

                // from equation of distance between line and point
                float dist = abs(slope*xI-1*yI+maxMy-slope*maxMx)/sqrt(slope*slope + 1);
                if(dist > maxDist) 
                {
                    maxDist = dist;
                    maxDx = xI;
                    maxDy = yI;
                }
            }

            // for center of gravity
            // maxDx = sumX/clusteredPoints[iCluster].size();
            // maxDy = sumY/clusteredPoints[iCluster].size();

            // vector adding
            float maxMvecX = maxMx - maxDx;
            float maxMvecY = maxMy - maxDy;
            float minMvecX = minMx - maxDx;
            float minMvecY = minMy - maxDy;
            float lastX = maxDx + maxMvecX + minMvecX;
            float lastY = maxDy + maxMvecY + minMvecY;

            pcPoints[0] = Point2f(minMx, minMy);
            pcPoints[1] = Point2f(maxDx, maxDy);
            pcPoints[2] = Point2f(maxMx, maxMy);
            pcPoints[3] = Point2f(lastX, lastY);
            // bool isPromising = ruleBasedFilter(pcPoints, maxZ, numPoints);
            // if(!isPromising) 
            //     continue;
        }
        else
        {
            //MAR fitting
            RotatedRect rectInfo = minAreaRect(pointVec);
            Point2f rectPoints[4]; 
            rectInfo.points(rectPoints);
            // covert points back to lidar coordinate
            for (int idx = 0; idx < 4; ++idx)
            {
                pcPoints[idx].x = rectPoints[idx].x;
                pcPoints[idx].y = rectPoints[idx].y;
            }
        }

        // make pcl cloud for 3d bounding box
        Cloud::Ptr oneBboxPtr(new Cloud);
        for(int pclH = 0; pclH < 2; pclH++)
        {//底面四个点,上面四个点
            for(int pclP = 0; pclP < 4; pclP++)
            {
                point o;
                o.x() = pcPoints[pclP].x;
                o.y() = pcPoints[pclP].y;
                if(pclH == 0) 
                    o.z() = -1.73;//车体坐标系下点云,地面高度估计为0.1m
                else 
                    o.z() = maxZ;
                oneBboxPtr->emplace_back(o);
            }
        }
        bbPoints.emplace_back(oneBboxPtr);
    }
}


// std::vector<Vertex> CloudToVertexs(const Cloud::Ptr & cloud, float & minZ, float & maxZ)
void CloudToVertexs(const Cloud::Ptr & cloud, 
            std::vector<Vertex> & bottomVertexs
            // std::vector<Vertex> & topVertexs
            )
{
    // std::vector<Vertex> res(cloud->size());
    bottomVertexs.clear();
    // topVertexs.clear();
    float & maxZ = cloud->maxZ;
    float & minZ = cloud->minZ;

    // float split = minZ + (maxZ - minZ) * 2 / 3;
    for (size_t idx = 0; idx < cloud->size(); ++idx)
    {
        Vertex vet;
        vet.x = (*cloud)[idx].x();
        vet.y = (*cloud)[idx].y();
        float & z = (*cloud)[idx].z();
        // if ((*cloud)[idx].z() < minZ)
        //     minZ = (*cloud)[idx].z();
        // if ((*cloud)[idx].z() > maxZ)
        //     maxZ = (*cloud)[idx].z();

        // res.emplace_back(vet);
        // res[idx] = vet;
        // 分割为上下俩个部分， 为了防止车耳朵造成偏差
        // if (z < split)
            bottomVertexs.emplace_back(vet);
        // else
        //     topVertexs.emplace_back(vet);        
    }

    // assert(cloud->size() == res.size());
    // fprintf(stderr, "point info:\n");
    // for (size_t idx = 0; idx < cloud->size(); ++idx)
    // {
    //     fprintf(stderr, "[%f, %f] <--> [%f, %f]\n", 
    //                 (*cloud)[idx].x(),
    //                 (*cloud)[idx].y(),
    //                 res[idx].x,
    //                 res[idx].y);
    // }
}


void getOrientedBBox(const vector<Cloud::Ptr> & clusteredPoints,
                    vector<Cloud::Ptr> & bbPoints)
{
    // 遍历所有 cloud
    // fprintf(stderr, "number of clusters %d\n", clusteredPoints.size());
    // fprintf(stderr, "------------------3.1\n");
    for (size_t idx = 0; idx < clusteredPoints.size(); ++idx)
    {
        // float minZ = 1000, maxZ = -1000;
        vector<Point2f> pcPoints(4);
        if (clusteredPoints[idx]->size() > 40)       
        { 
            // vector<Vertex> vertexs = CloudToVertexs(clusteredPoints[idx], minZ, maxZ);
            vector<Vertex> bottomVertexs;//, topVertexs;
            CloudToVertexs(clusteredPoints[idx], bottomVertexs);

            // if (bottomVertexs.size() > 20);
            // {
                // fprintf(stderr, "points about : %d\n", bottomVertexs.size());
                // fprintf(stderr, "------------------points size : %d\n", vertexs.size());
                // vector<Vertex> vertexs = generatePoints(100);
                // ConvexHull convex_hull(vertexs);
                std::shared_ptr<ConvexHull> convexHullBottom(new ConvexHull(bottomVertexs));
                vector<Vertex> pointsHullBottom = convexHullBottom->vertices_;
                pointsHullBottom = convexHullBottom->toRec1(); 

                // std::shared_ptr<ConvexHull> convexHullTop(new ConvexHull(topVertexs));
                // vector<Vertex> pointHullTop = convexHullTop->vertices_;
                // pointHullTop = convexHullTop->toRec1();

                // assert(points_hull.size() == 4);
                // for (size_t idx = 0; idx < points_hull.size(); ++idx)
                // {
                //     fprintf(stderr, "[%f, %f]\n", points_hull[idx].x, points_hull[idx].y);
                // }

                for (size_t pcIdx = 0; pcIdx < pcPoints.size(); ++pcIdx)
                {
                    pcPoints[pcIdx].x = pointsHullBottom[pcIdx].x;
                    pcPoints[pcIdx].y = pointsHullBottom[pcIdx].y;
                }
            // }
        }
        else
        {
            //MAR fitting
            Cloud & tmpCloud = (*clusteredPoints[idx]);
            vector<Point2f> pointVec(tmpCloud.size());

            for (int i = 0; i < tmpCloud.size(); ++i)
            {
                pointVec[i].x = tmpCloud[i].x();
                pointVec[i].y = tmpCloud[i].y();
                // if (tmpCloud[i].z() < minZ)
                //     minZ = tmpCloud[i].z();
                // if (tmpCloud[i].z() > maxZ)
                //     maxZ = tmpCloud[i].z();
            }

            RotatedRect rectInfo = minAreaRect(pointVec);
            Point2f rectPoints[4]; 
            rectInfo.points(rectPoints);
            // covert points back to lidar coordinate
            for (int j = 0; j < 4; ++j)
            {
                pcPoints[j].x = rectPoints[j].x;
                pcPoints[j].y = rectPoints[j].y;
            }
        }
        
        Cloud::Ptr oneBboxPtr(new Cloud);
        for(int pclH = 0; pclH < 2; pclH++)
        {//底面四个点,上面四个点
            for(int pclP = 0; pclP < 4; pclP++)
            {
                point o;
                o.x() = pcPoints[pclP].x;
                o.y() = pcPoints[pclP].y;
                if(pclH == 0) 
                    o.z() = clusteredPoints[idx]->minZ;//车体坐标系下点云,地面高度估计为0.1m
                    // o.z() = -1.73;
                else 
                    o.z() = clusteredPoints[idx]->maxZ;
                    // o.z() = 2;
                oneBboxPtr->emplace_back(o);
            }
        }
        bbPoints.emplace_back(oneBboxPtr);     
    }
}
