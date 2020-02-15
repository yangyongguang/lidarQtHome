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
float lSlopeDist = 1.0; // 大于 1.5 米的对角线车
// int lnumPoints = 200;
// 找到十二个 lShapePoint 就使用我们的算法
int lnumPoints = 12; 

float sensorHeight = 0.1;
// float tHeightMin = 1.2;
float tHeightMin = 0.8;
float tHeightMax = 2.6;
// float tWidthMin = 0.5;
// float tWidthMin = 0.4;
float tWidthMin = 0.25;
float tWidthMax = 5.5;
float tLenMin = 0.5;
float tLenMax = 25.0;
float tAreaMax = 50.0;
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
    if(numPoints < 8) return isPromising;
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
    // assuming right angle
    area = dist1*dist2;
    mass = area * maxZ;
    ratio = length / width;

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

    // if(height > 0.1 && height < tHeightMax)
    // {
    if(width < tWidthMax && length < tLenMax && area < tAreaMax)
    {
        // if(ratio < 6.0f)
        // {
        //     isPromising = true;
        //     return isPromising;
        // }
        // else
        // {
        //     return false;
        // }
        return true;
    }
    // }
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
                    vector<Cloud::Ptr>& bbPoints,
                    Cloud::Ptr & markPoints,
                    std::unordered_map<int, int> & bboxToCluster,
                    const int & debugID)
{
    // fprintf(stderr, "getBBox------------------------\n");
    int numCorrect = 0;
    // BBox 是从 bbPoints 中保留出来的， 所以根据有效的 BBox 的数目可以推出来使用的是那个 ID 跟踪的
    // fprintf(stderr, "current choose iCluster %d\n", debugID);
    for (size_t iCluster = 0; iCluster < clusteredPoints.size(); iCluster++)
    {   
        bool debugBool = false;
        if (iCluster == debugID)
            debugBool = true;
        //遍历每个物体
        // Check lidar points number
        Cloud clusterTmp;
        for (int idx = 0; idx < (*clusteredPoints[iCluster]).size(); ++idx)
        {
            if ((*clusteredPoints[iCluster])[idx].isLShapePoint != 0)
                clusterTmp.emplace_back((*clusteredPoints[iCluster])[idx]);
        }

        // int numPoints = (*clusteredPoints[iCluster]).size();
        int numPoints = clusterTmp.size();
        // 点数太少， minAreRect 不能调用
        if (numPoints == 0)
        {
            // 有被删除过的聚类
            continue;
        }
        // vector<Point> pointVec(numPoints);
        vector<cv::Point2f> pointVec(numPoints);
        vector<Point2f> pcPoints(4);
        float minMx, minMy, maxMx, maxMy;
        float minM = 999; float maxM = -999; float maxZ = -999;
        // for center of gravity
        // float sumX = 0; float sumY = 0;
        // for (size_t iPoint = 0; iPoint < (*clusteredPoints[iCluster]).size(); iPoint++)
        for (size_t iPoint = 0; iPoint < clusterTmp.size(); iPoint++)
        {
            //遍历某个点云簇中的所有点
            // float pX = (*clusteredPoints[iCluster])[iPoint].x();
            // float pY = (*clusteredPoints[iCluster])[iPoint].y();
            // float pZ = (*clusteredPoints[iCluster])[iPoint].z();
            float pX = clusterTmp[iPoint].x();
            float pY = clusterTmp[iPoint].y();
            float pZ = clusterTmp[iPoint].z();
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
        float slopeDist = sqrt(xDist * xDist + yDist * yDist);//最大最小斜率对应的两点之间的距离
        float slope = (maxMy - minMy)/(maxMx - minMx);

        // random variable
        mt19937_64 mt(0);
        uniform_int_distribution<> randPoints(0, numPoints-1);

        // start l shape fitting for car like object
        // lSlopeDist = 30, lnumPoints = 300
        int sizePoint;
        // fprintf(stderr, "slopeDist : %f, numPoints : %d\n", slopeDist, numPoints);
        if(slopeDist > lSlopeDist && numPoints > lnumPoints)
        {
            float maxDist = 0;
            float maxDx, maxDy;
            bool findDPoint = false;

            // 80 random points, get max distance
            if (numPoints < ramPoints)
                sizePoint = numPoints;
            else
                sizePoint = ramPoints;
            
            // float distToOrigin = -slope * maxMx / sqrt(slope * slope + 1);            
            float interceptB = maxMy - slope * maxMx;
            float distToOrigin = interceptB / sqrt(slope * slope + 1);
            for(int i = 0; i < sizePoint; i++)
            {
                size_t pInd = randPoints(mt);
                // assert(pInd >= 0 && pInd < (*clusteredPoints[iCluster]).size());
                // float xI = (*clusteredPoints[iCluster])[pInd].x();
                // float yI = (*clusteredPoints[iCluster])[pInd].y();
                float xI = clusterTmp[pInd].x();
                float yI = clusterTmp[pInd].y();

                // from equation of distance between line and point
                float dist = (slope * xI - yI + interceptB) / sqrt(slope * slope + 1);
                float distAbs = std::abs(dist);

                if (debugBool)
                {
                    fprintf(stderr, "dist: %f, distToOrigin: %f, slope: %f, b:%f\n",
                        dist, distToOrigin, slope, interceptB);
                }

                // 与原点不再同一侧， 不考虑其为 拐点
                if (dist * distToOrigin < 0.0f)
                    continue;
                if(distAbs > maxDist) 
                {
                    findDPoint = true;
                    maxDist = distAbs;
                    maxDx = xI;
                    maxDy = yI;
                }
            }

            if (!findDPoint)
            {
                maxDx = (maxMx + minMx) / 2;
                maxDy = (maxMy + minMy) / 2;
            }

            // for center of gravity
            // maxDx = sumX/clusteredPoints[iCluster].size();
            // maxDy = sumY/clusteredPoints[iCluster].size();

            // vector adding
            float maxMvecX = maxMx - maxDx;
            float maxMvecY = maxMy - maxDy;
            float minMvecX = minMx - maxDx;
            float minMvecY = minMy - maxDy;
            // // 夹角来个边的
            float theatAOBVal = maxMvecX * minMvecX + maxMvecY * minMvecY;
            // float lastX = maxDx + maxMvecX + minMvecX;
            // float lastY = maxDy + maxMvecY + minMvecY;

            // 添加三个可视化点
            
            markPoints->emplace_back(point(maxDx, maxDy, 0.0f));
            markPoints->emplace_back(point(maxMx, maxMy, 0.0f));
            markPoints->emplace_back(point(minMx, minMy, 0.0f));
            // 求 垂线 俩边的点集合
            // float k = (maxMvecY - minMvecY) / (maxMvecX - minMvecX + 1e-6);
            // 这里出现过一个重大的 bug 求斜率时候， 写错了
            // float k_1 = -1 / (k_1 + 1e-6);
            // 正确的写法
            // float k_1 = -1 / (k + 1e-6);
            
            // Vertex vet(k_1, maxDy - k_1 * maxDx);
            // 存储较长边点云集合
            // vector<point> ptSet;
            vector<Point2f> ptSet;//, ptSet2;
            float distA = std::sqrt((maxMy- maxDy) * (maxMy - maxDy) + (maxMx - maxDx) * (maxMx - maxDx));
            float distB = std::sqrt((minMy- maxDy) * (minMy - maxDy) + (minMx - maxDx) * (minMx - maxDx));
            
            // 所成的夹角的值
            float cosThetaAOB = theatAOBVal / (distB * distA);
            // 旋转 30 度角分割 ------------------
            // 第一步 长边到短边的叉成 （x1, y1） x (x2, y2) = (x1 * y2 - x2 * y1) 正， 顺时针， 负， 逆时针
            // 利用 tan (A + B) = (tan A + tan B) / (1 - tan A * tan B)
            // 替换掉新的 斜率
            // a 是长边
            float crossProduct;
            float Xa, Ya, Xb, Yb;
            if (distA < distB)
            {
                
                Xa = minMx - maxDx;
                Ya = minMy - maxDy;

                Xb = maxMx - maxDx;
                Yb = maxMy - maxDy;
            }
            else
            {
                Xa = maxMx - maxDx;
                Ya = maxMy - maxDy;

                Xb = minMx - maxDx;
                Yb = minMy - maxDy;
            }
            
            crossProduct = Xa * Yb - Xb * Ya;
            // 逆时针旋转 加角度
            float k_curr  = Ya / (Xa + 1e-6);
            float k_1; // 旋转 30 度之后的斜率
            // 角度大于 130 接近直线， 选取全部点
            bool moreThanTheta = false;
            if (cosThetaAOB < std::cos(130.0f / 180 * M_PI))
            {
                if (debugBool)
                    fprintf(stderr, "cosTheatAOB %f度, cos(150) : %f, cosThetaAOB %f\n", 
                                std::acos(cosThetaAOB) / M_PI * 180, 
                                std::cos(150.0f / 180 * M_PI),
                                cosThetaAOB);
                k_1 = Yb / (Xb + 1e-6);
                moreThanTheta = true;
            }
            else
            {
                if (crossProduct > 0)
                {
                    k_1 = (0.2679 + k_curr) / (1 - k_curr * 0.2679);
                }
                else
                {
                    // 顺时针旋转， 减去一个角度
                    k_1 = (-0.2679 + k_curr) / (1 - k_curr * (-0.2679));
                }
            }
            
            // 旋转之后的直线
            Vertex vet(k_1, maxDy - k_1 * maxDx);
            if (debugBool)
            {
                fprintf(stderr, "k_curr %f, k_1 %f, crossProduct %f\n", k_curr, k_1, crossProduct);
                fprintf(stderr, "tan(15) :%f,  tan(30)%f\n", tanf(15.0f / 180 * M_PI), tanf(60.0f / 180 * M_PI));
            }
            if (debugBool)
            {
                for (int idx = 1; idx < 30; ++idx)
                {
                    markPoints->emplace_back(point(maxDx + 0.04 * idx, maxDy + 0.04 * idx * k_1, -1.7f));
                }
            }
            // 判断长边            
            //----------------------------------
            // 垂直线段

            // fprintf(stderr, "dist A -- > B(%f, %f)\n", distA, distB);            
            // float zero
            float lessZero;
            if (distA < distB)
            {
                lessZero = k_1 * minMx + vet.y - minMy; 
                markPoints->emplace_back(point((minMx + maxDx) / 2, (minMy + maxDy) / 2, 0.0f));
            }            
            else
            {
                lessZero = k_1 * maxMx + vet.y - maxMy;
                markPoints->emplace_back(point((maxMx + maxDx) / 2, (maxMy + maxDy) / 2, 0.0f));
            }

            // fprintf(stderr, "k : %f\n", k);  
            // 相乘大于零， 说明处于同一侧， 否则不同侧
            // 搜集处于同一侧的点
            for (int i = 0; i < numPoints; ++i)
            {
                float dist = k_1 * clusterTmp[i].x() + vet.y - clusterTmp[i].y();
                if (moreThanTheta)
                {
                    if (debugBool)
                        markPoints->emplace_back(clusterTmp[i]);
                    ptSet.emplace_back(Point2f(clusterTmp[i].x(), clusterTmp[i].y()));
                }
                else if (dist * lessZero >= 0)
                {
                    if (debugBool)
                        fprintf(stderr, "K_1: %f,  dist : %f,  lessZero :%f \n", k_1, dist, lessZero);
                    // ptSet.emplace_back(clusterTmp[i]);
                    if (debugBool)
                        markPoints->emplace_back(clusterTmp[i]);
                    ptSet.emplace_back(Point2f(clusterTmp[i].x(), clusterTmp[i].y()));
                }
                

                
                // else
                // {
                //     ptSet2.emplace_back(Point2f(clusterTmp[i].x(), clusterTmp[i].y()));
                // }
            }

            // 如果这条边的点数太少占据总 lShapePoint 点数的百分之 30 不到， 那么就换一条边

            // fprintf(stderr, "pointSetSize : %d\n", ptSet.size());  

            // 拟合直线 ， 并确定 bbox
            // float rectK = fitLine(ptSet);
            // 使用 RANSAC 与噪声干扰有关系
            float rectK;
            // if (ptSet.size() > ptSet2.size() * 0.4)
            // {
            if (iCluster == debugID)
                fprintf(stderr, "ptSet size : %d\n", ptSet.size());

            if (ptSet.size() >= 10)
                rectK = fitLineRansac(ptSet, 100, 0.06f, debugBool);
            else
                rectK = fitLine(ptSet);
                // fprintf(stderr, "rectK : %f\n", rectK);  
                // 根据方向拟合 bbox 的                
            // }
            // else
            // {
            //     if (ptSet2.size() >= 10)
            //         rectK = fitLineRansac(ptSet2, 100, 0.13);
            //     else
            //         rectK = fitLine(ptSet2);
            //     // fprintf(stderr, "rectK : %f\n", rectK);                  
            // }
            // 根据方向拟合 bbox 的
            fitRect(rectK, *clusteredPoints[iCluster],  pcPoints);
            if (debugBool)
                fprintf(stderr, "\ncurrent rectK : %f\n", rectK);
            // fprintf(stderr, "pcPoints:\n");
            // for (int idx = 0; idx < 4; ++idx)
            //     fprintf(stderr, "(%f, %f)\n", pcPoints[idx].x, pcPoints[idx].y);
            // pcPoints[0] = Point2f(minMx, minMy);
            // pcPoints[1] = Point2f(maxDx, maxDy);
            // pcPoints[2] = Point2f(maxMx, maxMy);
            // pcPoints[3] = Point2f(lastX, lastY);
            // bool isPromising = ruleBasedFilter(pcPoints, maxZ, numPoints);
            // if(!isPromising) 
            //     continue;
        }
        else
        {
            //MAR fitting
            // fprintf(stderr, "minAreaRect point size %d\n", pointVec.size());
            auto & cloud = (*clusteredPoints[iCluster]);
            std::vector<Point2f> pointVec(cloud.size());
            for (int idx = 0; idx < cloud.size(); ++idx)
            {
                pointVec[idx] = Point2f(cloud[idx].x(), cloud[idx].y());
            }

            RotatedRect rectInfo = minAreaRect(pointVec);
            // fprintf(stderr, "minAreaRect finised\n");
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
        
        bool saveIt = ruleBasedFilter(pcPoints, 
            clusteredPoints[iCluster]->maxZ - clusteredPoints[iCluster]->minZ, 
            clusteredPoints[iCluster]->size());
        
        // 不进行保存 bbox
        if (!saveIt)
            continue;   

        Cloud::Ptr oneBboxPtr(new Cloud);
        for(int pclH = 0; pclH < 2; pclH++)
        {//底面四个点,上面四个点
            for(int pclP = 0; pclP < 4; pclP++)
            {
                point o;
                o.x() = pcPoints[pclP].x;
                o.y() = pcPoints[pclP].y;
                if(pclH == 0) 
                    o.z() = clusteredPoints[iCluster]->minZ;//车体坐标系下点云,地面高度估计为0.1m
                else 
                    o.z() = clusteredPoints[iCluster]->maxZ;
                oneBboxPtr->emplace_back(o);
            }
        }
        bbPoints.emplace_back(oneBboxPtr);
        // 正确的 bbox 新增一
        bboxToCluster[numCorrect] = iCluster;
        numCorrect++;
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

// ransac 算法
float fitLineRansac(const vector<cv::Point2f>& clouds, 
                    const int & num_iterations,
                    const float & tolerance,
                    const bool & debug)
{
    if (debug)
        fprintf(stderr, "num %d point fit ransac line \n", clouds.size());
    int max_num_fit = 0;
    Point2f best_plane(0.0f, 0.0f);
    mt19937_64 mt(0);
    uniform_int_distribution<> randPoints(0, clouds.size() - 1);
    for(int i = 0; i < num_iterations; ++i) 
    {
        // -- Get two random points.
        Point2f p;
        // int idx0 = rand() % clouds.size();
        size_t idx0 = randPoints(mt);
        p = clouds[idx0];

        Point2f q;
        // int idx1 = rand() % clouds.size();
        size_t idx1 = randPoints(mt);
        q = clouds[idx1];

        // 选择了相同的点
        if (p.x == q.x && p.y == q.y)
            continue;

        // -- Fit a line a'x = b.
        Point2f a;
        a.x = 1; //初始化其中一个轴为1, 后面归一化后， 就变成斜率向量了
        if(p.y - q.y == 0)
            continue;
        else
            a.y = -(p.x - q.x) / (p.y - q.y);
        // 归一化斜率向量
        float normVal = std::sqrt(a.x * a.x + a.y * a.y);
        a.x = a.x / normVal;
        a.y = a.y / normVal;

        // a.normalize();
        float b = a.dot(p);
        assert(fabs(a.x * a.x + a.y * a.y - 1) < 1e-3);
        assert(fabs(a.dot(p - q)) < 1e-3);
        assert(fabs(a.dot(q) - b) < 1e-3);

        // -- Count how many other points are close to the line.

        int num_fit = 0;
        for(int i = 0; i < clouds.size(); ++i) 
        {
            float adotx = a.dot(clouds[i]);
            if(fabs(adotx - b) <= tolerance)
                ++num_fit;
        }

        if(num_fit > max_num_fit) 
        { 
            max_num_fit = num_fit;
            best_plane = a;
        }
    }
    // return best_plane;
    // 返回 k
    if (debug)
        fprintf(stderr, "res line %f, max_num_fit %d\n", -1 * best_plane.x / best_plane.y, max_num_fit);
    return -1 * best_plane.x / best_plane.y;
}

// 最小二乘法
float fitLine(const std::vector<Point2f> & points)
{
    const unsigned int n_points = points.size();
    Eigen::MatrixXd X(n_points, 2);
    Eigen::VectorXd Y(n_points);

    unsigned int counter = 0;
    for (auto it = points.begin(); it != points.end(); ++it)
    {
        X(counter, 0) = it->x;
        X(counter, 1) = 1;
        Y(counter) = it->y;
        ++counter;
    }

    // y = k * x + b; 返回 k 和 b
    Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);
    return result(0);
}

void fitRect(const float & k, const Cloud & cloud, std::vector<Point2f> & rect)
{
    rect.reserve(4);
    float k_1 = -1 / k;

    // fprintf(stderr, "fitRect:\n");
    // fprintf(stderr, "cloud size : %d\n", cloud.size());
    // 斜率不为无穷时
    if(k_1 != -std::numeric_limits<float>::infinity() && 
        k_1 != std::numeric_limits<float>::infinity() &&
        k !=  -std::numeric_limits<float>::infinity() &&
        k !=  std::numeric_limits<float>::infinity())
    {
        // 初始化
        float b_1, b_2, b_3, b_4;
        b_1 = cloud[0].y() - k * cloud[0].x();
        b_2 = cloud[0].y() - k * cloud[0].x();
        b_3 = cloud[0].y() - k_1 * cloud[0].x();
        b_4 = cloud[0].y() - k_1 * cloud[0].x();

        for (size_t i = 0; i < cloud.size(); ++i)
        {
                  //点在直线的左边
            if(k * cloud[i].x() + b_1 <= cloud[i].y())
            {
                b_1 = cloud[i].y() - k * cloud[i].x();
            }

            if(k * cloud[i].x() + b_2 >= cloud[i].y())
            {
                b_2 = cloud[i].y() - k * cloud[i].x();
            }

            if(k_1 * cloud[i].x() + b_3 <= cloud[i].y())
            {
                b_3 = cloud[i].y() - k_1 * cloud[i].x();
            }

            if(k_1 * cloud[i].x() + b_4 >= cloud[i].y())
            {
                b_4 = cloud[i].y() - k_1 * cloud[i].x();
            }
        }

        // fprintf(stderr, "b_1, b_2, b_3, b_4, k, k_1--> (%f, %f, %f, %f, %f, %f)\n",
        //              b_1, b_2, b_3, b_4, k, k_1);
        Point2f vertex;
        vertex.x = (b_3 - b_1) / (k - k_1);
        vertex.y = vertex.x * k + b_1;
        rect[0] = vertex;

        vertex.x = (b_2 - b_3) / (k_1 - k);
        vertex.y = vertex.x * k + b_2;
        rect[1] = vertex;

        vertex.x = (b_4 - b_2) / (k - k_1);
        vertex.y = vertex.x * k + b_2;
        rect[2] = vertex;
        vertex.x = (b_1 - b_4) / (k_1 - k);
        vertex.y = vertex.x * k + b_1;
        rect[3] = vertex;
    }
    else
    {
        float max_x = -std::numeric_limits<float>::infinity();
        float max_y = -std::numeric_limits<float>::infinity();
        float min_x = std::numeric_limits<float>::infinity();
        float min_y = std::numeric_limits<float>::infinity();

        for(size_t i = 0; i < cloud.size(); i++)
        {
            max_x = (cloud[i].x() > max_x)?cloud[i].x():max_x;
            max_y = (cloud[i].y() > max_y)?cloud[i].y():max_y;
            min_x = (cloud[i].x() < min_x)?cloud[i].x():min_x;
            min_y = (cloud[i].y() < min_y)?cloud[i].y():min_y;
        }
        Point2f vertex;
        vertex.x = max_x;
        vertex.y = max_y;
        rect[0] = vertex;

        vertex.x = max_x;
        vertex.y = min_y;
        rect[1] = vertex;

        vertex.x = min_x;
        vertex.y = min_y;
        rect[2] = vertex;

        vertex.x = min_x;
        vertex.y = max_y;
        rect[3] = vertex;
    }
    
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
        
        bool saveIt = ruleBasedFilter(pcPoints, 
        clusteredPoints[idx]->maxZ - clusteredPoints[idx]->minZ, 
        clusteredPoints[idx]->size());
        
        // 不进行保存 bbox
        if (!saveIt)
            continue;   

        Cloud::Ptr oneBboxPtr(new Cloud);
        for(int pclH = 0; pclH < 2; pclH++)
        {
            // 底面四个点,上面四个点
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
