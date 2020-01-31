#ifndef BOX_FITTING_H
#define BOX_FITTING_H

#include <array>
#include <vector>
#include "cloud.h"
#include "param.h"

#include "box_type.h"
#include "convex_hull.h"

using namespace std;


extern float picScale; // picScale * roiM = 30 * 30
//const float picScale = 30;
extern int ramPoints;
extern int lSlopeDist;
extern int lnumPoints;

extern float tHeightMin;
extern float tHeightMax;
extern float tWidthMin;
extern float tWidthMax;
extern float tLenMin;
extern float tLenMax;
extern float tAreaMax;
extern float tRatioMin;
extern float tRatioMax;
extern float minLenRatio;
extern float tPtPerM3;

// 图像做的
void getBoundingBox(const vector<Cloud::Ptr> & clusteredPoints,
                    vector<Cloud::Ptr> & bbPoints);
// 使用原始点做的
void getBBox(const vector<Cloud::Ptr> & clusteredPoints,
                    vector<Cloud::Ptr> & bbPoints);        
// 论文
// An Orientation Corrected Bounding Box Fit Based on the Convex Hull under Real Time Constraintes
// std::vector<Vertex> CloudToVertexs(const Cloud::Ptr & cloud, float & minZ, float & maxZ);

void CloudToVertexs(const Cloud::Ptr & cloud, 
            std::vector<Vertex> & bottomVertexs//,
            // std::vector<Vertex> & topVertexs
            );

void getOrientedBBox(const vector<Cloud::Ptr> & clusteredPoints,
                    vector<Cloud::Ptr> & bbPoints);    

#endif //MY_PCL_TUTORIAL_BOX_FITTING_H
