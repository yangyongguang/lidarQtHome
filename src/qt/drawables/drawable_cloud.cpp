#include "drawable_cloud.h"
#include <math.h>

void DrawableCloud::Draw() const
{
    // fprintf(stderr, "DrawableCloud::Draw()\n");
    if (!_cloud_ptr)
    {
        throw std::runtime_error("DrawableCloud has no cloud to draw.");
    }

    // fprintf(stderr, "DrawableCloud::Draw() after _cloud_ptr check\n");

    glPushMatrix();
    glPointSize(_pointSize);
    glBegin(GL_POINTS);
    // glColor3f(_color[0], _color[1], _color[2]);    
    // fprintf(stderr, "there has about %ld points\n", _cloud_ptr->size());
    // bool multiColor = ((*_cloud_ptr)[0].classID != -1);
    bool multiColor = (_numCluster != -1);
    for (const auto & point : _cloud_ptr->points())
    {
        if (!multiColor)
        {
            glColor3f(_color[0], _color[1], _color[2]);
        }
        else
        {
            int classID = point.classID % _param.RANDOM_COLORS.size();
            // fprintf(stderr, "classID[%d]\n", classID);
            glColor3f((float)_param.RANDOM_COLORS[classID][0] / 255,
                      (float)_param.RANDOM_COLORS[classID][1] / 255,
                      (float)_param.RANDOM_COLORS[classID][2] / 255
                    );
        }        
        auto real_point = point.AsEigenVector();
        // fprintf(stderr, "(%f, %f, %f)\n", real_point.x(), real_point.y(), real_point.z());
        glVertex3f(real_point.x(), real_point.y(), real_point.z());
    }
    glEnd();
    glPopMatrix();    
}

DrawableCloud::Ptr DrawableCloud::FromCloud(const Cloud::ConstPtr& cloud,
                                            const Eigen::Vector3f& color,
                                            const GLfloat & pointSize,
                                            const int numCluster)
{
    return std::make_shared<DrawableCloud>(DrawableCloud(cloud, color, pointSize, numCluster));
}


// 矩形繪制
DrawableRect::DrawableRect(const std::vector<Rect2D> & posVec, const float & z)
{
    rectPosVec = posVec;
    hightToGround = z;
}

void DrawableRect::Draw() const
{
    // fprintf(stderr, "void DrawableRect::Draw() const\n");

    glPushMatrix();
    // fprintf(stderr, "rectPosVec size : %d\n", rectPosVec.size());
    glLineWidth(0.5f);
    glColor3f(0.8f, 0.8f, 0.8f);
    for (int idx = 0; idx < rectPosVec.size(); ++idx)
    {
        Rect2D rect = rectPosVec[idx];
        float size = rect.getSize();
        float x1 = rect.x() - size / 2;
        float y1 = rect.y() - size / 2;

        float x2 = rect.x() + size / 2;
        float y2 = rect.y() + size / 2;

        // glRectf(x1, y1, x2, y2);
        float hight = 0.0f;
        glBegin(GL_LINE_STRIP);
        glVertex3f(x1, y1, hight);
        glVertex3f(x1, y2, hight);
        glVertex3f(x2, y2, hight);
        glVertex3f(x2, y1, hight);
        glVertex3f(x1, y1, hight);
        glEnd();
        // fprintf(stderr, "[%.3f] [%.3f, %.3f, %.3f, %.3f]\n", size, x1, y1, x2, y2);
    }

    glPopMatrix();
}

DrawableRect::Prt DrawableRect::FromRectVec(const std::vector<Rect2D> & posVec, const float & z)
{
    return std::make_shared<DrawableRect>(DrawableRect(posVec, z));
}

// -------------------------------------------------------------------------------------
DrawableBBox::DrawableBBox(const std::vector<Cloud::Ptr> & posVec, bool drawZAxis, int color)
{
    rectPosVec.assign(posVec.begin(), posVec.end());
    _drawZAxis = drawZAxis;
    _color = color;
}

void DrawableBBox::Draw() const
{
    glPushMatrix();
    glLineWidth(2.0f);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //指定混合函数
    // glEnable(GL_BLEND);
    // glDisable(GL_LIGHTING);
    for (int iBBox = 0; iBBox < rectPosVec.size(); ++iBBox)
    {
        Cloud bboxPt = (*rectPosVec[iBBox]);
        glColor3f(0.4112f, 0.412f, 0.412f);

        glBegin(GL_QUAD_STRIP);
        std::array<int, 4> bottom = {1, 2, 0, 3};
        for (int idx = 0; idx < 4; ++idx)
        {
            glVertex3f(bboxPt[bottom[idx]].x(), 
                        bboxPt[bottom[idx]].y(), 
                        bboxPt[bottom[idx]].z());
        }
        // glVertex3f(bboxPt[0].x(), bboxPt[0].y(), bboxPt[0].z());
        glEnd();
        if (_drawZAxis)
        {
            if (_color == 0)
                glColor3f(0.80f, 0.745f, 0.448f);
            else
                glColor3f(1.0f, 0.0f, 0.0f);
                
            glBegin(GL_LINE_STRIP);
            for (int idx = 4; idx < 8;++idx)
            {
                glVertex3f(bboxPt[idx].x(), bboxPt[idx].y(), bboxPt[idx].z());
            } 
            glVertex3f(bboxPt[4].x(), bboxPt[4].y(), bboxPt[4].z());           
            glEnd();

            // 绘制四边
            glBegin(GL_LINES);
            glVertex3f(bboxPt[0].x(), bboxPt[0].y(), bboxPt[0].z());
            glVertex3f(bboxPt[4].x(), bboxPt[4].y(), bboxPt[4].z());

            glVertex3f(bboxPt[1].x(), bboxPt[1].y(), bboxPt[1].z());
            glVertex3f(bboxPt[5].x(), bboxPt[5].y(), bboxPt[5].z());

            glVertex3f(bboxPt[2].x(), bboxPt[2].y(), bboxPt[2].z());
            glVertex3f(bboxPt[6].x(), bboxPt[6].y(), bboxPt[6].z());

            glVertex3f(bboxPt[3].x(), bboxPt[3].y(), bboxPt[3].z());
            glVertex3f(bboxPt[7].x(), bboxPt[7].y(), bboxPt[7].z());           
            glEnd();
        }

        /*
        glBegin(GL_QUAD_STRIP);
        std::array<int, 4> bottom = {1, 2, 0, 3};

        for (auto it = bottom.begin(); it != bottom.end(); ++it)
        {
            glVertex3f(bboxPt[*it].x(), bboxPt[*it].y(), bboxPt[*it].z());
        }
        glEnd();

        if (_drawZAxis)
        {
            std::array<int, 4> top = {5, 6, 4, 7};
            std::array<int, 8> surround = {0, 4, 3, 7, 2, 6, 1, 5};
            glBegin(GL_QUAD_STRIP);
            for (auto it = top.begin(); it != top.end(); ++it)
            {
                glVertex3f(bboxPt[*it].x(), bboxPt[*it].y(), bboxPt[*it].z());
            }
            glEnd();           

            glBegin(GL_QUAD_STRIP);
            for (auto it = surround.begin(); it != surround.end(); ++it)
            {
                glVertex3f(bboxPt[*it].x(), bboxPt[*it].y(), bboxPt[*it].z());
            }
            glEnd(); 
        }
        */
    }

    // glEnable(GL_LIGHTING);
    // glDisable(GL_BLEND);
}

DrawableBBox::Prt DrawableBBox::FromCloud(
                        const std::vector<Cloud::Ptr> & posVec, 
                        bool drawZAxis,
                        int color)
{
    return std::make_shared<DrawableBBox>(DrawableBBox(posVec, drawZAxis, color));    
}