#ifndef SRC_DRAWABLECLOUD_H
#define SRC_DRAWABLECLOUD_H
#include "groundRemove/include/param.h"

#include "drawable.h"
#include <memory>

// 绘制点云
class DrawableCloud : public Drawable
{
public:
    using Ptr = std::shared_ptr<DrawableCloud>;
    explicit DrawableCloud(const Cloud::ConstPtr& cloud,
                            const Eigen::Vector3f& color = Eigen::Vector3f::Ones(),
                            const GLfloat & pointSize = 1.0,
                            const int numCluster = -1)
            : _cloud_ptr{cloud}, _color{color}, _pointSize(pointSize), _numCluster(numCluster){}
    
    void Draw() const override;

    static DrawableCloud::Ptr FromCloud(
      const Cloud::ConstPtr& cloud,
      const Eigen::Vector3f& color = Eigen::Vector3f::Ones(),
      const GLfloat & pointSize = 1.0,
      const int numCluster = -1);

    ~DrawableCloud() override {}

private:
    int _numCluster = -1;
    GLfloat _pointSize = 1.0;
    Cloud::ConstPtr _cloud_ptr = nullptr;
    Eigen::Vector3f _color = Eigen::Vector3f::Ones();
    params _param = params();
};


class DrawableRect : public Drawable
{
public:
    using Ptr = std::shared_ptr<DrawableRect>;
    explicit DrawableRect(const std::vector<Rect2D> & posVec, const float & z = 0);
    void Draw() const override;

    ~DrawableRect() override {}    
    static DrawableRect::Prt FromRectVec(const std::vector<Rect2D> & posVec, const float & z = 0);

private:
    std::vector<Rect2D> rectPosVec;
    float hightToGround;
};

// 可视化 BBox
class DrawableBBox: public Drawable
{
public:
    using Ptr = std::shared_ptr<DrawableBBox>;
    explicit DrawableBBox(const std::vector<Cloud::Ptr> & posVec, bool drawZAxis = true, int color = 0);
    
    void Draw() const override;

    ~DrawableBBox() override {}    
    static DrawableBBox::Prt FromCloud(
                        const std::vector<Cloud::Ptr> & posVec, 
                        bool drawZAxis = true,
                        int color = 0);

private:
    std::vector<Cloud::Ptr> rectPosVec;
    bool _drawZAxis;
    int _color;
};

#endif