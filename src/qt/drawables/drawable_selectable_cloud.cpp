#include "drawable_selectable_cloud.h"

DrawSelectAbleCloud::DrawSelectAbleCloud(const Cloud::ConstPtr& cloud,
                            const Eigen::Vector3f& color,
                            const GLfloat & pointSize): 
                            _cloud_ptr{cloud}, 
                            _color{color},
                            _pointSize(_pointSize)
{
    objects.clear();
    // 构造对象
    for (const auto & point : _cloud_ptr->points())
    {
        auto real_point = point.AsEigenVector();
        // 注意 make_shared 的用法
        auto obj = std::make_shared<Object>();
        obj->frame.setPosition(real_point.x(), real_point.y(), real_point.z());
        objects.push_back(obj);
    }
}

// void DrawSelectAbleCloud::DrawSeletable() const
// {
//     // 写入带名字的对象
//     for (size_t idx = 0; idx < _objects.size(); ++idx)
//     {
//         glPushName(idx);
//         _objects.at(idx)->draw();
//         glPopName();
//     }
// }

void DrawSelectAbleCloud::Draw() const
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
    glColor3f(_color[0], _color[1], _color[2]);    
    // fprintf(stderr, "there has about %ld points\n", _cloud_ptr->size());
    for (const auto & point : _cloud_ptr->points())
    {
        auto real_point = point.AsEigenVector();
        // fprintf(stderr, "(%f, %f, %f)\n", real_point.x(), real_point.y(), real_point.z());
        glVertex3f(real_point.x(), real_point.y(), real_point.z());
    }
    glEnd();
    glPopMatrix();    
}

DrawSelectAbleCloud::Ptr DrawSelectAbleCloud::FromCloud(const Cloud::ConstPtr& cloud,
                                            const Eigen::Vector3f& color,
                                            const GLfloat & pointSize)
{
    return std::make_shared<DrawSelectAbleCloud>(DrawSelectAbleCloud(cloud, color, pointSize));
}

