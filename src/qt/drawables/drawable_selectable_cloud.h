#ifndef SRC_DRAWABLE_SELECABLE_CLOUD_H
#define SRC_DRAWABLE_SELECABLE_CLOUD_H

#include "drawable.h"
#include <memory>
#include "object.h"

class DrawSelectAbleCloud : public Drawable
{
public:
    using Ptr = std::shared_ptr<DrawSelectAbleCloud>;
    DrawSelectAbleCloud(){}
    explicit DrawSelectAbleCloud(const Cloud::ConstPtr& cloud,
                            const Eigen::Vector3f& color = Eigen::Vector3f::Ones(),
                            const GLfloat & pointSize = 1.0);
    
    void Draw() const override;

    void DrawSeletable() const;

    static DrawSelectAbleCloud::Ptr FromCloud(
        const Cloud::ConstPtr& cloud,
        const Eigen::Vector3f& color = Eigen::Vector3f::Ones(),
        const GLfloat & pointSize = 1.0);

    ~DrawSelectAbleCloud() override {}

    std::vector<Object::Ptr> objects;   
private:
    GLfloat _pointSize = 1.0;
    Cloud::ConstPtr _cloud_ptr = nullptr;
    Eigen::Vector3f _color = Eigen::Vector3f::Ones();

};
#endif