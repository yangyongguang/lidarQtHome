#ifndef SRC_QT_UTILS_CLOUD_H
#define SRC_QT_UTILS_CLOUD_H
#include <iostream>

#include <algorithm>
#include <list>
#include <vector>
#include <Eigen/Core>
#include <memory>
#include <opencv2/core/core.hpp>

typedef std::pair<float, float> float2D;
typedef std::vector<float2D> float2DVec;

// 聚类显示的 2D 矩形
class Rect2D
{
public:
    explicit Rect2D(float x, float y, float hight):_x(x), _y(y), _hight(hight){}
    inline float x() {return _x;}
    
    inline float y() {return _y;}

    inline float hight() {return _hight;}

    inline void setSize(const float & size){_size = size;}

    inline float getSize() const{return _size;}

private:
    float _x;
    float _y;

    // hight 为离地高度， 暂时不确定使用最高还是最低高度
    float _hight;

    // 矩形大小
    float _size;
};


class point{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    point() {_point = Eigen::Vector3f::Zero();_intensity = 0;}
    explicit point(float x, float y, float z): _point(x, y, z) {}
    // explicit point(const point & pt);

    virtual ~point(){}

    // 不需要改值的声明
    inline float x() const { return _point.x(); }
    inline float y() const { return _point.y(); }
    inline float z() const { return _point.z(); }
    inline float i() const { return _intensity; }

    // 需要改值的申明
    inline float& x() { return _point.x(); }
    inline float& y() { return _point.y(); }
    inline float& z() { return _point.z(); }
    inline float& i() { return _intensity; }

    inline const Eigen::Vector3f& AsEigenVector() const {return _point; }
    inline Eigen::Vector3f& AsEigenVector() { return _point; }

    void operator=(const point& other);
    void operator=(const Eigen::Vector3f& other);
    bool operator==(const point& other) const;
public:
    int classID = -1;
    float toSensor2D = 0.0f;
    float atan2Val = 0.0f;
    // 是否是 lShapePoint
    int isLShapePoint = 0;
    

private:
    Eigen::Vector3f _point;
    float _intensity;
};

class Cloud{
public:
    Cloud():minZ(999.0f), maxZ(-999.0f), maxAngle(-999.0f), minAngle(999){}
    // explicit Cloud(const Cloud & cloud) {}
    // explicit Cloud(Cloud & cloud) {}
    inline size_t size() const { return _points.size(); }
    inline bool empty() const { return _points.empty(); }
    inline void reserve(size_t size) { _points.reserve(size); }
    inline void push_back(const point& pt) { _points.push_back(pt); }
    inline void emplace_back(const point& pt) {_points.emplace_back(pt);}
    inline point & operator[](int idx) {return _points[idx];}
    inline const point & operator[](int idx) const { return _points[idx];}

    inline const std::vector<point> & points() const {return _points;}
    
    // inline std::vector<point>::iterator begin() {return _points.begin();}
    // inline std::vector<point>::iterator end() {return _points.end();}
    inline std::vector<point>::iterator begin() {return _points.begin();}
    inline std::vector<point>::iterator end() {return _points.end();}

    // std::vector<point> _points;
    inline void clear(){_points.clear();}

    // 根据距离激光雷达的距离对点进行排序， 从近到远
    inline void sort(){std::sort(_points.begin(), _points.end(), 
        [](const point & a, const point & b){return a.toSensor2D < b.toSensor2D;});}

    typedef std::shared_ptr<Cloud> Ptr;
    typedef std::shared_ptr<const Cloud> ConstPtr;
private:
    std::vector<point> _points;
public:
    float minZ;
    float maxZ;

    // 最大与最小角度
    float maxAngle;
    float minAngle;

    // 存储 L-shape 的俩个拐点的坐标的索引
    int minLPoint = -1;
    int maxLPoint = -1;
    
    // 对称点的对数
    size_t numSymPoints = 0;
};
#endif