#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QMainWindow>
#include <iostream>
#include "groundRemove/include/param.h"
#include <QColor>
#include <QDebug>
#include <QImage>
#include <QPixmap>
#include <QString>
#include <QFileDialog>
#include <QTextCursor>
#include <QComboBox>

#include "utils.h"
#include "groundRemove/include/cloud.h"

#include <QKeyEvent>

#include "viewer.h"
#include "base_viewer_widget.h"

// groundRemove 的代码
#include "groundRemove/include/groundRemove.h"
#include "groundRemove/include/component_clustering.h"
#include "groundRemove/include/box_fitting.h"

#include <QTextEdit>

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPixmap>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <QLabel>
#include <QDockWidget>
#include <unordered_map>

#include <QDesktopWidget>
#include <QRect>

namespace Ui {
class MainWindow;
}

// class Widget : public QWidget
class MainWindow : public BaseViewerWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    // log 光标移动到最后
    void moveCursorToEnd();
    // std::vector<std::string> fileNameBin;
    // std::vector<std::string> fileNameImage;
    Cloud cloud;

    // Viewer *_viewer = nullptr;
    // groundRemove 的参数
    GroundSegmentationParams params_groundRemove;
    float angle_threshold;
    bool depthImagefilter;
    size_t girdImageResize;

    std::unordered_map<int, int> bboxToCluster;
protected:
    void keyPressEvent(QKeyEvent *event) override;
    virtual void resizeEvent(QResizeEvent *event) override;

private slots:
    void onOpenFolderToRead();
    /**
     *  @param 
     *  @brief 播放和暂停
     */
    void onPlayClouds();
    void onSliderMovedTo(int cloud_number);
    void onReset();

    // 重新显示， 当改变形式的时候
    void onUpdateShow();
    void onUpdateShow(int num);
    void onUpdateShow(bool isFullScreen);
    void onUpdate();

    // 改变参数类型， 为了调整参数
    void onParamSet();

    // 清除选择的 ID
    void onClearSelection();

private:
    // Ui::Widget *ui;
    std::unique_ptr<Ui::MainWindow> ui;

    params _params;
    std::vector<std::string> _file_names_velo;
    std::vector<std::string> _file_names_img;
    
    Cloud::Ptr _cloud;

    // 是播放还是暂停 
    bool playCloud;

    // 当前数据索引
    size_t curr_data_idx;

    // 总计所少个数据
    size_t numData;

    // new widget
    QTextEdit *infoTextEdit = nullptr;

    // 显示图像
    // std::unique_ptr<QGraphicsScene> _scene = nullptr;
    // std::unique_ptr<QGraphicsView> _graphView = nullptr;

    // 可供选择的 对象
    QComboBox *ObjSelectCB;
    QDockWidget *dock_Image;
    QDockWidget *dock_cluster_image;
    QDockWidget *dockshow_depth_image;
    QDockWidget *dockshow_depth_image2;

    QLabel *imgLabel;
    QLabel *cluster_image;
    QLabel *depth_image;
    QLabel *depth_image2;
    
    // 新建一个窗口试试

    // 创建要显示的 gird rect
    // std::vector<Rect2D> rect2DVec;
public:
    bool isFullScreen;
    QRect subWindSize;
signals:
    void fullScreenSignals();
};
#endif // WIDGET_H
