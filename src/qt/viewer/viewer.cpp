#include "viewer.h"
#include <QDebug>
using std::mutex;
using std::lock_guard;

void Viewer::AddDrawable(Drawable::Prt drawable, std::string name)
{
    // fprintf(stderr,  "%s\n", name.c_str());
    // drawable->Draw();
    // fprintf(stderr, "Viewer::AddDrawable()\n");
    lock_guard<mutex> guard(_cloud_mutex);
    _drawables.push_back(drawable);
    // fprintf(stderr, "drawable size : %d\n", _drawables.size());
}

void Viewer::Clear()
{
    lock_guard<mutex> guard(_cloud_mutex);
    _drawables.clear();
}

void Viewer::draw() 
{
    // 绘制自车
    glColor3f(1.0f, 0.0f, 0.0f);
    glLineWidth(1.0f);
    glBegin(GL_LINE_STRIP);
    glVertex3f(-2.5f, -1.4f, -1.73f);
    glVertex3f(2.5f, -1.4f, -1.73f);
    glVertex3f(2.5f, 1.4f, -1.73f);
    glVertex3f(-2.5f, 1.4f, -1.73f);
    glVertex3f(-2.5f, -1.4f, -1.73f);   
    glEnd();
    //
    // qDebug() << "window clicked .\n" << endl;
    // fprintf(stderr, "Viewer::draw()\n");    
    // fprintf(stderr, "\n\n\ncloud total number : %d\n", drawSelectableCloud.objects.size());
    // fprintf(stderr, "cloud selected id :\n");
    // for (auto it = selection.begin(); it != selection.end(); ++it)
    // {
    //     fprintf(stderr, "%d ", *it);
    // }

    // fprintf(stderr, "bbox selected id :\n");
    // for (auto it = bboxSelection.begin(); it != bboxSelection.end(); ++it)
    // {
    //     fprintf(stderr, "%d ", *it);
    // }

    glPointSize(4);
    glColor3f(0.9f, 0.3f, 0.3f);
    size_t CloudNum = drawSelectableCloud.objects.size();
    size_t BBoxNum = drawSelectableBBox.objects.size();
    if (selection.size() != 0 && CloudNum != 0)
    {
        for (auto it = selection.begin(); it != selection.end(); ++it)
        {
            if (*it < CloudNum)
                drawSelectableCloud.objects[*it]->draw();
            else if (CloudNum <= (*it) && (*it) <= (CloudNum + BBoxNum))
            {
                drawSelectableBBox.objects[*it - CloudNum]->draw();
            }
        }
    }

    lock_guard<mutex> guard(_cloud_mutex);
    for (auto& drawable : _drawables) 
    {
        drawable->Draw();
    }

    // fprintf(stderr, "Viewer::draw()\n");
    // Draws rectangular selection area. Could be done in postDraw() instead.
    if (selectionMode_ != NONE)
        drawSelectionRectangle();
    // // yyg add
    // // Draws selected objects only.
    // glColor3f(0.9f, 0.3f, 0.3f);
    // // for (QList<int>::const_iterator it = selection.begin(),
    // //                                 end = selection.end();
    // //     it != end; ++it)
    // //     objects_.at(*it)->draw();
    // for (auto it = selection.begin(); it != selection.end(); ++it)
    // {
    //     objects_[*it]->draw();
    // }

    // // Draws all the objects. Selected ones are not repainted because of GL depth
    // // test.
    // glColor3f(0.8f, 0.8f, 0.8f);
    // for (int i = 0; i < int(objects_.size()); i++)
    //     objects_.at(i)->draw();
}

void Viewer::drawWithNames() 
{
    // for (int i = 0; i < int(objects_.size()); i++) 
    // {
    //     glPushName(i);
    //     objects_.at(i)->draw();
    //     glPopName();
    // }
    // fprintf(stderr, "Viewer::drawWithNames()\n");
    // fprintf(stderr, "drawSelectableCloud.objects.size : %d\n",
    //                 drawSelectableCloud.objects.size());
    // 非空 画图
    if (drawSelectableCloud.objects.size() != 0)
    {
        // this->objects.assign(drawSelectableCloud.objects.begin(), 
        //         drawSelectableCloud.objects.end());
        int idx = 0;
        for (auto & elem : drawSelectableCloud.objects)
        {
            // fprintf(stderr, "Cloud pushName with %d\n", idx);
            glPushName(idx);
            elem->draw();
            glPopName();
            ++idx;
        }
    }

    // 对象减去点云对象数目， 等于 bbox 的 ID
    // fprintf(stderr, "drawSelectableBBox.objects.size : %d\n", drawSelectableBBox.objects.size());
    if(drawSelectableBBox.objects.size() != 0)
    {
        int idx = drawSelectableCloud.objects.size();
        for (auto & elem : drawSelectableBBox.objects)
        {
            // fprintf(stderr, "BBox pushName with %d\n", idx);
            glPushName(idx);
            elem->draw();
            glPopName();
            ++idx;
        }
    }

    

}

// 显示按压事件
void Viewer::mousePressEvent(QMouseEvent *e) {
    // Start selection. Mode is ADD with Shift key and TOGGLE with Alt key.
    //   qDebug() << e->pos() << std::endl;
    rectangle_ = QRect(e->pos(), e->pos());

    if ((e->button() == Qt::LeftButton) && (e->modifiers() == Qt::ShiftModifier))
    {
        selectionMode_ = ADD;
    }
        // else if ((e->button() == Qt::LeftButton) &&
        //          (e->modifiers() == Qt::AltModifier))
    else if ((e->button() == Qt::LeftButton) && (e->modifiers() == Qt::AltModifier))
    {
        selectionMode_ = REMOVE;    
    }
    else 
    {
        // if (e->modifiers() == Qt::ControlModifier)
        //   startManipulation();  //不需要操作
        QGLViewer::mousePressEvent(e);
    }
}

void Viewer::mouseMoveEvent(QMouseEvent *e) 
{
    if (selectionMode_ != NONE) 
    {
        // Updates rectangle_ coordinates and redraws rectangle
        // qDebug() << "curren pose :" << e->pos() << endl;
        rectangle_.setBottomRight(e->pos());
        update();
    } 
    else
    {
        QGLViewer::mouseMoveEvent(e);
    }
}

void Viewer::mouseReleaseEvent(QMouseEvent *e) 
{
    if (selectionMode_ != NONE) 
    {
        // Actual selection on the rectangular area.
        // Possibly swap left/right and top/bottom to make rectangle_ valid.
        rectangle_ = rectangle_.normalized();
        // Define selection window dimensions
        setSelectRegionWidth(rectangle_.width());
        setSelectRegionHeight(rectangle_.height());
        // Compute rectangle center and perform selection
        select(rectangle_.center());
        // Update display to show new selected objects
        update();
    } 
    else
        QGLViewer::mouseReleaseEvent(e);
}

void Viewer::endSelection(const QPoint &) 
{
    // Flush GL buffers
    // fprintf(stderr, "Viewer::endSelection(const QPoint &)\n");
    glFlush();

    // Get the number of objects that were seen through the pick matrix frustum.
    // Reset GL_RENDER mode.
    GLint nbHits = glRenderMode(GL_RENDER);
    // fprintf(stderr, "nbHits %d has hited\n", nbHits);

    if (nbHits > 0) 
    {
        // Interpret results : each object created 4 values in the selectBuffer().
        // (selectBuffer())[4*i+3] is the id pushed on the stack.
        for (int i = 0; i < nbHits; ++i)
        {
            switch (selectionMode_) 
            {
            case ADD:
                addIdToSelection((selectBuffer())[4 * i + 3]);
                break;
            case REMOVE:
                removeIdFromSelection((selectBuffer())[4 * i + 3]);
                break;
            default:
                break;
            }
        }

        // for (auto & ele : selection)
        // {
        //     // std::cout << ele << " ";
        //     fprintf(stderr, "%d ", ele);
        // }
        // fprintf(stderr, "\nhas been selected!\n\n\n\n");
    }

    // 打印添加后的 ID

    selectionMode_ = NONE;
}



void Viewer::removeIdFromSelection(int id) 
{ 
    // selection.removeAll(id); 
    selection.clear();
}

void Viewer::init()
{    
    // glClearColor(1.0f,1.0f,1.0f,1.0f);
    // glClear(GL_COLOR_BUFFER_BIT);
    // fprintf(stderr, "Viewer::init()\n");

    setSceneRadius(100.0);
    // setBackgroundColor(QColor(0, 0, 0));
    // setBackgroundColor(QColor(1, 1, 1));
    // fprintf(stderr, "setBackgroundColor(QColor(1, 1, 1));\n");
    camera()->showEntireScene();
    glDisable(GL_LIGHTING);
    glBlendFunc(GL_ONE, GL_ONE);
    // setBackgroundColor(QColor(0.0f, 0.0f, 0.0f));
    // setForegroundColor(QColor(1.0f, 1.0f, 1.0f));
}

void Viewer::addIdToSelection(int id) 
{
    // if (!selection.contains(id))
    //     selection.push_back(id);
    int cloudNum = drawSelectableCloud.objects.size();
    int bboxNum = drawSelectableBBox.objects.size();
    auto it = std::find(selection.begin(), selection.end(), id);
    if (it == selection.end())
    {
        if (id < cloudNum)
            selection.emplace_back(id);
        if (id >= cloudNum && 
            id < (cloudNum + bboxNum))
        {
            bboxSelection.emplace_back(id - cloudNum);
        }
    }
}


// void Viewer::postSelection(const QPoint &point)
// {
//     bool found;
//     selectedPoint = camera()->pointUnderPixel(point, found);
//     // qDebug() << "point (" << QString(point.x()) + "," + QString(point.y()) + ")" << endl;
//     // std::cout << "selectedPoint " << selectedPoint << std::endl;
//     // qDebug() <<"selectedPoint " << "(" + QString(selectedPoint.x) + "," +
//     //                                      QString(selectedPoint.y) + "," +
//     //                                      QString(selectedPoint.z) +")" << endl;

//     fprintf(stderr, "(%f, %f , %f ) cilicked\n", selectedPoint.x, selectedPoint.y, selectedPoint.z);
// }

void Viewer::getClickedPoint(double &x, double &y)
{
    // auto a = selectedPoint.x;
    // qreal c = selectedPoint.y;
    // x = reinterpret_cast<double>(selectedPoint.x);
    // y = reinterpret_cast<double>(selectedPoint.y);
    x = (double)(selectedPoint.x);
    y = (double)(selectedPoint.y);
}


void Viewer::drawSelectionRectangle() const 
{
    startScreenCoordinatesSystem();
    glDisable(GL_LIGHTING);

    glEnable(GL_BLEND);
    glColor4f(0.0, 0.0, 0.3f, 0.3f);
    glBegin(GL_QUADS);
    glVertex2i(rectangle_.left(), rectangle_.top());
    glVertex2i(rectangle_.right(), rectangle_.top());
    glVertex2i(rectangle_.right(), rectangle_.bottom());
    glVertex2i(rectangle_.left(), rectangle_.bottom());
    glEnd();

    glLineWidth(2.0);
    glColor4f(0.4f, 0.4f, 0.5f, 0.5f);
    glBegin(GL_LINE_LOOP);
    glVertex2i(rectangle_.left(), rectangle_.top());
    glVertex2i(rectangle_.right(), rectangle_.top());
    glVertex2i(rectangle_.right(), rectangle_.bottom());
    glVertex2i(rectangle_.left(), rectangle_.bottom());
    glEnd();

    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
    stopScreenCoordinatesSystem();
}