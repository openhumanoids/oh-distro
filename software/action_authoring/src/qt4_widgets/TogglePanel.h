#ifndef TOGGLEPANEL_H
#define TOGGLEPANEL_H

#include <QtGui>
#include <boost/shared_ptr.hpp>
#include "ExtendedLabel.h"

class TogglePanel : public QWidget
{
    Q_OBJECT

    typedef enum
    {
        PLANNER_OK,
        PLANNER_UNKNOWN,
        PLANNER_NOT_OK,
        PLANNER_WARNING
    } PlannerStatus;

private:
    QWidget *_headerArea;
    QString _headerText;
    QWidget *_widgetArea;
    QPushButton *icon;
    QPushButton *_plannerButton;
    QPushButton *_constraintActiveButton;
    ExtendedQLabel *_headerTextLabel;
    QVBoxLayout *_widgetLayout;
    int _state;
    QObject *_parent;

public:
    ~TogglePanel();
    TogglePanel(QObject *parent, QString headerText, bool hasStatus);
    void addWidget(QWidget *widget);
    void addLayout(QLayout *layout);
    void setSelected(bool selected);
    void setTitle(QString title);
    void setPlannerStatus(PlannerStatus p);
    void setConstraintActiveStatus(bool active);
    QSize sizeHint();
    enum { OPEN, CLOSED };

private slots:
    void changeState();
};

#endif // TOGGLEPANEL_H
