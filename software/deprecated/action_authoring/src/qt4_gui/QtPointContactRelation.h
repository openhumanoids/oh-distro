#ifndef QTPOINTCONTACTRELATION_H
#define QTPOINTCONTACTRELATION_H

// QT includes
#include <QtGui>
#include <QWidget>
#include <QApplication>

#include <action_authoring/RelationState.h>
#include <action_authoring/PointContactRelation.h>

namespace action_authoring
{
class QtPointContactRelation : public QWidget
{
    Q_OBJECT

public:
	QtPointContactRelation();
	~QtPointContactRelation();
    void setPointContactRelation(PointContactRelationPtr relation);
    PointContactRelationPtr getPointContactRelation();
    QWidget* getPanel();
    void updateGUIFromState();
    std::string getModePrompt();

private: 
    bool _initialized;
    PointContactRelationPtr _pcr;
    QDoubleSpinBox* _xOffset;
    QDoubleSpinBox* _yOffset;
    QDoubleSpinBox* _zOffset;
    QComboBox* _xInequality;
    QComboBox* _yInequality;
    QComboBox* _zInequality;
    QWidget* _inequalities;

    QWidget* _exacts;
    QDoubleSpinBox* _point1X;
    QDoubleSpinBox* _point1Y;
    QDoubleSpinBox* _point1Z;

    QDoubleSpinBox* _point2X;
    QDoubleSpinBox* _point2Y;
    QDoubleSpinBox* _point2Z;

    QWidget* _panel;
    QLabel *_actionDescLabel;

    bool internalUpdateHappening;

private slots:
    void handleXIneqChange();
    void handleYIneqChange();
    void handleZIneqChange();
    void handleXOffsetChange();
    void handleYOffsetChange();
    void handleZOffsetChange();
    void updateStateFromGUI();

signals:
    void activatedSignal();

}; //QtPointContactRelation

typedef boost::shared_ptr<QtPointContactRelation> QtPointContactRelationPtr;
} // end namespace action_authoring
#endif
