#include "QtPointContactRelation.h"

#define DEFAULT_OFFSET 0.0

using namespace action_authoring;
using namespace std;

typedef PointContactRelation PCR;
typedef PointContactRelationPtr PcrPtr;

QtPointContactRelation::
QtPointContactRelation()
{
    internalUpdateHappening = true;
    _xOffset = new QDoubleSpinBox();
    _yOffset = new QDoubleSpinBox();
    _zOffset = new QDoubleSpinBox();

    _xInequality = new QComboBox();
    _yInequality = new QComboBox();
    _zInequality = new QComboBox();

    _inequalities = new QWidget();

    _point1X = new QDoubleSpinBox();
    _point1Y = new QDoubleSpinBox();
    _point1Z = new QDoubleSpinBox();
    _point2X = new QDoubleSpinBox();
    _point2Y = new QDoubleSpinBox();
    _point2Z = new QDoubleSpinBox();

    _point1X->setDecimals(6);
    _point1Y->setDecimals(6);
    _point1Z->setDecimals(6);
    _point2X->setDecimals(6);
    _point2Y->setDecimals(6);
    _point2Z->setDecimals(6);


    _exacts = new QWidget();

    _panel = new QWidget(this);
    _actionDescLabel = new QLabel();

    _initialized = false;
    internalUpdateHappening = false;
}

QtPointContactRelation::
~QtPointContactRelation()
{

}

QWidget*
QtPointContactRelation::
getPanel() 
{
    if (_initialized)
    {
        return _inequalities;
    }

    _xOffset->setValue(DEFAULT_OFFSET);
    _yOffset->setValue(DEFAULT_OFFSET);
    _zOffset->setValue(DEFAULT_OFFSET);

    _xInequality->insertItem(0, PCR::UNDEFINED_STR.c_str());
    _xInequality->insertItem(0, PCR::GREATER_THAN_STR.c_str());
    _xInequality->insertItem(0, PCR::LESS_THAN_STR.c_str());
    _xInequality->insertItem(0, PCR::EQUAL_STR.c_str());

    _yInequality->insertItem(0, PCR::UNDEFINED_STR.c_str());
    _yInequality->insertItem(0, PCR::GREATER_THAN_STR.c_str());
    _yInequality->insertItem(0, PCR::LESS_THAN_STR.c_str());
    _yInequality->insertItem(0, PCR::EQUAL_STR.c_str());

    _zInequality->insertItem(0, PCR::UNDEFINED_STR.c_str());
    _zInequality->insertItem(0, PCR::GREATER_THAN_STR.c_str());
    _zInequality->insertItem(0, PCR::LESS_THAN_STR.c_str());
    _zInequality->insertItem(0, PCR::EQUAL_STR.c_str());

    _actionDescLabel->setTextFormat(Qt::RichText);

    QHBoxLayout* exactsLayout = new QHBoxLayout();
    exactsLayout->addWidget(new QLabel("p1 X:"));
    exactsLayout->addWidget(_point1X);
    exactsLayout->addWidget(new QLabel("p1 Y:"));
    exactsLayout->addWidget(_point1Y);
    exactsLayout->addWidget(new QLabel("p1 Z:"));
    exactsLayout->addWidget(_point1Z);

    exactsLayout->addWidget(new QLabel("p2 X:"));
    exactsLayout->addWidget(_point2X);
    exactsLayout->addWidget(new QLabel("p2 Y:"));
    exactsLayout->addWidget(_point2Y);
    exactsLayout->addWidget(new QLabel("p2 Z:"));
    exactsLayout->addWidget(_point2Z);

    _exacts->setLayout(exactsLayout);

    QHBoxLayout* inequalitiesLayout = new QHBoxLayout();

    inequalitiesLayout->addWidget(new QLabel("x-axis: "));
    inequalitiesLayout->addWidget(_xInequality);
    inequalitiesLayout->addWidget(_xOffset);
    
    inequalitiesLayout->addWidget(new QLabel("y-axis: "));
    inequalitiesLayout->addWidget(_yInequality);
    inequalitiesLayout->addWidget(_yOffset);

    inequalitiesLayout->addWidget(new QLabel("z-axis: "));
    inequalitiesLayout->addWidget(_zInequality);
    inequalitiesLayout->addWidget(_zOffset);

    _inequalities->setLayout(inequalitiesLayout);


    QVBoxLayout* fullLayout = new QVBoxLayout();
    fullLayout->addWidget(_actionDescLabel);
    fullLayout->addWidget(_exacts);
    fullLayout->addWidget(_inequalities);
    _panel->setLayout(fullLayout);

    connect(_xOffset, SIGNAL(valueChanged(double)), this, SLOT(updateStateFromGUI()));
    connect(_yOffset, SIGNAL(valueChanged(double)), this, SLOT(updateStateFromGUI()));
    connect(_zOffset, SIGNAL(valueChanged(double)), this, SLOT(updateStateFromGUI()));
    connect(_point1X, SIGNAL(valueChanged(double)), this, SLOT(updateStateFromGUI()));
    connect(_point1Y, SIGNAL(valueChanged(double)), this, SLOT(updateStateFromGUI()));
    connect(_point1Z, SIGNAL(valueChanged(double)), this, SLOT(updateStateFromGUI()));
    connect(_point2X, SIGNAL(valueChanged(double)), this, SLOT(updateStateFromGUI()));
    connect(_point2Y, SIGNAL(valueChanged(double)), this, SLOT(updateStateFromGUI()));
    connect(_point2Z, SIGNAL(valueChanged(double)), this, SLOT(updateStateFromGUI()));
    connect(_xInequality, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStateFromGUI()));
    connect(_yInequality, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStateFromGUI()));
    connect(_zInequality, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStateFromGUI()));

    _initialized = true;

    return _panel;
}

void
QtPointContactRelation::
setPointContactRelation(PointContactRelationPtr relation)
{
    if (relation != NULL) {
        _pcr = relation;
        updateGUIFromState();
    }
}

void
QtPointContactRelation::
updateStateFromGUI() {
    if (! internalUpdateHappening) {
        if (_pcr != NULL) {
            handleZIneqChange();
            handleYIneqChange();
            handleXIneqChange();
            handleZOffsetChange();
            handleYOffsetChange();
            handleXOffsetChange();
        
            _pcr->setPoint1(Eigen::Vector3f(_point1X->value(),
                                            _point1Y->value(),
                                            _point1Z->value()));
            _pcr->setPoint2(Eigen::Vector3f(_point2X->value(),
                                            _point2Y->value(),
                                            _point2Z->value()));

            _actionDescLabel->setText(QString::fromStdString(getModePrompt()));
            cout << "sent signal!" << endl;
            emit activatedSignal();
    
        }
    }
}

void
QtPointContactRelation::
updateGUIFromState()
{
    internalUpdateHappening = true;

    // set the x,y,z relations
    string xiq = PCR::typeToStr(_pcr->getXInequality());
    string yiq = PCR::typeToStr(_pcr->getYInequality());
    string ziq = PCR::typeToStr(_pcr->getZInequality());

    int xIndex = _xInequality->findText(xiq.c_str());
    int yIndex = _yInequality->findText(yiq.c_str());
    int zIndex = _zInequality->findText(ziq.c_str());    

    if (xIndex != -1)
      _xInequality->setCurrentIndex(xIndex);
    if (yIndex != -1)
      _yInequality->setCurrentIndex(yIndex);
    if (zIndex != -1)
      _zInequality->setCurrentIndex(zIndex);

    //set offset if any of the above got set
    if (xIndex != -1 || yIndex != -1 || zIndex != -1)
      {
        _xOffset->setValue(_pcr->getXOffset());
        _yOffset->setValue(_pcr->getYOffset());
        _zOffset->setValue(_pcr->getZOffset());
      }

    _point1X->setValue(_pcr->getPoint1()[0]);
    _point1Y->setValue(_pcr->getPoint1()[1]);
    _point1Z->setValue(_pcr->getPoint1()[2]);

    _point2X->setValue(_pcr->getPoint2()[0]);
    _point2Y->setValue(_pcr->getPoint2()[1]);
    _point2Z->setValue(_pcr->getPoint2()[2]);

    // prompt to set relation state
    _actionDescLabel->setText(QString::fromStdString(getModePrompt()));

    internalUpdateHappening = false;
}

/**set the corresponding point contact relation to reflect the change
in the inequality menu*/
void QtPointContactRelation::handleXIneqChange()
{
//  PcrPtr _pcr = getCurrentPCR();
//  if (_pcr == PcrPtr())
//    return;
  _pcr->setXInequality(PointContactRelation::strToType(_xInequality->currentText().toStdString()));
}

/**set the corresponding point contact relation to reflect the change
in the inequality menu*/
void QtPointContactRelation::handleYIneqChange()
{
//  PcrPtr _pcr = getCurrentPCR();
//  if (_pcr == PcrPtr())
//    return;
    _pcr->setYInequality(PointContactRelation::strToType(_yInequality->currentText().toStdString()));  
}

/**set the corresponding point contact relation to reflect the change
in the inequality menu*/
void QtPointContactRelation::handleZIneqChange()
{
//  PcrPtr _pcr = getCurrentPCR();
//  if (_pcr == PcrPtr())
//    return;
  _pcr->setZInequality(PointContactRelation::strToType(_zInequality->currentText().toStdString()));
}

 
/**set the corresponding point contact relation to reflect the change
in the offset box*/
void QtPointContactRelation::handleXOffsetChange()
{
//  PcrPtr _pcr = getCurrentPCR();
//  if (_pcr == PcrPtr())
//    return;
  _pcr->setOffset(Eigen::Vector3f(_xOffset->value(),  //change only the x
                                  _pcr->getYOffset(),
                                  _pcr->getZOffset()));
}

/**set the corresponding point contact relation to reflect the change
in the offset box*/
void QtPointContactRelation::handleYOffsetChange()
{
//  PcrPtr _pcr = getCurrentPCR();
//  if (_pcr == PcrPtr())
//    return;
  _pcr->setOffset(Eigen::Vector3f(_pcr->getXOffset(),
                                  _yOffset->value(),  //change only the y
                                  _pcr->getZOffset()));
}


/**set the corresponding point contact relation to reflect the change
in the offset box*/
void QtPointContactRelation::handleZOffsetChange()
{
//  PcrPtr _pcr = getCurrentPCR();
//  if (_pcr == PcrPtr())
//    return;
  _pcr->setOffset(Eigen::Vector3f(_pcr->getXOffset(),
                                  _pcr->getYOffset(),
                                  _zOffset->value()));  //change only the z
}

PointContactRelationPtr 
QtPointContactRelation::
getPointContactRelation() {
    return _pcr;
}

std::string
QtPointContactRelation::
getModePrompt()
{
    std::stringstream ss;
    ss << "<b>constraint " << "</b>:" <<
       _pcr->getState() <<
       "<br/><b>prompt: </b>" << _pcr->getPrompt();
    return ss.str();
}
