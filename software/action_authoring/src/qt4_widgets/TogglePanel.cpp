#include "TogglePanel.h"

TogglePanel::
TogglePanel(QObject *parent, QString headerText) 
{
    _headerText = headerText;
    _parent = parent;
    _state = CLOSED;

    _headerArea = new QWidget;
    _widgetArea = new QWidget;
    _headerArea->setMaximumHeight(20);
    _widgetArea->setMaximumHeight(1000);
    this->setMaximumHeight(1000);
//    this->setMinimumHeight(this->sizeHint().height());
//    this->setMinimumWidth(this->sizeHint().width());

    icon = new QPushButton(QString::fromUtf8("\u25B8"));
    icon->setMaximumSize(15, 15);

    //icon->setPixmap(_collapseIcon);
    connect(icon, SIGNAL(clicked()), this, SLOT(changeState()));

    _headerTextLabel = new ExtendedQLabel;
    _headerTextLabel->setText(_headerText);

    connect(_headerTextLabel, SIGNAL(clicked()), this, SLOT(changeState()));

    _widgetLayout = new QVBoxLayout;
    _widgetLayout->setMargin(0);
    _widgetArea->setLayout(_widgetLayout);
    _widgetArea->hide();

    QHBoxLayout* headerLayout = new QHBoxLayout;
    headerLayout->setMargin(0);
    headerLayout->addWidget(icon); /*, 0, Qt::AlignTop | Qt:: AlignLeft);*/
    headerLayout->addWidget(_headerTextLabel); /*, 0, Qt::AlignTop | Qt:: AlignLeft);*/
    _headerArea->setLayout(headerLayout);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->setMargin(0);
    mainLayout->setSpacing(0);
    mainLayout->addWidget(_headerArea);/*, 0, Qt::AlignTop | Qt:: AlignLeft);*/
    _headerArea->setStyleSheet("QPushButton { border: 0px solid gray; border-radius: 0px; "
			       "padding: 0px; margin: 0px; background-color: transparent; } "
			       "QPushButton:focus { background-color: none; }");


    mainLayout->addWidget(_widgetArea);/*, 0, Qt::AlignTop | Qt:: AlignLeft);*/

    setLayout(mainLayout);
//    show();
}

void TogglePanel::setSelected(bool selected) {
    if (selected) {
	this->setStyleSheet("QGroupBox { background-color: #90EE90; }");
	_headerTextLabel->setStyleSheet("font-weight: bold");
    } else {
	this->setStyleSheet("");
	_headerTextLabel->setStyleSheet("");
    }
}

void TogglePanel::changeState()
{
    if(_state == OPEN)
    {
        _state = CLOSED;
	icon->setText(QString::fromUtf8("\u25B8"));
        //_headerTextLabel->setText("Closed");
        _widgetArea->hide();
    }
    else
    {
        _state = OPEN;
	icon->setText(QString::fromUtf8("\u25BE"));
        //_headerTextLabel->setText("Open");
        _widgetArea->show();
    }
}

void TogglePanel::addWidget(QWidget * widget)
{
    _widgetLayout->addWidget(widget);
}

void TogglePanel::addLayout(QLayout *layout)
{
    _widgetLayout->addLayout(layout);
}

void TogglePanel::setTitle(QString title) {
    _headerText = title;
    _headerTextLabel->setText(_headerText);
}

QSize
TogglePanel::
sizeHint() { 
    if (_state == OPEN) {
	return QSize(_widgetArea->sizeHint().width(), _widgetArea->sizeHint().height() + _headerArea->sizeHint().height());
    } else {
	return QSize(_headerArea->sizeHint().width(), _headerArea->sizeHint().height());
    }
}

TogglePanel::
~TogglePanel()
{
    
}

