/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Mon Jan 21 17:43:52 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_action_authoring__MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      30,   29,   29,   29, 0x08,
      60,   54,   29,   29, 0x08,
      77,   29,   29,   29, 0x08,
      96,   29,   29,   29, 0x08,
     115,   29,   29,   29, 0x08,
     138,   29,   29,   29, 0x08,
     153,   29,   29,   29, 0x08,
     170,   29,   29,   29, 0x08,
     204,  194,   29,   29, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_action_authoring__MainWindow[] = {
    "action_authoring::MainWindow\0\0"
    "affordanceUpdateCheck()\0value\0"
    "updateJoint(int)\0handleLoadAction()\0"
    "handleSaveAction()\0handleDeleteWaypoint()\0"
    "handleMoveUp()\0handleMoveDown()\0"
    "handleRobotLinkChange()\0activator\0"
    "setSelectedAction(QString)\0"
};

void action_authoring::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->affordanceUpdateCheck(); break;
        case 1: _t->updateJoint((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->handleLoadAction(); break;
        case 3: _t->handleSaveAction(); break;
        case 4: _t->handleDeleteWaypoint(); break;
        case 5: _t->handleMoveUp(); break;
        case 6: _t->handleMoveDown(); break;
        case 7: _t->handleRobotLinkChange(); break;
        case 8: _t->setSelectedAction((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData action_authoring::MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject action_authoring::MainWindow::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_action_authoring__MainWindow,
      qt_meta_data_action_authoring__MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &action_authoring::MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *action_authoring::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *action_authoring::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_action_authoring__MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QWidget::qt_metacast(_clname);
}

int action_authoring::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
