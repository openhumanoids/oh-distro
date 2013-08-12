/****************************************************************************
** Meta object code from reading C++ file 'controlform.hpp'
**
** Created: Thu Aug 8 11:05:00 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/gui_control/controlform.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'controlform.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ControlForm[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      13,   12,   12,   12, 0x08,
      36,   12,   12,   12, 0x08,
      59,   12,   12,   12, 0x08,
      82,   12,   12,   12, 0x08,
     105,   12,   12,   12, 0x08,
     128,   12,   12,   12, 0x08,
     151,   12,   12,   12, 0x08,
     174,   12,   12,   12, 0x08,
     197,   12,   12,   12, 0x0a,
     211,   12,   12,   12, 0x0a,
     227,   12,   12,   12, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_ControlForm[] = {
    "ControlForm\0\0on_posButton_clicked()\0"
    "on_velButton_clicked()\0on_hndButton_clicked()\0"
    "on_calButton_clicked()\0on_topButton_clicked()\0"
    "on_mdlButton_clicked()\0on_btmButton_clicked()\0"
    "on_stpButton_clicked()\0sendControl()\0"
    "sendCalibrate()\0stop()\0"
};

void ControlForm::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ControlForm *_t = static_cast<ControlForm *>(_o);
        switch (_id) {
        case 0: _t->on_posButton_clicked(); break;
        case 1: _t->on_velButton_clicked(); break;
        case 2: _t->on_hndButton_clicked(); break;
        case 3: _t->on_calButton_clicked(); break;
        case 4: _t->on_topButton_clicked(); break;
        case 5: _t->on_mdlButton_clicked(); break;
        case 6: _t->on_btmButton_clicked(); break;
        case 7: _t->on_stpButton_clicked(); break;
        case 8: _t->sendControl(); break;
        case 9: _t->sendCalibrate(); break;
        case 10: _t->stop(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData ControlForm::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ControlForm::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_ControlForm,
      qt_meta_data_ControlForm, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ControlForm::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ControlForm::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ControlForm::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ControlForm))
        return static_cast<void*>(const_cast< ControlForm*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int ControlForm::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
