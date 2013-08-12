/****************************************************************************
** Meta object code from reading C++ file 'homing_dialog.h'
**
** Created: Tue Aug 6 13:41:09 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../gui/homing/homing_dialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'homing_dialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ManualFingerSubtab[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      20,   19,   19,   19, 0x0a,
      37,   19,   19,   19, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_ManualFingerSubtab[] = {
    "ManualFingerSubtab\0\0sendFingerPose()\0"
    "setFingerHome()\0"
};

void ManualFingerSubtab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ManualFingerSubtab *_t = static_cast<ManualFingerSubtab *>(_o);
        switch (_id) {
        case 0: _t->sendFingerPose(); break;
        case 1: _t->setFingerHome(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData ManualFingerSubtab::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ManualFingerSubtab::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_ManualFingerSubtab,
      qt_meta_data_ManualFingerSubtab, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ManualFingerSubtab::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ManualFingerSubtab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ManualFingerSubtab::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ManualFingerSubtab))
        return static_cast<void*>(const_cast< ManualFingerSubtab*>(this));
    return QWidget::qt_metacast(_clname);
}

int ManualFingerSubtab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}
static const uint qt_meta_data_ManualTab[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

static const char qt_meta_stringdata_ManualTab[] = {
    "ManualTab\0"
};

void ManualTab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObjectExtraData ManualTab::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ManualTab::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_ManualTab,
      qt_meta_data_ManualTab, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ManualTab::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ManualTab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ManualTab::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ManualTab))
        return static_cast<void*>(const_cast< ManualTab*>(this));
    return QWidget::qt_metacast(_clname);
}

int ManualTab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
static const uint qt_meta_data_AutoTab[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      17,    9,    8,    8, 0x0a,
      32,    8,    8,    8, 0x0a,
      39,    8,    8,    8, 0x0a,
      52,    8,    8,    8, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_AutoTab[] = {
    "AutoTab\0\0enabled\0automove(bool)\0home()\0"
    "move_thumb()\0rosTimerTimeout()\0"
};

void AutoTab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        AutoTab *_t = static_cast<AutoTab *>(_o);
        switch (_id) {
        case 0: _t->automove((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->home(); break;
        case 2: _t->move_thumb(); break;
        case 3: _t->rosTimerTimeout(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData AutoTab::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject AutoTab::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_AutoTab,
      qt_meta_data_AutoTab, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &AutoTab::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *AutoTab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *AutoTab::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_AutoTab))
        return static_cast<void*>(const_cast< AutoTab*>(this));
    return QWidget::qt_metacast(_clname);
}

int AutoTab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}
static const uint qt_meta_data_HomingDialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

static const char qt_meta_stringdata_HomingDialog[] = {
    "HomingDialog\0"
};

void HomingDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObjectExtraData HomingDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject HomingDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_HomingDialog,
      qt_meta_data_HomingDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &HomingDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *HomingDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *HomingDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_HomingDialog))
        return static_cast<void*>(const_cast< HomingDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int HomingDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
QT_END_MOC_NAMESPACE
