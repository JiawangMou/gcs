/****************************************************************************
** Meta object code from reading C++ file 'teleop_pad.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rviz_teleop_commander/include/rviz_teleop_commander/teleop_pad.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'teleop_pad.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz_teleop_commander__FMAVStatusPanel_t {
    QByteArrayData data[9];
    char stringdata0[167];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz_teleop_commander__FMAVStatusPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz_teleop_commander__FMAVStatusPanel_t qt_meta_stringdata_rviz_teleop_commander__FMAVStatusPanel = {
    {
QT_MOC_LITERAL(0, 0, 38), // "rviz_teleop_commander::FMAVSt..."
QT_MOC_LITERAL(1, 39, 15), // "updateMAVStatus"
QT_MOC_LITERAL(2, 55, 0), // ""
QT_MOC_LITERAL(3, 56, 36), // "mav_comm_driver::MAVStatus::C..."
QT_MOC_LITERAL(4, 93, 12), // "uploadConfig"
QT_MOC_LITERAL(5, 106, 15), // "checkConnection"
QT_MOC_LITERAL(6, 122, 12), // "setParamMode"
QT_MOC_LITERAL(7, 135, 16), // "changeTuningAxis"
QT_MOC_LITERAL(8, 152, 14) // "enableThrottle"

    },
    "rviz_teleop_commander::FMAVStatusPanel\0"
    "updateMAVStatus\0\0mav_comm_driver::MAVStatus::ConstPtr\0"
    "uploadConfig\0checkConnection\0setParamMode\0"
    "changeTuningAxis\0enableThrottle"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz_teleop_commander__FMAVStatusPanel[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x09 /* Protected */,
       4,    0,   47,    2, 0x09 /* Protected */,
       5,    0,   48,    2, 0x09 /* Protected */,
       6,    1,   49,    2, 0x09 /* Protected */,
       7,    1,   52,    2, 0x09 /* Protected */,
       8,    0,   55,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void,

       0        // eod
};

void rviz_teleop_commander::FMAVStatusPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        FMAVStatusPanel *_t = static_cast<FMAVStatusPanel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateMAVStatus((*reinterpret_cast< const mav_comm_driver::MAVStatus::ConstPtr(*)>(_a[1]))); break;
        case 1: _t->uploadConfig(); break;
        case 2: _t->checkConnection(); break;
        case 3: _t->setParamMode((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->changeTuningAxis((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->enableThrottle(); break;
        default: ;
        }
    }
}

const QMetaObject rviz_teleop_commander::FMAVStatusPanel::staticMetaObject = {
    { &rviz::Panel::staticMetaObject, qt_meta_stringdata_rviz_teleop_commander__FMAVStatusPanel.data,
      qt_meta_data_rviz_teleop_commander__FMAVStatusPanel,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz_teleop_commander::FMAVStatusPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz_teleop_commander::FMAVStatusPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_teleop_commander__FMAVStatusPanel.stringdata0))
        return static_cast<void*>(const_cast< FMAVStatusPanel*>(this));
    return rviz::Panel::qt_metacast(_clname);
}

int rviz_teleop_commander::FMAVStatusPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
