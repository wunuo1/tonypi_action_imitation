#ifndef INCLUDE_ORDER_INTERPRETER_H_
#define INCLUDE_ORDER_INTERPRETER_H_

#include "python3.10/Python.h"
#include <string>


class OrderInterpreter{
public:
    OrderInterpreter(){
        Py_Initialize();

        pModule_serial_servo_ = PyImport_ImportModule("hiwonder.ActionGroupControl");
        if (!pModule_serial_servo_) {
            PyErr_Print();
        }
        
        pFunc_serial_servo_ = PyObject_GetAttrString(pModule_serial_servo_, "runActionGroup");
        if (!pFunc_serial_servo_ || !PyCallable_Check(pFunc_serial_servo_)) {
            PyErr_Print();
        }

        pModule_PWM_servo_ = PyImport_ImportModule("hiwonder.Board");
        if (!pModule_PWM_servo_) {
            PyErr_Print();
        }
        
        pFunc_PWM_servo_ = PyObject_GetAttrString(pModule_PWM_servo_, "setPWMServoPulse");
        if (!pFunc_PWM_servo_ || !PyCallable_Check(pFunc_PWM_servo_)) {
            PyErr_Print();
        }

        pTimeModule = PyImport_ImportModule("time");
        if (pTimeModule == nullptr) {
            PyErr_Print();
        }

        pSleepFunc = PyObject_GetAttrString(pTimeModule, "sleep");
        if (pSleepFunc == nullptr || !PyCallable_Check(pSleepFunc)) {
            PyErr_Print();
        }

    }

    ~OrderInterpreter(){
        Py_DECREF(pFunc_serial_servo_);
        Py_DECREF(pModule_serial_servo_);

        Py_DECREF(pFunc_PWM_servo_);
        Py_DECREF(pModule_PWM_servo_);

        Py_Finalize();
    }

    void control_serial_servo(const std::string &order){
        const char* cstr = order.c_str();
        PyObject* pArgs = PyTuple_Pack(1, PyUnicode_FromString(cstr));
        if (!pArgs) {
            PyErr_Print();
            return;
        }
        PyObject_CallObject(pFunc_serial_servo_, pArgs);
        Py_DECREF(pArgs);
    }

    void control_PWM_servo(const int &servo_id, const int &pulse, const int &use_time){
        PyObject* pArgs = PyTuple_Pack(3, PyLong_FromLong(servo_id), PyLong_FromLong(pulse), PyLong_FromLong(use_time));
        if (!pArgs) {
            PyErr_Print();
            return;
        }
        PyObject_CallObject(pFunc_PWM_servo_, pArgs);
        PyObject* pArgs_sleep = PyTuple_Pack(1, PyFloat_FromDouble(use_time / 1000.0));
        PyObject_CallObject(pSleepFunc, pArgs_sleep);

        Py_DECREF(pArgs);
        Py_DECREF(pArgs_sleep);

    }

private:
    PyObject* pModule_serial_servo_;
    PyObject* pFunc_serial_servo_;

    PyObject* pModule_PWM_servo_;
    PyObject* pFunc_PWM_servo_;

    PyObject* pTimeModule;
    PyObject* pSleepFunc;
};

#endif  // INCLUDE_ORDER_INTERPRETER_H_