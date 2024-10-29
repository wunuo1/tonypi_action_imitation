#ifndef INCLUDE_ORDER_INTERPRETER_H_
#define INCLUDE_ORDER_INTERPRETER_H_

#include "python3.10/Python.h"
#include <string>


class OrderInterpreter{
public:
    OrderInterpreter(){
        Py_Initialize();

        pModule_action_group_ = PyImport_ImportModule("hiwonder.ActionGroupControl");
        if (!pModule_action_group_) {
            PyErr_Print();
        }
        
        pFunc_run_action_group_ = PyObject_GetAttrString(pModule_action_group_, "runActionGroup");
        if (!pFunc_run_action_group_ || !PyCallable_Check(pFunc_run_action_group_)) {
            PyErr_Print();
        }

        pModule_board_ = PyImport_ImportModule("hiwonder.Board");
        if (!pModule_board_) {
            PyErr_Print();
        }
        
        pFunc_PWM_servo_ = PyObject_GetAttrString(pModule_board_, "setPWMServoPulse");
        if (!pFunc_PWM_servo_ || !PyCallable_Check(pFunc_PWM_servo_)) {
            PyErr_Print();
        }

        PFunc_Bus_servo_ = PyObject_GetAttrString(pModule_board_, "setBusServoPulse");
        if (!PFunc_Bus_servo_ || !PyCallable_Check(PFunc_Bus_servo_)) {
            PyErr_Print();
        }


        pTimeModule_ = PyImport_ImportModule("time");
        if (pTimeModule_ == nullptr) {
            PyErr_Print();
        }

        pSleepFunc_ = PyObject_GetAttrString(pTimeModule_, "sleep");
        if (pSleepFunc_ == nullptr || !PyCallable_Check(pSleepFunc_)) {
            PyErr_Print();
        }

    }

    ~OrderInterpreter(){
        Py_DECREF(pFunc_run_action_group_);
        Py_DECREF(pModule_action_group_);

        Py_DECREF(pFunc_PWM_servo_);
        Py_DECREF(pModule_board_);

        Py_Finalize();
    }

    void control_serial_servo(const std::string &order, const int &sleep_time = 0){
        const char* cstr = order.c_str();
        PyObject* pArgs = PyTuple_Pack(1, PyUnicode_FromString(cstr));
        if (!pArgs) {
            PyErr_Print();
            return;
        }
        PyObject_CallObject(pFunc_run_action_group_, pArgs);
        Py_DECREF(pArgs);
        if(sleep_time != 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
        }
    }

    void control_serial_servo(const int &servo_id, const int &pulse, const int &use_time){
        PyObject* pArgs = PyTuple_Pack(3, PyLong_FromLong(servo_id), PyLong_FromLong(pulse), PyLong_FromLong(use_time));
        if (!pArgs) {
            PyErr_Print();
            return;
        }
        PyObject_CallObject(PFunc_Bus_servo_, pArgs);
        PyObject* pArgs_sleep = PyTuple_Pack(1, PyFloat_FromDouble(use_time / 1000.0));
        PyObject_CallObject(pSleepFunc_, pArgs_sleep);
        Py_DECREF(pArgs);
        Py_DECREF(pArgs_sleep);
    }


    void control_PWM_servo(const int &servo_id, const int &pulse, const int &use_time){
        PyObject* pArgs = PyTuple_Pack(3, PyLong_FromLong(servo_id), PyLong_FromLong(pulse), PyLong_FromLong(use_time));
        if (!pArgs) {
            PyErr_Print();
            return;
        }
        PyObject_CallObject(pFunc_PWM_servo_, pArgs);
        PyObject* pArgs_sleep = PyTuple_Pack(1, PyFloat_FromDouble(use_time / 1000.0));
        PyObject_CallObject(pSleepFunc_, pArgs_sleep);

        Py_DECREF(pArgs);
        Py_DECREF(pArgs_sleep);
    }

private:
    PyObject* pModule_action_group_;
    PyObject* pFunc_run_action_group_;

    PyObject* pModule_board_;
    PyObject* pFunc_PWM_servo_;
    PyObject* PFunc_Bus_servo_;

    PyObject* pTimeModule_;
    PyObject* pSleepFunc_;
};

#endif  // INCLUDE_ORDER_INTERPRETER_H_