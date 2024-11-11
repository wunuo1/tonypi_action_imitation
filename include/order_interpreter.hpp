#ifndef INCLUDE_ORDER_INTERPRETER_H_
#define INCLUDE_ORDER_INTERPRETER_H_

#include "python3.8/Python.h"
#include <string>
#include <iostream>


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

        pModule_controller_ = PyImport_ImportModule("hiwonder.Controller");
        pModule_rrc_ = PyImport_ImportModule("hiwonder.ros_robot_controller_sdk");
        pFunc_controller_ = PyObject_GetAttrString(pModule_controller_, "Controller");
        pClass_board_ = PyObject_GetAttrString(pModule_rrc_, "Board");
        pInstance_board_ = PyObject_CallObject(pClass_board_, nullptr);
        pArgs_ = PyTuple_Pack(1, pInstance_board_);
        pInstance_ = PyObject_CallObject(pFunc_controller_, pArgs_);
        pFunc_set_pwm_servo_pulse_ = PyObject_GetAttrString(pInstance_, "set_pwm_servo_pulse");
        pFunc_set_bus_servo_pulse_ = PyObject_GetAttrString(pInstance_, "set_bus_servo_pulse");
        // pModule_board_ = PyImport_ImportModule("hiwonder.Board");
        // if (!pModule_board_) {
        //     PyErr_Print();
        // }
        
        // pFunc_PWM_servo_ = PyObject_GetAttrString(pModule_board_, "setPWMServoPulse");
        // if (!pFunc_PWM_servo_ || !PyCallable_Check(pFunc_PWM_servo_)) {
        //     PyErr_Print();
        // }

        // PFunc_Bus_servo_ = PyObject_GetAttrString(pModule_board_, "setBusServoPulse");
        // if (!PFunc_Bus_servo_ || !PyCallable_Check(PFunc_Bus_servo_)) {
        //     PyErr_Print();
        // }


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

        // Py_DECREF(pFunc_PWM_servo_);
        // Py_DECREF(pModule_board_);
        Py_DECREF(pModule_controller_);
        Py_DECREF(pModule_rrc_);
        Py_DECREF(pFunc_controller_);
        Py_DECREF(pClass_board_);
        Py_DECREF(pInstance_board_);
        Py_DECREF(pArgs_);
        Py_DECREF(pInstance_);
        Py_DECREF(pFunc_set_pwm_servo_pulse_);
        Py_DECREF(pFunc_set_bus_servo_pulse_);

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
        PyObject_CallObject(pFunc_set_bus_servo_pulse_, pArgs);
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
        PyObject_CallObject(pFunc_set_pwm_servo_pulse_, pArgs);
        PyObject* pArgs_sleep = PyTuple_Pack(1, PyFloat_FromDouble(use_time / 1000.0));
        PyObject_CallObject(pSleepFunc_, pArgs_sleep);

        Py_DECREF(pArgs);
        Py_DECREF(pArgs_sleep);
    }

private:
    PyObject* pModule_action_group_;
    PyObject* pFunc_run_action_group_;

    PyObject* pModule_controller_;
    PyObject* pModule_rrc_;
    PyObject* pFunc_controller_;
    PyObject* pClass_board_;
    PyObject* pInstance_board_;
    PyObject* pArgs_;
    PyObject* pInstance_;
    PyObject* pFunc_set_pwm_servo_pulse_;
    PyObject* pFunc_set_bus_servo_pulse_;

    // PyObject* pFunc_PWM_servo_;
    // PyObject* PFunc_Bus_servo_;

    PyObject* pTimeModule_;
    PyObject* pSleepFunc_;
};

#endif  // INCLUDE_ORDER_INTERPRETER_H_



















