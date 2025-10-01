/*
 * vilma_ma_labeling.hpp
 *
 *  Created on: Mar 12, 2025
 *
 *  Author: Gabriel Toffanetto França da Rocha
 *
 *  Laboratory of Autonomous Mobility (LMA)
 *  School of Mechanical Engineering (FEM)
 *  University of Campinas (Unicamp)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

 #ifndef VILMA_MA_LABELING__HPP_
#define VILMA_MA_LABELING__HPP_

//* Sensors UDP message from MA definitions
class SensorsMA
{
public:
    constexpr static int ROS_TIME = 0;
    constexpr static int TIME_PPC = 1;
    constexpr static int OPERATION_STATE = 2;
    constexpr static int STEER_POS = 3;
    constexpr static int SPEER_SPEED = 4;
    constexpr static int STEER_STATE = 5;
    constexpr static int STEER_VM = 6;
    constexpr static int STEER_T_USER = 7;
    constexpr static int GAS_VALUE = 8;
    constexpr static int GAS_USER_VALUE = 9;
    constexpr static int CURRENT_SENSOR = 10;
    constexpr static int ACC_COUNTER = 11;
    constexpr static int ACC_X = 12;
    constexpr static int ACC_Y = 13;
    constexpr static int ACC_Z = 14;
    constexpr static int ATMOSPHERE_PRESSURE = 15;
    constexpr static int TIME_PCC_BRAKE = 16;
    constexpr static int BRAKE_STATE = 17;
    constexpr static int BRAKE_VALUE = 18;
    constexpr static int BRAKE_WS_FRONT_LEFT = 19;
    constexpr static int BRAKE_WS_FRONT_RIGHT = 20;
    constexpr static int BRAKE_WS_BACK_LEFT = 21;
    constexpr static int BRAKE_WS_BACK_RIGHT = 22;
    constexpr static int BRAKE_WS_ACC_X = 23;
    constexpr static int BRAKE_WS_ACC_Y = 24;
    constexpr static int BRAKE_WS_ACC_Z = 25;
    constexpr static int BRAKE_WS_GYR_Z = 26;
    constexpr static int BRAKE_USER_PRESSURE = 27;
    constexpr static int BRAKE_FRONT_ENCODER = 28;
    constexpr static int GEAR_STATE = 30;
    constexpr static int GEAR_OFF = 0;
    constexpr static int GEAR_N = 1;
    constexpr static int GEAR_R = 2;
    constexpr static int GEAR_D = 3;
};

//* State UDP message from MA definitions
class StateMA
{
public:
    constexpr static int ROS_TIME = 0;
    constexpr static int TIME_PCC = 1;
    constexpr static int STEER_ANGLE = 2;
    constexpr static int STEER_SPEED_ANGLE = 3;
    constexpr static int STEER_TIRE_ANGLE = 4;
    constexpr static int STEER_TIRE_SPEED_ANGLE = 5;
    constexpr static int LATERAL_VELOCITY = 6;
    constexpr static int ANGULAR_YAW_SPEED = 7;
    constexpr static int X_GLOBAL_POSITION = 8;
    constexpr static int Y_GLOBAL_POSITION = 9;
    constexpr static int YAW_ANGLE = 10;
    constexpr static int LONGITUDINAL_SPEED = 11;
    constexpr static int USER_TORQUE = 12;
    constexpr static int STEER_MOTOR_VOLTAGE = 13;
    constexpr static int YAW_DC_ERROR = 14;
};

//* Joystick UDP message to MA definitions
class JoystickMA
{
public:
    constexpr static int ROS_TIME = 0;
    constexpr static int TIME_VALIDITY = 1;
    constexpr static int BRAKE_COMMAND = 2;
    constexpr static int BRAKE_VALUE = 3;
    constexpr static int STEER_COMMAND = 4;
    constexpr static int STEER_VALUE = 5;
    constexpr static int GAS_COMMAND = 6;
    constexpr static int GAS_VALUE = 7;
    constexpr static int GEAR_STATE = 8;
    constexpr static int GEAR_VALUE = 9;
    constexpr static int GAS_COMMAND_OFF = 0;
    constexpr static int GAS_COMMAND_PEDAL_SPEED = 1;
    constexpr static int GAS_COMMAND_POSITION = 2;
    constexpr static int BRAKE_COMMAND_OFF = 0;
    constexpr static int BRAKE_COMMAND_AUTO = 2;
    constexpr static int STEER_COMMAND_OFF = 0;
    constexpr static int STEER_COMMAND_SPEED = 1;
    constexpr static int STEER_COMMAND_POSITION = 2;
    constexpr static int STEER_COMMAND_VOLTAGE = 3;
    constexpr static int STEER_COMMAND_LOOK_ZERO = 4;
    constexpr static int GEAR_COMMAND_OFF = 0;
    constexpr static int GEAR_COMMAND_NEUTRAL = 1;
    constexpr static int GEAR_COMMAND_REVERSE = 2;
    constexpr static int GEAR_COMMAND_DRIVE = 3;
    constexpr static int JOYSTICK_MA_DATA_LENGTH = 10;
};

//* MA's operation modes definitions
class OperationModeMA
{
public:
    constexpr static int INITIAL_STATE_MODE = 1;
    constexpr static int MANUAL_MODE = 2;
    constexpr static int JOYSTICK_MODE = 3;
};

//* MA to PC UDP message types definitions
class TxTypeMA
{
public:
    constexpr static int SENSORS_MA = 1000;
    constexpr static int STATE_MA = 2000;
};

//* PC to MA UDP message types definitions
class RxTypeMA
{
public:
    constexpr static int ONLY_RECEIVE_DATA = 0;
    constexpr static int JOYSTICK_MODE_COMMAND = 30;
};

#endif // VILMA_MA_LABELING__HPP_