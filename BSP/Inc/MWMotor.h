#ifndef __ODRIVEMOTOR_H
#define __ODRIVEMOTOR_H

#include <stdint.h>
#include <string.h>

#define MAX_BUS_NUM 8
#define MAX_MOTOR_NUM_PER_BUS 16

typedef enum {
    MW_ERROR_SUCCESS = 0,
    MW_ERROR_OUT_OF_RANGE = 1,
} MW_ERROR;

typedef enum {
    MW_ENDPOINT_DATA_TYPE_INT = 0,
    MW_ENDPOINT_DATA_TYPE_FLOAT = 1,
} MW_ENDPOINT_DATA_TYPE;

typedef struct {
    MW_ENDPOINT_DATA_TYPE type;
    union {
        int32_t intData;
        float floatData;
    } data;
} MW_ENDPOINT_DATA;

typedef enum {
    MW_ENDPOINT_OPCODE_READ = 0,
    MW_ENDPOINT_OPCODE_WRITE = 1,
} MW_ENDPOINT_OPCODE;

typedef struct {
    MW_ENDPOINT_OPCODE opcode;
    uint16_t id;
    MW_ENDPOINT_DATA data;
} MW_ENDPOINT;

/**
 * @brief MW_CMD_IDָ��
 */
typedef enum {
    MW_HEARTBEAT_CMD                  = 0x001,
    MW_ESTOP_CMD                      = 0x002,
    MW_GET_ERROR_CMD                  = 0x003,
    MW_RXSDO_CMD                      = 0x004,
    MW_TXSDO_CMD                      = 0x005,
    MW_SET_AXIS_NODE_ID_CMD           = 0x006,
    MW_SET_AXIS_STATE_CMD             = 0x007,
    MW_MIT_CONTROL_CMD                = 0x008,
    MW_GET_ENCODER_ESTIMATES_CMD      = 0x009,
    MW_GET_ENCODER_COUNT_CMD          = 0x00A,
    MW_SET_CONTROLLER_MODE_CMD        = 0x00B,
    MW_SET_INPUT_POS_CMD              = 0x00C,
    MW_SET_INPUT_VEL_CMD              = 0x00D,
    MW_SET_INPUT_TORQUE_CMD           = 0x00E,
    MW_SET_LIMITS_CMD                 = 0x00F,
    MW_START_ANTICOGGING_CMD          = 0x010,
    MW_SET_TRAJ_VEL_LIMIT_CMD         = 0x011,
    MW_SET_TRAJ_ACCEL_LIMIT_CMD       = 0x012,
    MW_SET_TRAJ_INERTIA_CMD           = 0x013,
    MW_GET_IQ_CMD                     = 0x014,
    MW_GET_SENSORLESS_ESTIMATES_CMD   = 0x015,
    MW_REBOOT_CMD                     = 0x016,
    MW_GET_BUS_VOLTAGE_CURRENT_CMD    = 0x017,
    MW_CLEAR_ERRORS_CMD               = 0x018,
    MW_SET_LINEAR_COUNT_CMD           = 0x019,
    MW_SET_POS_GAIN_CMD               = 0x01A,
    MW_SET_VEL_GAIN_CMD               = 0x01B,
    MW_GET_TORQUES_CMD                = 0x01C,
    MW_GET_POWERS_CMD                 = 0x01D,
    MW_DISABLE_CAN_CMD                = 0x01E,
    MW_SAVE_CONFIGURATION_CMD         = 0x01F
} MW_CMD_ID;

/**
 * @brief ���״̬
 */
typedef enum { 
    MW_AXIS_STATE_UNDEFINED                            = 0x0,
    MW_AXIS_STATE_IDLE                                 = 0x1,
    MW_AXIS_STATE_STARTUP_SEQUENCE                     = 0x2,
    MW_AXIS_STATE_FULL_CALIBRATION_SEQUENCE            = 0x3,
    MW_AXIS_STATE_MOTOR_CALIBRATION                    = 0x4,
    MW_AXIS_STATE_ENCODER_INDEX_SEARCH                 = 0x6,
    MW_AXIS_STATE_ENCODER_OFFSET_CALIBRATION           = 0x7,
    MW_AXIS_STATE_CLOSED_LOOP_CONTROL                  = 0x8,
    MW_AXIS_STATE_LOCKIN_SPIN                          = 0x9,
    MW_AXIS_STATE_ENCODER_DIR_FIND                     = 0xA,
    MW_AXIS_STATE_HOMING                               = 0xB,
    MW_AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION    = 0xC,
    MW_AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION       = 0XD,
    MW_AXIS_STATE_ANTICOGGING_CALIBRATION              = 0XE
} MW_MOTER_STATE;

/**
 * @brief �������ģʽ
 */
typedef enum { 
    MW_VOLTAGE_CONTROL          = 0,
    MW_TORQUE_CONTROL           = 1,
    MW_VELOCITY_CONTROL         = 2,
    MW_POSITION_CONTROL         = 3
} MW_CONTROL_MODE;

/**
 * @brief �������ģʽ
 */
typedef enum { 
    MW_IDLE_INPUT               = 0,
    MW_DIRECT_CONTROL_INPUT     = 1,
    MW_RAMP_RATE_INPUT          = 2,
    MW_POSITION_FILTERING_INPUT = 3,
    MW_TRAPEZOIDAL_CURVE_INPUT  = 5,
    MW_TORQUE_RAMP_INPUT        = 6,
    MW_MIT_INPUT                = 9
} MW_INPUT_MODE;

/**
 * @brief �����������
 */
typedef enum {
    MW_MOTOR_ERROR = 0,
    MW_ENCODER_ERROR = 1,
    MW_SENSORLESS_ERROR = 2,
    MW_CONTROLLER_ERROR = 3,
} MW_ERROR_TYPE;

/**
 * @brief MW���MITģʽ���ݽṹ��
 */
typedef struct {
    double targetPos;
    double ffVel;
    double kp;
    double kd;
    double ffTorque;
} MW_MIT_CTRL_INPUT;

/**
 * @brief MW������ݽṹ��
 */
typedef struct {
    uint8_t busId;                       //!<@brief ����������
    uint8_t nodeId;                      //!<@brief �������ڵ�
    
    struct {                                  
        uint32_t axisError;              //!<@brief �޸��쳣λ
        uint8_t motorErrorFlag;          //!<@brief ����쳣λ
        uint8_t encoderErrorFlag;        //!<@brief �������쳣λ
        uint8_t controllerErrorFlag;     //!<@brief �������쳣λ
    } ErrorStatus;                       //!<@brief �쳣��־λ
    uint32_t error;                      //!<@brief �쳣��ȡ
    uint8_t currentState;                //!<@brief ��ǰ���״̬
                                           
    float encoderPosEstimate;            //!<@brief ������λ��Ԥ��ֵ
    float encoderVelEstimate;            //!<@brief �������ٶ�Ԥ��ֵ
    float sensorlessPosEstimate;         //!<@brief ���໷����Ƕ�
    float sensorlessVelEstimate;         //!<@brief ���໷�ٶ�
    float torque;                        //!<@brief ʵ������
    float torqueSetpoint;                //!<@brief ת�ؿ�����ʵ��ʹ�õ�ת�زο�
    float torqueConstant;                //!<@brief ת�س���
    int32_t shadowCount;                 //!<@brief ÿת������
    int32_t countInCPR;                  //!<@brief ÿת����(CPR)
    float iqSetpoint;                    //!<@brief ����������ò���
    float iqMeasured;                    //!<@brief ��������
                                           
    float busVoltage;                    //!<@brief ��Դ��ѹ
    float busCurrent;                    //!<@brief ��Դ����
    float electricalPower;               //!<@brief �������
    float mechanicalPower;               //!<@brief �����е����
    float motorTemperature;              //!<@brief ����¶�
    float fetTemperature;                //!<@brief �������¶�
                                           
    MW_MIT_CTRL_INPUT motorMIT;          //!<@brief MITģʽ�¿��Ʒ��ز���
                                           
    uint8_t controlMode;                 //!<@brief �������ģʽ
    uint8_t inputMode;                   //!<@brief �������ģʽ
    float velGain;                       //!<@brief �ٶȻ�PID���Ƶ�Pֵ
    float velIntegratorGain;             //!<@brief �ٶȻ�PID���Ƶ�Iֵ
    float posGain;                       //!<@brief λ�û�PID���Ƶ�Pֵ
                                      
    MW_ENDPOINT EndpointData;
} MW_MOTOR_DATA;

typedef void (*MotorSender)(uint8_t busId, uint8_t canId, uint8_t *data, uint8_t dataSize);
typedef void (*MotorNotifier)(uint8_t busId, uint8_t nodeId, MW_CMD_ID cmdId);

typedef struct {
    uint8_t busId;
    uint8_t nodeId;
    MW_MOTOR_DATA *motorData;
    MotorSender sender;
    MotorNotifier notifier;
} MWMotorAccessInfo;


/* �������߹ҵ����е��������Ϣ */
extern MWMotorAccessInfo motors[MAX_BUS_NUM][MAX_MOTOR_NUM_PER_BUS];

/**
 * ע����
 * 
 * ������������ϵͳ��ע��һ�������������뵽ָ�������ߺͽڵ��ϡ�
 * ע��ɹ�����MW_ERROR_SUCCESS�����򷵻�MW_ERROR_OUT_OF_RANGE��
 * 
 * @param motor һ���ṹ�壬�������������ID�ͽڵ�ID���Լ�����ķ�����Ϣ��
 * @return ������������ID��ڵ�ID������Χ���򷵻�MW_ERROR_OUT_OF_RANGE��
 *         ���򷵻�MW_ERROR_SUCCESS����ʾע��ɹ���
 */
uint8_t MWRegisterMotor(MWMotorAccessInfo motor);

/**
 * @brief �û��ĳ�������Ҫ������ߺ�����������յ������MW�����CAN��Ϣ��
 * 
 * �˺�������CAN��Ϣ��ID�����ݣ�ִ����Ӧ�Ĵ����߼���������CAN��Ϣ�еĽڵ�ID������ID��
 * ��������Щ��Ϣ������Ӧ�Ĵ���������������Ǵ���MW�������ϵͳ��CANͨ�ŵĺ��ĺ�����
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param canId CAN��Ϣ��ID������ʶ��ڵ�ID������ID��
 * @param data CAN��Ϣ�����ݲ��֣���������ľ������ݡ�
 */
void MWReceiver(uint8_t busId, uint32_t canId, uint8_t *data);

/**
 * ����ָֹͣ�����ߺͽڵ��ϵĵ��
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @note cmd_id: 0x002
 */
void MWEstop(uint8_t busId, uint8_t nodeId);

/**
 * @brief ��ȡ���������Ϣ
 * 
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param errorType �������ͣ�ָ��Ҫ��ѯ�Ĵ�������
 * @note cmd_id: 0x003
 */
void MWGetMotorError(uint8_t busId, uint8_t nodeId, MW_ERROR_TYPE errorType);

/**
 * ���ʵ��������������˵����ݣ���ν�˵�������ָ�����״̬����������������ݡ�
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param endpointData �˵����ݽṹ�壬���������롢�˵�ID������
 * @note cmd_id: 0x004
 */
void MWRxSdo(uint8_t busId, uint8_t nodeId, MW_ENDPOINT endpointData);

/**
 * ���õ���Ľڵ�ID
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param newNodeId �µĽڵ�ID������������Ϊ������µĽڵ�ID��
 * @param motorInfo ���������Ϣ��
 * @note cmd_id: 0x006
 */
void MWSetAxisNodeID(uint8_t busId, uint8_t nodeId, uint8_t newNodeId, MWMotorAccessInfo *motorInfo);

/**
 * ���õ��״̬
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param state �������״̬������״̬�������ϲ�Ӧ�ö��塣
 * @note cmd_id: 0x007
 */
void MWSetAxisState(uint8_t busId, uint8_t nodeId, MW_MOTER_STATE state);

/**
 * MIT�˶�����
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param mit �����������������Ŀ��λ�á�ǰ���ٶȡ�ǰ�����ء��������Ϣ
 * @note cmd_id: 0x008
 */
void MWMitControl(uint8_t busId, uint8_t nodeId, MW_MIT_CTRL_INPUT *mit);

/**
 * ��ȡ���������ٶȺ�λ����Ϣ
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������\
 * @note cmd_id: 0x009
 */
void MWGetEncoderEstimates(uint8_t busId, uint8_t nodeId);

/**
 * ���õ���������Ŀ���ģʽ������ģʽ��
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param ctrlMode ������ģʽ��ָ������������Ĺ���ģʽ��
 * @param inputMode ����ģʽ��ָ�����������������ģʽ��
 * @note cmd_id: 0x00B
 */
void MWSetControllerMode(uint8_t busId, uint8_t nodeId, MW_CONTROL_MODE ctrlMode, MW_INPUT_MODE inputMode);

/**
 * λ�ÿ��ƣ�������λ�á��ٶ�ǰ��������ǰ����
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param inputPos ���������λ�ã�ͨ�������ⲿ�������ṩ������λ�ã���λturns��
 * @param velFF �ٶ�ǰ��ֵ�����ڲ������ϵͳ�Ĺ��Ի���������λ0.001turns/s��
 * @param torqueFF ����ǰ��ֵ�����ڲ������ϵͳ��Ħ��������Ӱ�죬��λ0.001Nm��
 * @note cmd_id: 0x00C
 */
void MWPosControl(uint8_t busId, uint8_t nodeId, float inputPos, int16_t velFF, int16_t torqueFF);

/**
 * @brief �ٶȿ��ƣ��������ٶȺ�����ǰ����
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param inputVel ���������ٶȣ���λturns/s��
 * @param torqueFF Ť��ǰ��ֵ�����ڲ�������ǿ�����Ť�أ���λNm��
 * @note cmd_id: 0x00D
 */
void MWVelControl(uint8_t busId, uint8_t nodeId, float inputVel, float torqueFF);

/**
 * @brief ���ؿ���
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param inputTorque ����Ť��ֵ����λNm��
 * @note cmd_id: 0x00E
 */
void MWTorqueControl(uint8_t busId, uint8_t nodeId, float inputTorque);

/**
 * ���õ�����ٺ�����
 * 
 * ����������ͨ��ָ��������ID�͵���ڵ�ID�����øõ��������ٶ����ƺ����������ơ�
 * ��Щ�������ڱ����������ص�·��������ߵ��ٶȻ���������𻵡�
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param velLim ������ٶ����ƣ���λturns/s��
 * @param currLim ����ĵ������ƣ���λΪ���ࣨA����
 * @note cmd_id: 0x00F
 */
void MWSetLimits(uint8_t busId, uint8_t nodeId, float velLim, float currLim);

/**
 * @brief ���������Ʋ�У׼
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @note cmd_id: 0x010
 */
void MWStartAnticogging(uint8_t busId, uint8_t nodeId);

/**
 * @brief ������������λ�ÿ��ƻ����ٶ�����
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param velLim �����ٶ����ֵ
 * @note cmd_id: 0x011
 */
void MWSetTrajVelLimit(uint8_t busId, uint8_t nodeId, float velLim);

/**
 * @brief ������������λ�ÿ��Ƽ��ٶ�����
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param accLim ���ٶ����ֵ
 * @param decLim ���ٶ����ֵ
 * @note cmd_id: 0x012
 */
void MWSetTrajAccelLimits(uint8_t busId, uint8_t nodeId, float accLim, float decLim);

/**
 * @brief ������������λ�ÿ��ƹ���
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param inertia ����
 * @note cmd_id: 0x013
 */
void MWSetTrajInertia(uint8_t busId, uint8_t nodeId, float inertia);

/**
 * @brief �������
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @note cmd_id: 0x016
 */
void MWReboot(uint8_t busId, uint8_t nodeId);

/**
 * @brief ������д���
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @note cmd_id: 0x018
 */
void MWClearErrors(uint8_t busId, uint8_t nodeId);

/**
* @brief ���õ��λ�û�PID���Ƶ�Pֵ

 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param posgain λ�û�PID���Ƶ�Pֵ
 * @note cmd_id: 0x01A
 */
void MWSetPosGain(uint8_t busId, uint8_t nodeId, float posGain);

/**
* @brief �����ٶȻ�PID���Ƶ�Pֵ��Iֵ

 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @param velGain λ�û�PID���Ƶ�Pֵ
 * @param velIntGain λ�û�PID���Ƶ�Iֵ
 * @note cmd_id: 0x01B
 */
void MWSetVelGain(uint8_t busId, uint8_t nodeId, float velGain, float velIntGain);

/**
 * @brief ����CAN
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @note cmd_id: 0x01E
 */
void MWDisableCAN(uint8_t busId, uint8_t nodeId);

/**
 * @brief ���������ò��������
 * 
 * @param busId ����������ߵ�ID������ID����ʶ��������ϵͳ�еĲ�ͬ���ߡ�
 * @param nodeId ����������ϵĽڵ�ID���ڵ�ID������������Ψһ��ʶÿ�������
 * @note cmd_id: 0x01F
 */
void MWSaveConfigeration(uint8_t busId, uint8_t nodeId);

#endif
