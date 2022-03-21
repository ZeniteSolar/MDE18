#include <SPI.h>
#include <mcp_can.h>
#include "can_ids.h"

#define ADC_CHANNEL_POTENTIOMETER A0
#define STEERING_WHEEL_CENTER_POSITION 128

#define CAN_SIGNATURE_SELF CAN_SIGNATURE_MDE22
#define CAN_APP_CHECKS_WITHOUT_MIC19_MSG 200
#define CAN_APP_SEND_STATE_CLK_DIV 100
#define CAN_APP_SEND_STEERING_WHEEL_POSITION_CLK_DIV 10

typedef union error_flags
{
    struct
    {
        uint8_t no_canbus : 1;
    };
    uint8_t all;
} error_flags_t;

typedef struct control
{
    uint8_t position_setpoint;
    uint8_t position_measurement;
    uint8_t position_control;
} control_t;

volatile error_flags_t error_flags;
volatile control_t control;
volatile uint16_t can_app_send_state_clk_div;
volatile uint16_t can_app_send_steering_wheel_position_clk_div;

void can_init();
void check_can();
void can_app_send_state();
void can_app_recv_mic19();
void can_app_send_state();
void can_app_send_steering_wheel_postion();
void position_control();

MCP_CAN CAN(10); // SPI CS PIN

void setup()
{
    Serial.begin(115200);
    can_init();

    pinMode(3, OUTPUT); // configura pino como saí­da
    pinMode(5, OUTPUT); // configura pino como saí­da
}

void can_init()
{
    while (CAN_OK != CAN.begin(CAN_500KBPS))
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}

void check_can()
{
    can_app_recv_mic19();

    if (can_app_send_state_clk_div++ >= CAN_APP_SEND_STATE_CLK_DIV)
    {
        Serial.println("state msg was sent.");
        can_app_send_state();
        can_app_send_state_clk_div = 0;
    }
    if (can_app_send_steering_wheel_position_clk_div++ >= CAN_APP_SEND_STEERING_WHEEL_POSITION_CLK_DIV)
    {
        Serial.println("steering wheel msg was sent.");
        can_app_send_steering_wheel_postion();
        can_app_send_steering_wheel_position_clk_div = 0;
    }
}

void can_app_send_state()
{
    uint8_t buf[CAN_MSG_GENERIC_STATE_LENGTH];

    buf[CAN_MSG_GENERIC_STATE_SIGNATURE_BYTE] = CAN_SIGNATURE_SELF;
    buf[CAN_MSG_GENERIC_STATE_STATE_BYTE] = 0; // ignoring because we are not using a finite state machine here
    buf[CAN_MSG_GENERIC_STATE_ERROR_BYTE] = error_flags.all;

    CAN.sendMsgBuf(CAN_MSG_MDE22_STATE_ID, 0, CAN_MSG_GENERIC_STATE_LENGTH, buf);
}

void can_app_send_steering_wheel_postion()
{
    uint8_t buf[CAN_MSG_MDE22_POSITION_LENGTH];

    buf[CAN_MSG_GENERIC_STATE_SIGNATURE_BYTE] = CAN_SIGNATURE_SELF;
    buf[CAN_MSG_MDE22_POSITION_SETPOINT_BYTE] = (uint8_t)control.position_setpoint;
    buf[CAN_MSG_MDE22_POSITION_MEASUREMENT_BYTE] = (uint8_t)control.position_measurement;
    buf[CAN_MSG_MDE22_POSITION_CONTROL_BYTE] = (uint8_t)control.position_control;

    CAN.sendMsgBuf(CAN_MSG_MDE22_STATE_ID, 0, CAN_MSG_GENERIC_STATE_LENGTH, buf);
}

/**
 * @brief extracts the specific MIC19 MDE message
 *
 * The msg is AAAAAAAA0000000BBBBBBBB
 * A is the Signature of module
 * B is the steering wheel position potentiometer
 *
 * @param *msg pointer to the message to be extracted
 */
void can_app_recv_mic19()
{
    uint8_t len = 0;
    uint8_t buf[CAN_MSG_MIC19_MDE_LENGTH];
    uint16_t can_app_checks_without_mic19_msg = 0;

    if (CAN_MSGAVAIL == CAN.checkReceive())
    {
        CAN.readMsgBuf(&len, buf);
        if (buf[CAN_MSG_GENERIC_STATE_SIGNATURE_BYTE] == CAN_SIGNATURE_MIC19)
        {
            if (len == CAN_MSG_MIC19_MDE_LENGTH)
            {
                can_app_checks_without_mic19_msg = 0;
                error_flags.no_canbus = 0;
                control.position_setpoint = buf[CAN_MSG_MIC19_MDE_POSITION_BYTE];
            }
        }
    }
    else if (can_app_checks_without_mic19_msg++ >= CAN_APP_CHECKS_WITHOUT_MIC19_MSG)
    {
        Serial.println("Error: too many cycles without MIC19 messages.");
        control.position_setpoint = STEERING_WHEEL_CENTER_POSITION;
        error_flags.no_canbus = 1;
    }
}

void position_control()
{
    const uint8_t position_error_zero_region = 2;        // ajuste do intervalo de zero, estabilidade na direção
    const uint8_t position_error_minimum_threshold = 85; // ajuste de potencia inicial do pwm, ajusta a potencia e velocidade do motor
    const uint8_t position_error_maximum_threshold = 90; // ajuste de potencia final do pwm, ajusta a potencia e velocidade do motor

    control.position_measurement = map(analogRead(ADC_CHANNEL_POTENTIOMETER), 0, 1024, 0, 255);
    Serial.print("motor = ");
    Serial.println(control.position_measurement);

    int16_t position_error = control.position_measurement - control.position_setpoint;
    if (position_error > position_error_zero_region)
    {
        Serial.print("positivo = ");
        Serial.println(position_error);
        if (position_error > position_error_maximum_threshold)
        {
            position_error = position_error_maximum_threshold;
        }
        control.position_control = map(position_error, 0, position_error_maximum_threshold, position_error_minimum_threshold, 255);
        analogWrite(5, 0);
        analogWrite(3, control.position_control);
    }
    else if (position_error < -position_error_zero_region)
    {
        Serial.print("negativo = ");
        position_error = position_error * (-1);
        Serial.println(position_error);
        if (position_error > position_error_maximum_threshold)
        {
            position_error = position_error_maximum_threshold;
        }
        control.position_control = map(position_error, 0, position_error_maximum_threshold, position_error_minimum_threshold, 255);
        analogWrite(5, control.position_control);
        analogWrite(3, 0);
    }
    else
    {
        analogWrite(5, 0);
        analogWrite(3, 0);
    }
}

void loop()
{
    check_can();

    position_control();

    delay(5); // Maximum 200Hz loop
}
