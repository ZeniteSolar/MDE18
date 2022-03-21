#include <SPI.h>
#include <mcp_can.h>
#include "can_ids.h"

const uint8_t position_center_value = 128;           // ajuste do centro do potenciômetro da popa
const uint8_t position_error_zero_region = 2;        // ajuste do intervalo de zero, estabilidade na direção
const uint8_t position_error_minimum_threshold = 85; // ajuste de potencia inicial do pwm, ajusta a potencia e velocidade do motor
const uint8_t position_error_maximum_threshold = 90; // ajuste de potencia final do pwm, ajusta a potencia e velocidade do motor

#define ADC_CHANNEL_POTENTIOMETER A0
#define MOTOR_FORWARD_PIN 5
#define MOTOR_BACKWARD_PIN 3

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

    pinMode(MOTOR_FORWARD_PIN, OUTPUT); // configura pino como saí­da
    pinMode(MOTOR_BACKWARD_PIN, OUTPUT); // configura pino como saí­da

    Serial.println("Position Controller Configurations: ");
    Serial.print("\t position_center_value = ");
    Serial.println(position_center_value);
    Serial.print("\t position_error_zero_region = ");
    Serial.println(position_error_zero_region);
    Serial.print("\t position_error_minimum_threshold = ");
    Serial.println(position_error_minimum_threshold);
    Serial.print("\t position_error_maximum_threshold = ");
    Serial.println(position_error_maximum_threshold);
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

    if (CAN_MSGAVAIL != CAN.checkReceive())
    {
        CAN.readMsgBuf(&len, buf);
        if (buf[CAN_MSG_GENERIC_STATE_SIGNATURE_BYTE] != CAN_SIGNATURE_MIC19){
            Serial.println("CAN: wrong SIGNATURE on received message.");
            return;
        }
        if (len != CAN_MSG_MIC19_MDE_LENGTH){
            Serial.println("CAN: wrong message length.");
        }

        Serial.print("Received a MIC19 MDE message: [");
        for (uint8_t i = 0; i < len; i++)
        {
            Serial.print(buf[i]);
            if (i != len)
                Serial.print(", ");
        }
        Serial.println("]");

        can_app_checks_without_mic19_msg = 0;
        error_flags.no_canbus = 0;
        control.position_setpoint = buf[CAN_MSG_MIC19_MDE_POSITION_BYTE];
    }
    else
    {
        control.position_setpoint = position_center_value;
        // Serial.print("Fail: no MIC19 MDE message. setpoint is now: ");
        // Serial.println(control.position_setpoint);
    }

    if (can_app_checks_without_mic19_msg++ >= CAN_APP_CHECKS_WITHOUT_MIC19_MSG)
    {
        Serial.println("Error: too many cycles without MIC19 messages.");
        error_flags.no_canbus = 1;
    }
}

void position_control()
{
    Serial.print("Position { setpoint: ");
    Serial.print(control.position_setpoint);

    control.position_measurement = map(analogRead(ADC_CHANNEL_POTENTIOMETER), 0, 1024, 0, 255);
    Serial.print(", measurement: ");
    Serial.print(control.position_measurement);

    int16_t position_error = control.position_measurement - control.position_setpoint;
    Serial.print(", error: ");
    Serial.print(position_error);
    if (position_error > position_error_zero_region)
    {
        if (position_error > position_error_maximum_threshold)
            position_error = position_error_maximum_threshold;
        control.position_control = map(position_error, 0, position_error_maximum_threshold, position_error_minimum_threshold, 255);
        analogWrite(MOTOR_FORWARD_PIN, 0);
        analogWrite(MOTOR_BACKWARD_PIN, control.position_control);
    }
    else if (position_error < -position_error_zero_region)
    {
        position_error *= -1;
        if (position_error > position_error_maximum_threshold)
            position_error = position_error_maximum_threshold;
        control.position_control = map(position_error, 0, position_error_maximum_threshold, position_error_minimum_threshold, 255);
        analogWrite(MOTOR_FORWARD_PIN, control.position_control);
        analogWrite(MOTOR_BACKWARD_PIN, 0);
    }
    else
    {
        control.position_control = 0;
        analogWrite(MOTOR_FORWARD_PIN, 0);
        analogWrite(MOTOR_BACKWARD_PIN, 0);
    }
    Serial.print(", control: ");
    Serial.print(control.position_control);

    Serial.println(" }");
}

void loop()
{
    check_can();

    position_control();

    delay(5); // Maximum 200Hz loop
}
