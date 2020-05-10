#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include "timer.h"

#define CMD_PACKAGE                         0x01
#define RESPONSE_PACKAGE					0x02

#define PACKAGE_SIZE                        512
#define CRC_WRONG                           0
#define SUCCESS_                            1
#define FAILED_                           	2
#define UNKNOW                              3
#define RUNNING_                            4

#define USB_BUFFER_MAX              1024
#define UART_PACK_SIZE_MAX           520


#define RESPOND_SUCESS              0x01
#define RESPOND_FAILED              0x02
#define RESPOND_UNKNOW              0x03    // Command is not supported
#define RESPOND_CHECKSUM_ERROR      0x04    // Checksum error

#define RESPOND_CANNOT_GOHOME       0x05
#define RESPOND_CANNOT_CALIB        0x06
#define RESPOND_CANNOT_CONTROL      0x07
#define RESPOND_SYSTEM_INIT         0x08
#define RESPOND_RUNNING             0x09

// command
#define CMD_READ_VERSION			0x02
#define CMD_JUMP_BOOTLOADER         0x15
#define CMD_START_UPGRADE           0x16
#define CMD_UPGRADE_RUNNING         0x17
#define CMD_UPGRADE_FINNIHED        0x18
#define CMD_TRANSFER_TESTING        0x19
#define CMD_CTRL_GOHOME             0x1A
#define CMD_GET_MOTOR_STATUS        0x1B
#define CMD_CHECK_PHONE_IN          0x1C
#define CMD_CTRL_MOTOR              0x1D
#define CMD_CTRL_AUTO_PROCESS       0x1E
#define CMD_CTRL_CALIB              0x1F
#define CMD_SW_WRITE_CALIB          0x20
#define CMD_SPECIAL_RELEASE_FUNC    0x21
#define CMD_READ_RANGE_SENSOR       0x22
#define CMD_CTRL_ALL_SUB_LINEAR     0x23
#define CMD_CTRL_SENSOR_POWER       0x24
#define CMD_READ_THERMAL_SENSOR     0x25



#pragma pack(1)
    typedef struct _PAYLOAD_Grippers_infor_{
		unsigned char     Status;
        char fw_version[4];
        char hw_version[4];
		char Serial_number[12];
		uint8_t gripper_No1;
		uint8_t gripper_No2;
		uint8_t gripper_No3;
		uint8_t gripper_No4;
    }PAYLOAD_Grippers_infor;

	typedef struct Cmd_usb_list{
		unsigned char   Status;
	}Cmd_usb_list;

	typedef struct USB_package_struct{
		unsigned short    Header;
		unsigned short    Length;
		unsigned char     Type;
		unsigned short    Transaction_ID;
		unsigned char     CMD_Code;
		unsigned char     Payload[UART_PACK_SIZE_MAX-8];
	}USB_package_struct, *USB_package_struct_p;

	typedef struct Payload_Control_system{
		uint8_t     Data;
	}Payload_Control_system;

	typedef enum led_status_t{
		LED_OFF = 0x01,
		LED_RED,
		LED_GREEN,
		LED_YELLOW,
		LED_BLINK_RED,
		LED_BLINK_GREEN,
        LED_BLINK_YELLOW
	}led_status_t;


	typedef struct Payload_upgrade_begin
	{
		uint32_t 	Total_length;
		uint32_t	CRC_32;
	}Payload_upgrade_begin;

	typedef struct Payload_upgrade_running
	{
		uint32_t 	Sent_length;
		uint16_t 	Length;
		uint8_t    	Data[256];
	}Payload_upgrade_running;

	// add new
	typedef struct _uart_rx_control_
	{
		uint8_t rx_buff;
		uint8_t rx_index;
		uint8_t rx_step;
		uint8_t status;
	}uart_rx_control;

	typedef enum card_get_cmd_stt_t{
		GET_HEADER0 = 0,
		GET_HEADER1,
		GET_LEN0,
		GET_LEN1,
		GET_TYPE,
		GET_TRAN_ID0,
		GET_TRAN_ID1,
		GET_OTHER
	}card_get_cmd_stt_t;

	typedef struct card_cmd_t{
		uint16_t header;
		uint16_t len;
		uint8_t type;
		uint16_t id;
		uint8_t opcode;
		uint8_t payload[512];
	}card_cmd_t;
	typedef struct card_cmd_long_t{
		uint16_t header;
		uint16_t len;
		uint8_t type;
		uint16_t id;
		uint8_t opcode;
		uint8_t payload[2053];
	}card_cmd_long_t;

	typedef enum card_stt_t{
		CARD_STT_NONE,
		CARD_STT_SEND,
		CARD_STT_GET_FINISH,
		CARD_STT_GET_EVENT_FINISH,
		CARD_STT_SUCCESS,
		CARD_STT_FAILED
	}card_stt_t;


    typedef struct _struct_check_phone_in_{
        uint8_t cmd_status;
        uint8_t gripper_no;
        uint8_t phone_status;
    }struct_check_phone_in;

    typedef struct _pl_motor_infor_{  // use for save all motor information: is power on, status, duty cycle of pwm, current motor consume
        uint8_t index;
        uint8_t status;
        uint8_t pwr;
        uint16_t duty;
        uint16_t current;
    }pl_motor_infors;

    typedef struct _struct_ctrl_motor{
        uint8_t motor_index;
        uint16_t duty;
    }struct_ctrl_motor;

    typedef struct _struct_ctrl_process_{
        uint8_t gripper;
        uint8_t step;
        uint8_t style;
    }struct_ctrl_process;

    typedef struct _struct_calib_value{
        uint8_t read_write;
        uint8_t clamp_release;
        uint16_t G1_servo_1;
        uint16_t G1_servo_2;
        uint16_t G2_servo_1;
        uint16_t G2_servo_2;
        uint16_t G3_servo_1;
        uint16_t G3_servo_2;
        uint16_t G4_servo_1;
        uint16_t G4_servo_2;
    }struct_calib_value;

    typedef struct _struct_sw_write_calib_{
        uint16_t G1_servo_1_clamp;
        uint16_t G1_servo_1_release;
        uint16_t G1_servo_2_clamp;
        uint16_t G1_servo_2_release;
        uint16_t G2_servo_1_clamp;
        uint16_t G2_servo_1_release;
        uint16_t G2_servo_2_clamp;
        uint16_t G2_servo_2_release;
        uint16_t G3_servo_1_clamp;
        uint16_t G3_servo_1_release;
        uint16_t G3_servo_2_clamp;
        uint16_t G3_servo_2_release;
        uint16_t G4_servo_1_clamp;
        uint16_t G4_servo_1_release;
        uint16_t G4_servo_2_clamp;
        uint16_t G4_servo_2_release;
    }struct_sw_write_calib;

    typedef struct _struct_special_release_{
        uint8_t gripper_index;
        uint8_t function;
    }struct_special_release;

typedef struct _struct_range_sensor_data_{
    uint8_t status;
    uint16_t distance;
}struct_range_sensor_data;
typedef struct _struct_thermal_sensor_data_{
	uint8_t status;
	uint8_t Rxbuf[2051];
}struct_thermal_sensor_data;

#pragma pack()

#define NOT_ALLOW_CTRL  0
#define ALLOW_CTRL      1
#define UNKNOW_ALLOW    2

#define GRIPPER_1           0
#define GRIPPER_2           1
#define GRIPPER_3           2
#define GRIPPER_4           3


#define PROCESS_CLAMP_MAIN    1
#define PROCESS_CLAMP_SUB     2  // CLAMP DONE
#define PROCESS_RELEASE_MAIN  3  // RELEASE DONE
#define PROCESS_RELEASE_SUB   4

#define WRITE_CALIB         0
#define READ_CALIB          1
#define CALIB_RELEASE       0
#define CALIB_CLAMP         1

#define GOHOME      1

#define SPECIAL_RELEASE_1   1
#define SPECIAL_RELEASE_2   2

	extern uart_rx_control uart_pc;	
	extern card_get_cmd_stt_t get_cmd_stt;
	extern card_cmd_t cmd_send, cmd_get,cardlong_cmd_t;
	extern card_cmd_long_t cmd_send_long;
	extern card_stt_t card_stt;
	extern uint32_t tickstart;
    extern PAYLOAD_Grippers_infor Grippers_infor;
    extern uint8_t data_sent[UART_PACK_SIZE_MAX];
	extern struct_check_phone_in check_phone_in;

    extern pl_motor_infors motor_infor[PWM_NUM_CHANNEL];
    extern pl_motor_infors pl_respond_motor_infor;
    extern struct_ctrl_motor pl_control_motor[PWM_NUM_CHANNEL];
    extern struct_ctrl_motor pl_get_ctrl_motor;

    extern uint8_t allow_control_motor[PWM_NUM_CHANNEL];
    extern struct_ctrl_process ctrl_process;
    extern struct_ctrl_process process_of_gripper[4];
    extern struct_ctrl_process Get_ctrl_proces[4];
    extern struct_ctrl_process process_of_gripper[4];
    extern struct_calib_value calib_value;
    extern struct_sw_write_calib sw_write_calib;
    extern struct_special_release pl_special_release;

    extern uint8_t check_special_release_2_done[4];

	extern	struct_thermal_sensor_data thermal_sensor;
    extern char str__[100];
	
	unsigned char USB_send_respond_package(unsigned char cmd, uint16_t ID, unsigned char status);
	void process_data_input(void);
	unsigned char Package_Calculator_checksum(unsigned char *packageBuffer, int packageSize);
	unsigned char Package_check_checksum(unsigned char *packageBuffer, int packageSize);
    void machine_init(void);
    void prepare_motor_infor_to_respond(void);
    uint8_t get_servo1_index_from_gripper_index(uint8_t gripper_index);
    uint8_t get_Gr_index_from_Mo_index(uint8_t index);
    uint8_t check_conflict_ctrl_process(void);
    void auto_process_ctrl(uint8_t motor_index);
    void calib_process(void);
    void ctrl_motor(void);

#endif
