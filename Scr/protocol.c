/*------------------------------------------------------------------------------
 *
 *
 *------------------------------------------------------------------------------
 * Name:
 * Purpose:
 *----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
//#include <rl_usb.h>
#include "uart_debug.h"
#include "main.h"
#include "protocol.h"
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "timer.h"
#include "adc.h"



#define HEADER0         0x55
#define HEADER1         0xFF
#define TIMEOUT_GET_BYTE     200

USB_package_struct  USB_Input_packet;
USB_package_struct  USB_Output_packet;

uart_rx_control uart_pc;
card_get_cmd_stt_t get_cmd_stt;
card_cmd_t cmd_send, cmd_get, cmd_process;
card_cmd_long_t cmd_send_long;
struct_check_phone_in check_phone_in;

pl_motor_infors motor_infor[PWM_NUM_CHANNEL];
pl_motor_infors pl_respond_motor_infor;
struct_ctrl_motor pl_control_motor[PWM_NUM_CHANNEL];
struct_ctrl_motor pl_get_ctrl_motor;

uint8_t allow_control_motor[PWM_NUM_CHANNEL];
struct_ctrl_process ctrl_process;
struct_ctrl_process Get_ctrl_proces[4];
struct_ctrl_process process_of_gripper[4];
struct_calib_value calib_value;
struct_sw_write_calib sw_write_calib;
struct_special_release pl_special_release;
struct_range_sensor_data range_sensor;
struct_thermal_sensor_data thermal_sensor;

uint16_t getIndex, sizeSend;
card_stt_t card_stt;
uint32_t tickstart, ticksByte;
uint8_t rxBuf;
uint8_t doSetting, valueReg00, valueReg01, valueReg02;

uint8_t check_special_release_2_done[4];

char str__[100];
uint8_t data_sent[UART_PACK_SIZE_MAX];
uint8_t data_sent_long[2061];

PAYLOAD_Grippers_infor      Grippers_infor;
Payload_upgrade_begin       Upgrade_begin;
Payload_upgrade_running     Upgrade_running;

bool IsFirstTimeGoHome = true;

void machine_init(void){
    char tmp[15];
    start_process = 0;
    start_gohome = 0;
    check_special_release_2_done[0] = check_special_release_2_done[1] = check_special_release_2_done[2] = check_special_release_2_done[3] = 0;
    check_on_auto_process[GRIPPER_1] = 0;  check_on_auto_process[GRIPPER_2] = 0;  check_on_auto_process[GRIPPER_3] = 0;  check_on_auto_process[GRIPPER_4] = 0;
    start_auto_process[GRIPPER_1] = STOP_AUTO_PROCESS;   start_auto_process[GRIPPER_2] = STOP_AUTO_PROCESS;
    start_auto_process[GRIPPER_3] = STOP_AUTO_PROCESS;   start_auto_process[GRIPPER_4] = STOP_AUTO_PROCESS;
    Get_ctrl_proces[GRIPPER_1].style = STYLE_BIG_PHONE;   Get_ctrl_proces[GRIPPER_2].style = STYLE_BIG_PHONE;
    Get_ctrl_proces[GRIPPER_3].style = STYLE_BIG_PHONE;   Get_ctrl_proces[GRIPPER_4].style = STYLE_BIG_PHONE;
    // init information
    strncpy(Grippers_infor.fw_version, "1.12", 4);
    get_hw_version();

    // Serial Number
    sprintf(tmp, "%02X%02X%02X%02X%02X%02X",
        system_getUnique8(11)^	/* MSB */
        system_getUnique8(10),
        system_getUnique8(9)^
        system_getUnique8(8),
        system_getUnique8(7)^
        system_getUnique8(6),
        system_getUnique8(5)^
        system_getUnique8(4),
        system_getUnique8(3)^
        system_getUnique8(2),
        system_getUnique8(1)^
        system_getUnique8(0)	/* LSB */
    );
    strncpy(Grippers_infor.Serial_number, tmp, 12);

    send_debugging_data("machine init is OK\n\r");
    HAL_UART_Receive_IT(&usart1_pc_handel, &rxBuf, 1);
}


unsigned char Package_Calculator_checksum(unsigned char *packageBuffer, int packageSize){
    unsigned char result;
    int i;
    result = 0;
    for(i = 0; i<packageSize; i++){
        result += packageBuffer[i];
    }
    return(~result + 1);
}

unsigned char Package_check_checksum(unsigned char *packageBuffer, int packageSize){
    unsigned char result;
    int i;
    result = 0;
    for(i = 0; i < (packageSize); i++){
        result += packageBuffer[i];
    }
    return result;
}

unsigned char USB_send_respond_package(unsigned char cmd, uint16_t ID, unsigned char status)
{
    unsigned char ck;

    if(status == RESPOND_CHECKSUM_ERROR){
        cmd_send.payload[0] = RESPOND_CHECKSUM_ERROR;
        cmd_send.len = 7;
    }
    else
    {
        switch(cmd){//USB_Input_packet.CMD_Code){
        case CMD_READ_VERSION:
            if(check_system_init == 0)
                Grippers_infor.Status = RESPOND_SYSTEM_INIT;
            else
                Grippers_infor.Status = RESPOND_SUCESS;

            memcpy(cmd_send.payload, &Grippers_infor, sizeof(Grippers_infor));
            cmd_send.len = 6 + sizeof(Grippers_infor);
            break;
        case CMD_JUMP_BOOTLOADER:
            cmd_send.payload[0] = status;
            cmd_send.len = 7;
            break;
        case CMD_TRANSFER_TESTING:
            memcpy(&cmd_send, &cmd_process, sizeof(cmd_process));
            break;
        case CMD_CHECK_PHONE_IN:
            check_phone_in.cmd_status = status;
            check_phone_in.gripper_no = cmd_process.payload[0];
            if(cmd_process.payload[0] == GRIPPER_1){
                check_phone_in.phone_status = Grippers_infor.gripper_No1;
            }
            else if(cmd_process.payload[0] == GRIPPER_2){
                check_phone_in.phone_status = Grippers_infor.gripper_No2;
            }
            else if(cmd_process.payload[0] == GRIPPER_3){
                check_phone_in.phone_status = Grippers_infor.gripper_No3;
            }
            else{ // GRIPPER_4
                check_phone_in.phone_status = Grippers_infor.gripper_No4;
            }

            memcpy(cmd_send.payload, &check_phone_in, sizeof(check_phone_in));
            cmd_send.len = 6 + sizeof(check_phone_in);
            break;
        case CMD_CTRL_MOTOR:
            cmd_send.payload[0] = status;
            cmd_send.len = 7;
            break;
        case CMD_GET_MOTOR_STATUS:
            cmd_send.payload[0] = status;
            memcpy(&cmd_send.payload[1], &pl_respond_motor_infor, sizeof(pl_respond_motor_infor));
            cmd_send.len = 6 + 1 +  sizeof(pl_respond_motor_infor);
            break;
        case CMD_CTRL_AUTO_PROCESS:
            cmd_send.payload[0] = status;
            cmd_send.len = 7;
            break;
        case CMD_CTRL_CALIB:
            cmd_send.payload[0] = status;
            memcpy(&cmd_send.payload[1], &calib_value, sizeof(calib_value));
            cmd_send.len = 6 + 1 + sizeof(calib_value);
            break;
        case CMD_CTRL_GOHOME:
            cmd_send.payload[0] = status;
            cmd_send.len = 7;
            break;
        case CMD_SW_WRITE_CALIB:
            cmd_send.payload[0] = status;
            cmd_send.len = 7;
            break;
        case CMD_SPECIAL_RELEASE_FUNC:
            cmd_send.payload[0] = status;
            cmd_send.len = 7;
            break;
        case CMD_READ_RANGE_SENSOR:
            memcpy(cmd_send.payload, &range_sensor, sizeof(range_sensor));
            cmd_send.len = 6 + sizeof(range_sensor);
            break;
		case CMD_READ_THERMAL_SENSOR:
             memcpy(cmd_send_long.payload, &thermal_sensor, sizeof(thermal_sensor));
            cmd_send_long.len = 6 + sizeof(thermal_sensor);
            break;
		case CMD_CTRL_ALL_SUB_LINEAR:
            cmd_send.payload[0] = status;
            cmd_send.len = 7;
            break;
        case CMD_CTRL_SENSOR_POWER:
            cmd_send.payload[0] = status;
            cmd_send.len = 7;
            break;
        default:
            cmd_send.payload[0] = FAILED_;
            cmd_send.len = 7;
            break;
        }
    }
	if(cmd == CMD_READ_THERMAL_SENSOR)
	{
		cmd_send_long.header = 0xFF55;
		cmd_send_long.type  = RESPONSE_PACKAGE;
		cmd_send_long.id = cmd_process.id;
		cmd_send_long.opcode = cmd;
		if(cmd_send_long.len > (6+sizeof(thermal_sensor))) return FAILED_;
		ck = Package_Calculator_checksum((unsigned char*)&cmd_send_long + 2, cmd_send_long.len);
		memcpy(data_sent_long, &cmd_send_long, cmd_send_long.len+3);
		data_sent_long[cmd_send_long.len+2] = ck;
		send_response_data(data_sent_long, cmd_send_long.len+3);

	}
	else
	{
		cmd_send.header = 0xFF55;
		cmd_send.type  = RESPONSE_PACKAGE;
		cmd_send.id = cmd_process.id;
		cmd_send.opcode = cmd;
		if(cmd_send.len > UART_PACK_SIZE_MAX) return FAILED_;
		ck = Package_Calculator_checksum((unsigned char*)&cmd_send + 2, cmd_send.len);
		memcpy(data_sent, &cmd_send, cmd_send.len+3);
		data_sent[cmd_send.len+2] = ck;

		send_response_data(data_sent, cmd_send.len+3);
	}


    return SUCCESS_;

}
void process_data_input(void)
{
    uint8_t i = 0;
    if(cmd_process.header != 0xFF55)
    {
        return;
    }
    if(cmd_process.len > UART_PACK_SIZE_MAX)
    {
        return;
    }
    if(Package_check_checksum((unsigned char*)&cmd_process + 2, cmd_process.len+1) != 0)
    {
        send_debugging_data("check sum package error\n\r");
        USB_send_respond_package(cmd_process.opcode, cmd_process.id, RESPOND_CHECKSUM_ERROR);
        return;
    }

    switch(cmd_process.opcode){
    case CMD_READ_VERSION:
        USB_send_respond_package(CMD_READ_VERSION, cmd_process.id, RESPOND_SUCESS);
        break;
    case CMD_JUMP_BOOTLOADER:
        USB_send_respond_package(CMD_JUMP_BOOTLOADER, cmd_process.id, RESPOND_SUCESS);
        send_debugging_data("CMD_JUMP_BOOTLOADER\n\r");

        osDelay(300);
        system_resetMCU();
        break;
    case CMD_TRANSFER_TESTING:
        if((cmd_process.payload[cmd_process.len - 8] == 0xBB) && (cmd_process.payload[cmd_process.len - 7] == 0xAA))
            USB_send_respond_package(CMD_TRANSFER_TESTING, cmd_process.id, RESPOND_SUCESS);
        else
            USB_send_respond_package(0xCC, cmd_process.id, 0xDD);
        break;
    case CMD_CHECK_PHONE_IN:
        USB_send_respond_package(CMD_CHECK_PHONE_IN, cmd_process.id, RESPOND_SUCESS);
        break;
    case CMD_CTRL_MOTOR:
        memcpy(&pl_get_ctrl_motor, cmd_process.payload, sizeof(pl_get_ctrl_motor));

        if(pl_get_ctrl_motor.duty != pl_control_motor[pl_get_ctrl_motor.motor_index].duty){
            if( check_is_motor_linear1(pl_get_ctrl_motor.motor_index) == SUCCESS_){
//                check_allow_ctr_motor_sub_linear();
//                if(allow_control_motor[pl_get_ctrl_motor.motor_index] == ALLOW_CTRL){
                    pl_control_motor[pl_get_ctrl_motor.motor_index].motor_index = pl_get_ctrl_motor.motor_index;
                    pl_control_motor[pl_get_ctrl_motor.motor_index].duty = pl_get_ctrl_motor.duty;
                    motor_step[pl_get_ctrl_motor.motor_index] = STEP_1;
                    USB_send_respond_package(CMD_CTRL_MOTOR, cmd_process.id, RESPOND_SUCESS);
//                }
//                else{
//                    USB_send_respond_package(CMD_CTRL_MOTOR, cmd_process.id, RESPOND_FAILED);
//                }
            }
            else{
                pl_control_motor[pl_get_ctrl_motor.motor_index].motor_index = pl_get_ctrl_motor.motor_index;
                pl_control_motor[pl_get_ctrl_motor.motor_index].duty = pl_get_ctrl_motor.duty;
                motor_step[pl_get_ctrl_motor.motor_index] = STEP_1;
                USB_send_respond_package(CMD_CTRL_MOTOR, cmd_process.id, RESPOND_SUCESS);
            }
        }
        else
            USB_send_respond_package(CMD_CTRL_MOTOR, cmd_process.id, RESPOND_FAILED);

        break;
    case CMD_GET_MOTOR_STATUS:
        pl_respond_motor_infor.index = cmd_process.payload[0];
        prepare_motor_infor_to_respond();
        USB_send_respond_package(CMD_GET_MOTOR_STATUS, cmd_process.id, RESPOND_SUCESS);
        break;
    case CMD_CTRL_AUTO_PROCESS:
        memcpy(&ctrl_process, cmd_process.payload, sizeof(ctrl_process));
        if(ctrl_process.style >= STYLE_BIG_PHONE)
            ctrl_process.style = STYLE_BIG_PHONE;
        // check can be control: grippers must be gone home before control
        if(check_calib_gohome_ctrl_step != CONTROL){
            USB_send_respond_package(CMD_CTRL_AUTO_PROCESS, cmd_process.id, RESPOND_CANNOT_CONTROL);
            return;
        }
        else{
            // check conflict
            if(check_conflict_ctrl_process() == UNKNOW){
                USB_send_respond_package(CMD_CTRL_AUTO_PROCESS, cmd_process.id, RESPOND_UNKNOW);
            }
            else if(check_conflict_ctrl_process() == FAILED_){
                USB_send_respond_package(CMD_CTRL_AUTO_PROCESS, cmd_process.id, RESPOND_FAILED);
            }
            else if(check_conflict_ctrl_process() == RUNNING_){
                USB_send_respond_package(CMD_CTRL_AUTO_PROCESS, cmd_process.id, RESPOND_RUNNING);
            }
            else{
                USB_send_respond_package(CMD_CTRL_AUTO_PROCESS, cmd_process.id, RESPOND_SUCESS);

                if(ctrl_process.step == PROCESS_RELEASE_SUB){
                    if(process_of_gripper[ctrl_process.gripper].step != PROCESS_CLAMP_SUB){
                        break;
                    }
                }

                if(ctrl_process.step == PROCESS_CLAMP_SUB){
                    if(process_of_gripper[ctrl_process.gripper].step == PROCESS_RELEASE_MAIN){
                        break;
                    }
                    if(process_of_gripper[ctrl_process.gripper].step == PROCESS_CLAMP_SUB){
                        break;
                    }
                }

                send_debugging_data("CMD_CTRL_AUTO_PROCESS start\n\r");
                // process
                Get_ctrl_proces[ctrl_process.gripper].gripper = ctrl_process.gripper;
                Get_ctrl_proces[ctrl_process.gripper].step = ctrl_process.step;
                Get_ctrl_proces[ctrl_process.gripper].style = ctrl_process.style;
                check_on_auto_process[ctrl_process.gripper] = 1;
                start_auto_process[ctrl_process.gripper] = START_AUTO_PROCESS;
            }
        }
        break;
    case CMD_CTRL_CALIB:
        memcpy(&calib_value, cmd_process.payload, sizeof(calib_value));
        // check can be calib function: if any gripper has phone in, the calib function can't do
        if((Grippers_infor.gripper_No1 == HAVE_PHONE)||(Grippers_infor.gripper_No2 == HAVE_PHONE)||\
                (Grippers_infor.gripper_No3 == HAVE_PHONE)||(Grippers_infor.gripper_No4 == HAVE_PHONE)){
            USB_send_respond_package(CMD_CTRL_CALIB, cmd_process.id, RESPOND_CANNOT_CALIB);
        }
        else{
            calib_process();
            USB_send_respond_package(CMD_CTRL_CALIB, cmd_process.id, RESPOND_SUCESS);
            ctrl_motor();
        }
        break;
    case CMD_CTRL_GOHOME:
        if((check_calib_gohome_ctrl_step == CALIB)){
            USB_send_respond_package(CMD_CTRL_GOHOME, cmd_process.id, RESPOND_CANNOT_GOHOME);
        }
        else{
            USB_send_respond_package(CMD_CTRL_GOHOME, cmd_process.id, RESPOND_SUCESS);
            start_gohome = GOHOME;
            ctrl_motor();
            check_calib_gohome_ctrl_step = CONTROL;
        }
        break;
    case CMD_SW_WRITE_CALIB:
        memcpy(&sw_write_calib, cmd_process.payload, sizeof(sw_write_calib));

        clamp_duty_cycle[G1_SERVO_1] = sw_write_calib.G1_servo_1_clamp;
        clamp_duty_cycle[G1_SERVO_2] = sw_write_calib.G1_servo_2_clamp;
        clamp_duty_cycle[G2_SERVO_1] = sw_write_calib.G2_servo_1_clamp;
        clamp_duty_cycle[G2_SERVO_2] = sw_write_calib.G2_servo_2_clamp;
        clamp_duty_cycle[G3_SERVO_1] = sw_write_calib.G3_servo_1_clamp;
        clamp_duty_cycle[G3_SERVO_2] = sw_write_calib.G3_servo_2_clamp;
        clamp_duty_cycle[G4_SERVO_1] = sw_write_calib.G4_servo_1_clamp;
        clamp_duty_cycle[G4_SERVO_2] = sw_write_calib.G4_servo_2_clamp;

        release_duty_cycle[G1_SERVO_1] = sw_write_calib.G1_servo_1_release;
        release_duty_cycle[G1_SERVO_2] = sw_write_calib.G1_servo_2_release;
        release_duty_cycle[G2_SERVO_1] = sw_write_calib.G2_servo_1_release;
        release_duty_cycle[G2_SERVO_2] = sw_write_calib.G2_servo_2_release;
        release_duty_cycle[G3_SERVO_1] = sw_write_calib.G3_servo_1_release;
        release_duty_cycle[G3_SERVO_2] = sw_write_calib.G3_servo_2_release;
        release_duty_cycle[G4_SERVO_1] = sw_write_calib.G4_servo_1_release;
        release_duty_cycle[G4_SERVO_2] = sw_write_calib.G4_servo_2_release;

        check_calib_gohome_ctrl_step = GO_HOME;
        USB_send_respond_package(CMD_SW_WRITE_CALIB, cmd_process.id, RESPOND_SUCESS);
        break;
    case CMD_SPECIAL_RELEASE_FUNC:
        memcpy(&pl_special_release, cmd_process.payload, sizeof(pl_special_release));
        if((pl_special_release.function == SPECIAL_RELEASE_1)&&(check_special_release_2_done[pl_special_release.gripper_index] ==0)){
            USB_send_respond_package(CMD_SPECIAL_RELEASE_FUNC, cmd_process.id, RESPOND_FAILED);
        }
        else if((pl_special_release.function == SPECIAL_RELEASE_1)&&(check_special_release_2_done[pl_special_release.gripper_index] == 1)){
            // do linear 2: motor 9.11.13.15
            Get_ctrl_proces[pl_special_release.gripper_index].gripper = pl_special_release.gripper_index;
            Get_ctrl_proces[pl_special_release.gripper_index].step = PROCESS_RELEASE_MAIN;

            start_auto_process[pl_special_release.gripper_index] = START_AUTO_PROCESS;
            check_on_auto_process[pl_special_release.gripper_index] = 1;
            check_special_release_2_done[pl_special_release.gripper_index] = 0;
            USB_send_respond_package(CMD_SPECIAL_RELEASE_FUNC, cmd_process.id, RESPOND_SUCESS);
        }
        else{// if(pl_special_release.function == SPECIAL_RELEASE_2){
            // do linear 1: motor 8.10.12.14
            Get_ctrl_proces[pl_special_release.gripper_index].gripper = pl_special_release.gripper_index;
            Get_ctrl_proces[pl_special_release.gripper_index].step = PROCESS_RELEASE_SUB;

            start_auto_process[pl_special_release.gripper_index] = START_AUTO_PROCESS;
            check_on_auto_process[pl_special_release.gripper_index] = 1;
            check_special_release_2_done[pl_special_release.gripper_index] = 1;
            USB_send_respond_package(CMD_SPECIAL_RELEASE_FUNC, cmd_process.id, RESPOND_SUCESS);
        }
        break;
    case CMD_READ_RANGE_SENSOR:
        range_sensor.status = RESPOND_SUCESS;
        range_sensor.distance = read_ultrasonic_value();
        if(range_sensor.distance == 0 || range_sensor.distance == 1){
            USB_send_respond_package(CMD_READ_RANGE_SENSOR, cmd_process.id, RESPOND_FAILED);
        }
        else{
            USB_send_respond_package(CMD_READ_RANGE_SENSOR, cmd_process.id, RESPOND_SUCESS);
        }
        break;
   case CMD_READ_THERMAL_SENSOR:
	   thermal_sensor.status = RESPOND_SUCESS;
	   doSetting = cmd_process.payload[0];
       valueReg00 = cmd_process.payload[1];
       valueReg01 = cmd_process.payload[2];
       valueReg02 = cmd_process.payload[3];
		if (doSetting == 0x01)
		{
			if (settingThermalSensorDT32L01A(doSetting,valueReg00,valueReg01,valueReg02) != SUCCESS_)
			{
				USB_send_respond_package(CMD_READ_THERMAL_SENSOR, cmd_process.id, RESPOND_FAILED);
			}
		}
		if(getValueThermalSensorD6T32L01A() != SUCCESS_)
		{				
			USB_send_respond_package(CMD_READ_THERMAL_SENSOR, cmd_process.id, RESPOND_FAILED);
		}
		else{
			USB_send_respond_package(CMD_READ_THERMAL_SENSOR, cmd_process.id, RESPOND_SUCESS);
		}			
		break;
    case CMD_CTRL_ALL_SUB_LINEAR:
        memcpy(&ctrl_process, cmd_process.payload, sizeof(ctrl_process));
        // check can be control: grippers must be gone home before control
        if(check_calib_gohome_ctrl_step != CONTROL){
            USB_send_respond_package(CMD_CTRL_AUTO_PROCESS, cmd_process.id, RESPOND_CANNOT_CONTROL);
            return;
        }
        else{
            USB_send_respond_package(CMD_CTRL_ALL_SUB_LINEAR, cmd_process.id, RESPOND_SUCESS);
            for(i = GRIPPER_1; i <= GRIPPER_4; i++){
                if(check_conflict_ctrl_process() == FAILED_){
                    continue;
                }
                else if(check_conflict_ctrl_process() == RUNNING_){
                    continue;
                }

                if((Grippers_infor.gripper_No1 == HAVE_PHONE) && (i == GRIPPER_1)){
                    ctrl_process.gripper = GRIPPER_1;
                }
                else if((Grippers_infor.gripper_No2 == HAVE_PHONE) && (i == GRIPPER_2)){
                        ctrl_process.gripper = GRIPPER_2;
                }
                else if((Grippers_infor.gripper_No3 == HAVE_PHONE) && (i == GRIPPER_3)){
                        ctrl_process.gripper = GRIPPER_3;
                }
                else if((Grippers_infor.gripper_No4 == HAVE_PHONE) && (i == GRIPPER_4)){
                        ctrl_process.gripper = GRIPPER_4;
                }
                else{
                    ctrl_process.gripper = 4;
                }
                // process
                if(ctrl_process.gripper != 4){
                    if(ctrl_process.step == PROCESS_RELEASE_SUB){
                        if(process_of_gripper[ctrl_process.gripper].step != PROCESS_CLAMP_SUB){
                            continue;
                        }
                    }

                    if(ctrl_process.step == PROCESS_CLAMP_SUB){
                        if(process_of_gripper[ctrl_process.gripper].step == PROCESS_RELEASE_MAIN){
                            continue;
                        }
                        if(process_of_gripper[ctrl_process.gripper].step == PROCESS_CLAMP_SUB){
                            continue;
                        }
                    }

                    sprintf(str__, "CMD_CTRL_ALL_SUB_LINEAR start gripper %d \n\r", ctrl_process.gripper);
                    send_debugging_data(str__);

                    Get_ctrl_proces[ctrl_process.gripper].gripper = ctrl_process.gripper;
                    Get_ctrl_proces[ctrl_process.gripper].step = ctrl_process.step;
                    Get_ctrl_proces[ctrl_process.gripper].style = ctrl_process.style;
                    check_on_auto_process[ctrl_process.gripper] = 1;
                    start_auto_process[ctrl_process.gripper] = START_AUTO_PROCESS;
                }
            }
        }
        break;
    case CMD_CTRL_SENSOR_POWER:
        if(cmd_process.payload[0] == 0){
            OnOffSensorPower(GPIO_PIN_RESET);
        }
        else{
            OnOffSensorPower(GPIO_PIN_SET);
        }
        USB_send_respond_package(CMD_CTRL_SENSOR_POWER, cmd_process.id, RESPOND_SUCESS);
        break;
    default:
        USB_send_respond_package(cmd_process.opcode, cmd_process.id, RESPOND_FAILED);
        break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    uint8_t b = 0, *p;
    uint32_t ticks = HAL_GetTick();

    if(huart->Instance == USART1){
        b = rxBuf;
        if(ticks - ticksByte > TIMEOUT_GET_BYTE){
            get_cmd_stt = GET_HEADER0;
            ticksByte = ticks;
        }
        switch(get_cmd_stt){
        case GET_HEADER0:
            if(b==HEADER0){
                get_cmd_stt = GET_HEADER1;
            }
            break;
        case GET_HEADER1:
            if(b==HEADER1){
                get_cmd_stt = GET_LEN0;
                cmd_get.header = 0xFF55;
            }else if(b!=HEADER0){
                get_cmd_stt = GET_HEADER0;
            }
            break;
        case GET_LEN0:
            get_cmd_stt = GET_LEN1;
            cmd_get.len = b;
            getIndex = 0;
            break;
        case GET_LEN1:
            get_cmd_stt = GET_TYPE;
            cmd_get.len += (uint16_t)b << 8;
            getIndex++;
            break;
        case GET_TYPE:
            get_cmd_stt = GET_TRAN_ID0;
            cmd_get.type = b;
            getIndex++;
            break;
        case GET_TRAN_ID0:
            get_cmd_stt = GET_TRAN_ID1;
            cmd_get.id = b;
            getIndex++;
            break;
        case GET_TRAN_ID1:
            get_cmd_stt = GET_OTHER;
            cmd_get.id += (uint16_t)b << 8;
            getIndex++;
            break;
        case GET_OTHER:
            p = (uint8_t*)&cmd_get.len + 1;
            p += getIndex;
            *p = b;

            if(++getIndex >=cmd_get.len){
                memcpy((void*)&cmd_process, (void*)&cmd_get, cmd_get.len + 3);
                get_cmd_stt = GET_HEADER0;
                ticksByte = ticks;//GDDB fix timeout response
                start_process = 1;
            }
            else{
                get_cmd_stt = GET_OTHER;
            }
            break;
        }
        HAL_UART_Receive_IT(huart, &rxBuf, 1);
    }
}

void prepare_motor_infor_to_respond(void){
    pl_respond_motor_infor.status = motor_infor[pl_respond_motor_infor.index].status;
    pl_respond_motor_infor.pwr = motor_infor[pl_respond_motor_infor.index].pwr;
    pl_respond_motor_infor.duty = motor_infor[pl_respond_motor_infor.index].duty;
    pl_respond_motor_infor.current = motor_infor[pl_respond_motor_infor.index].current;
}

uint8_t check_conflict_ctrl_process(void){

//    return SUCCESS_;
    if(process_of_gripper[ctrl_process.gripper].step == PROCESS_RELEASE_MAIN) // is have no phone // main linear (linear2) released
    {
//        if(ctrl_process.step != PROCESS_CLAMP_MAIN){
//            return FAILED_;
//        }
        if((motor_step[ctrl_process.gripper*2 + 9] != UNKNOW_STEP) && (start_auto_process[ctrl_process.gripper] != STOP_AUTO_PROCESS)){
            return RUNNING_;
        }
    }
    else if(process_of_gripper[ctrl_process.gripper].step == PROCESS_CLAMP_MAIN)  // main linear (linear2) clamped
    {
//        if(ctrl_process.step != PROCESS_CLAMP_SUB){
//            return FAILED_;
//        }
        if((motor_step[ctrl_process.gripper*2 + 8] != UNKNOW_STEP) && (start_auto_process[ctrl_process.gripper] != STOP_AUTO_PROCESS)){
            return RUNNING_;
        }
    }
    else if(process_of_gripper[ctrl_process.gripper].step == PROCESS_CLAMP_SUB)  // SUB linear (linear1) clamped
    {
//        if(ctrl_process.step != PROCESS_RELEASE_SUB){
//            return FAILED_;
//        }
        if((motor_step[ctrl_process.gripper*2 + 9] != UNKNOW_STEP) && (start_auto_process[ctrl_process.gripper] != STOP_AUTO_PROCESS)){
            return RUNNING_;
        }
    }
    else if(process_of_gripper[ctrl_process.gripper].step == PROCESS_RELEASE_SUB)  // SUB linear (linear1) released
    {
//        if(ctrl_process.step != PROCESS_RELEASE_MAIN){
//            return FAILED_;
//        }
        if((motor_step[ctrl_process.gripper*2] != UNKNOW_STEP) && (start_auto_process[ctrl_process.gripper] != STOP_AUTO_PROCESS)){
            return RUNNING_;
        }
    }
    else // unknow
    {
        return UNKNOW;
    }
    return SUCCESS_;
}

uint8_t get_servo1_index_from_gripper_index(uint8_t gripper_index){
    if(gripper_index == GRIPPER_1){
        return G1_SERVO_1;
    }
    else if(gripper_index == GRIPPER_2){
        return G2_SERVO_1;
    }
    else if(gripper_index == GRIPPER_3){
        return G3_SERVO_1;
    }
    else{// if(gripper_index == GRIPPER_4){
        return G4_SERVO_1;
    }
}

uint8_t get_Gr_index_from_Mo_index(uint8_t index){
    uint8_t G_index = 0;
    switch (index) {
    case G1_SERVO_1:
    case G1_SERVO_2:
    case G1_LINEAR_SERVO_1:
    case G1_LINEAR_SERVO_2:
        G_index = GRIPPER_1;
        break;
    case G2_SERVO_1:
    case G2_SERVO_2:
    case G2_LINEAR_SERVO_1:
    case G2_LINEAR_SERVO_2:
        G_index = GRIPPER_2;
        break;
    case G3_SERVO_1:
    case G3_SERVO_2:
    case G3_LINEAR_SERVO_1:
    case G3_LINEAR_SERVO_2:
        G_index = GRIPPER_3;
        break;
    case G4_SERVO_1:
    case G4_SERVO_2:
    case G4_LINEAR_SERVO_1:
    case G4_LINEAR_SERVO_2:
        G_index = GRIPPER_4;
        break;
    default:
        break;
    }
    return G_index;
}

void auto_process_ctrl(uint8_t motor_index){ // motor index must is; G1_SERVO_1 , G2_SERVO_1, G3_SERVO_1, G4_SERVO_1
//    uint8_t motor_index;
//    motor_index = get_servo1_index_from_gripper_index(ctrl_process.gripper);

    if(Get_ctrl_proces[get_Gr_index_from_Mo_index(motor_index)].step == PROCESS_CLAMP_MAIN){ // ctrl linear2: 9,11,13,15
        if(check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] == 1){
            send_debugging_data("AAAAAAAAAAAAAAAAAAAa");

            check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] = 2;
            pl_control_motor[motor_index + 9].motor_index = motor_index + 9;
            pl_control_motor[motor_index + 9].duty =  clamp_duty_cycle[motor_index + 9];
            motor_step[motor_index + 9] = STEP_1;
            //update process value
            process_of_gripper[get_Gr_index_from_Mo_index(motor_index)].gripper = get_Gr_index_from_Mo_index(motor_index);
            process_of_gripper[Get_ctrl_proces[get_Gr_index_from_Mo_index(motor_index)].gripper].step = PROCESS_CLAMP_MAIN;
        }else if(check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] == 2){
            if(motor_step[motor_index + 9] == UNKNOW_STEP){
                send_debugging_data("BBBBBBBBBBBBBBB");
                check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] = 0;
                start_auto_process[get_Gr_index_from_Mo_index(motor_index)] = STOP_AUTO_PROCESS;
            }
        }
    }
    else if(Get_ctrl_proces[get_Gr_index_from_Mo_index(motor_index)].step == PROCESS_CLAMP_SUB){
        // set 2 servo clamp
        if(check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] == 1){
            check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] = 2;

            pl_control_motor[motor_index].motor_index = motor_index;
            pl_control_motor[motor_index].duty =  clamp_duty_cycle[motor_index];
            motor_step[motor_index] = STEP_1;

            pl_control_motor[motor_index + 1].motor_index = motor_index + 1;
            pl_control_motor[motor_index + 1].duty =  clamp_duty_cycle[motor_index + 1];
            motor_step[motor_index + 1] = STEP_1;
            process_of_gripper[Get_ctrl_proces[get_Gr_index_from_Mo_index(motor_index)].gripper].step = PROCESS_CLAMP_SUB;
        }
        else if(check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] == 2){
            // check 2 servo clamp done
            // check motor servo error or not
            if((motor_infor[motor_index].status == CLAMP)&&(motor_infor[motor_index+1].status == CLAMP)){
                // set linear 1 clamp: 8,10,12,14
                pl_control_motor[motor_index + 8].motor_index = motor_index + 8;
                pl_control_motor[motor_index + 8].duty = clamp_duty_cycle[motor_index + 8];
                motor_step[motor_index + 8] = STEP_1;
                check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] = 3;
            }
            else{
                motor_step[motor_index + 8] = UNKNOW_STEP;
            }
            //update process value
            process_of_gripper[get_Gr_index_from_Mo_index(motor_index)].gripper = get_Gr_index_from_Mo_index(motor_index);
        }else if(check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] == 3){
            if(motor_step[motor_index + 8] == UNKNOW_STEP){
                check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] = 0;
                start_auto_process[get_Gr_index_from_Mo_index(motor_index)] = STOP_AUTO_PROCESS;
            }
        }

    }
    else if(Get_ctrl_proces[get_Gr_index_from_Mo_index(motor_index)].step == PROCESS_RELEASE_SUB){

        if(check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] == 1){
            check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] = 2;
            // ctrl linear1 release: 8,10,12,14
            pl_control_motor[motor_index + 8].motor_index = motor_index + 8;
            pl_control_motor[motor_index + 8].duty =  release_duty_cycle[motor_index + 8];
            motor_step[motor_index + 8] = STEP_1;
            process_of_gripper[Get_ctrl_proces[get_Gr_index_from_Mo_index(motor_index)].gripper].step = PROCESS_RELEASE_SUB;
        }
        else if(check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] == 2){
            // check linear1 release done
            // set 2 servo clamp
            if(motor_infor[motor_index + 8].status == RELEASE){
                pl_control_motor[motor_index].motor_index = motor_index;
                pl_control_motor[motor_index].duty =  release_duty_cycle[motor_index];
                motor_step[motor_index] = STEP_1;

                pl_control_motor[motor_index + 1].motor_index = motor_index + 1;
                pl_control_motor[motor_index + 1].duty =  release_duty_cycle[motor_index + 1];
                motor_step[motor_index + 1] = STEP_1;
                //update process value
                process_of_gripper[get_Gr_index_from_Mo_index(motor_index)].gripper = get_Gr_index_from_Mo_index(motor_index);
                check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] = 3;
            }
        }
        else if(check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] == 3){
            if((motor_step[motor_index] == UNKNOW_STEP) && (motor_step[motor_index + 1] == UNKNOW_STEP)){
                check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] = 0;
                start_auto_process[get_Gr_index_from_Mo_index(motor_index)] = STOP_AUTO_PROCESS;
            }
        }
    }
    else{// if(Get_ctrl_proces[get_Gr_index_from_Mo_index(motor_index)].step == PROCESS_RELEASE_MAIN){ // ctrl linear2 release: 9,11,13,15
        if(check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] == 1){
            check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] = 2;
            pl_control_motor[motor_index + 9].motor_index = motor_index + 9;
            pl_control_motor[motor_index + 9].duty =  release_duty_cycle[motor_index + 9];
            motor_step[motor_index + 9] = STEP_1;
            //update process value
            process_of_gripper[get_Gr_index_from_Mo_index(motor_index)].gripper = get_Gr_index_from_Mo_index(motor_index);
            process_of_gripper[Get_ctrl_proces[get_Gr_index_from_Mo_index(motor_index)].gripper].step = PROCESS_RELEASE_MAIN;
        }
        else if(check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] == 2){
            if(motor_step[motor_index + 9] == UNKNOW_STEP){
                check_on_auto_process[get_Gr_index_from_Mo_index(motor_index)] = 0;
                start_auto_process[get_Gr_index_from_Mo_index(motor_index)] = STOP_AUTO_PROCESS;
            }
        }
    }
}

void calib_process(void){
    if(calib_value.read_write == READ_CALIB){
        if(calib_value.clamp_release == CALIB_CLAMP){
            sprintf(str__, " calib process read clamp\n\r");
            send_debugging_data(str__);

            calib_value.G1_servo_1 = clamp_duty_cycle[G1_SERVO_1];
            calib_value.G1_servo_2 = clamp_duty_cycle[G1_SERVO_2];
            calib_value.G2_servo_1 = clamp_duty_cycle[G2_SERVO_1];
            calib_value.G2_servo_2 = clamp_duty_cycle[G2_SERVO_2];
            calib_value.G3_servo_1 = clamp_duty_cycle[G3_SERVO_1];
            calib_value.G3_servo_2 = clamp_duty_cycle[G3_SERVO_2];
            calib_value.G4_servo_1 = clamp_duty_cycle[G4_SERVO_1];
            calib_value.G4_servo_2 = clamp_duty_cycle[G4_SERVO_2];
        }
        else{
            sprintf(str__, " calib process read release\n\r");
            send_debugging_data(str__);

            calib_value.G1_servo_1 = release_duty_cycle[G1_SERVO_1];
            calib_value.G1_servo_2 = release_duty_cycle[G1_SERVO_2];
            calib_value.G2_servo_1 = release_duty_cycle[G2_SERVO_1];
            calib_value.G2_servo_2 = release_duty_cycle[G2_SERVO_2];
            calib_value.G3_servo_1 = release_duty_cycle[G3_SERVO_1];
            calib_value.G3_servo_2 = release_duty_cycle[G3_SERVO_2];
            calib_value.G4_servo_1 = release_duty_cycle[G4_SERVO_1];
            calib_value.G4_servo_2 = release_duty_cycle[G4_SERVO_2];
        }
    }
    else{
        if(calib_value.clamp_release == CALIB_CLAMP){
            sprintf(str__, " calib process write clamp\n\r");
            send_debugging_data(str__);

            clamp_duty_cycle[G1_SERVO_1] = calib_value.G1_servo_1;
            clamp_duty_cycle[G1_SERVO_2] = calib_value.G1_servo_2;
            clamp_duty_cycle[G2_SERVO_1] = calib_value.G2_servo_1;
            clamp_duty_cycle[G2_SERVO_2] = calib_value.G2_servo_2;
            clamp_duty_cycle[G3_SERVO_1] = calib_value.G3_servo_1;
            clamp_duty_cycle[G3_SERVO_2] = calib_value.G3_servo_2;
            clamp_duty_cycle[G4_SERVO_1] = calib_value.G4_servo_1;
            clamp_duty_cycle[G4_SERVO_2] = calib_value.G4_servo_2;
        }
        else{
            sprintf(str__, " calib process write release\n\r");
            send_debugging_data(str__);

            release_duty_cycle[G1_SERVO_1] = calib_value.G1_servo_1;
            release_duty_cycle[G1_SERVO_2] = calib_value.G1_servo_2;
            release_duty_cycle[G2_SERVO_1] = calib_value.G2_servo_1;
            release_duty_cycle[G2_SERVO_2] = calib_value.G2_servo_2;
            release_duty_cycle[G3_SERVO_1] = calib_value.G3_servo_1;
            release_duty_cycle[G3_SERVO_2] = calib_value.G3_servo_2;
            release_duty_cycle[G4_SERVO_1] = calib_value.G4_servo_1;
            release_duty_cycle[G4_SERVO_2] = calib_value.G4_servo_2;
        }
    }
}

uint8_t check_motor_gohome_control(uint8_t index){
    if((Grippers_infor.gripper_No1 == HAVE_PHONE) && ((index == G1_SERVO_1)||(index == G1_SERVO_2)||(index == G1_LINEAR_SERVO_1)||(index == G1_LINEAR_SERVO_2))){
//        motor_infor[index].status = CLAMP;
        return FAILED_;
    }
    else if((Grippers_infor.gripper_No2 == HAVE_PHONE) && ((index == G2_SERVO_1)||(index == G2_SERVO_2)||(index == G2_LINEAR_SERVO_1)||(index == G2_LINEAR_SERVO_2))){
//        motor_infor[index].status = CLAMP;
        return FAILED_;
    }
    else if((Grippers_infor.gripper_No3 == HAVE_PHONE) && ((index == G3_SERVO_1)||(index == G3_SERVO_2)||(index == G3_LINEAR_SERVO_1)||(index == G3_LINEAR_SERVO_2))){
//        motor_infor[index].status = CLAMP;
        return FAILED_;
    }
    else if((Grippers_infor.gripper_No4 == HAVE_PHONE) && ((index == G4_SERVO_1)||(index == G4_SERVO_2)||(index == G4_LINEAR_SERVO_1)||(index == G4_LINEAR_SERVO_2))){
//        motor_infor[index].status = CLAMP;
        return FAILED_;
    }
    else
        return SUCCESS_;
}

void set_process_step(void){
    if(Grippers_infor.gripper_No1 == HAVE_PHONE){
        process_of_gripper[GRIPPER_1].step = PROCESS_CLAMP_SUB;
    }
    else{
        process_of_gripper[GRIPPER_1].step = PROCESS_RELEASE_MAIN;
    }

    if(Grippers_infor.gripper_No2 == HAVE_PHONE){
        process_of_gripper[GRIPPER_2].step = PROCESS_CLAMP_SUB;
    }
    else{
        process_of_gripper[GRIPPER_2].step = PROCESS_RELEASE_MAIN;
    }

    if(Grippers_infor.gripper_No3 == HAVE_PHONE){
        process_of_gripper[GRIPPER_3].step = PROCESS_CLAMP_SUB;
    }
    else{
        process_of_gripper[GRIPPER_3].step = PROCESS_RELEASE_MAIN;
    }

    if(Grippers_infor.gripper_No4 == HAVE_PHONE){
        process_of_gripper[GRIPPER_4].step = PROCESS_CLAMP_SUB;
    }
    else
    {
        process_of_gripper[GRIPPER_4].step = PROCESS_RELEASE_MAIN;
    }
}

void ctrl_motor(void){
    uint8_t index;
    if(start_gohome == GOHOME){
        if(IsFirstTimeGoHome == false){
            for(index = 8 ; index < PWM_NUM_CHANNEL; index++){
                if(check_motor_gohome_control(index) != FAILED_){
                    ctrl_pwm_duty(index, release_duty_cycle[index]);
                    on_off_pwR(index, PIN_HIGH);
                    motor_infor[index].index = pl_control_motor[index].motor_index = index;
                    motor_infor[index].status = UNKNOW_;
                    motor_infor[index].duty = pl_control_motor[index].duty = release_duty_cycle[index];
                    motor_infor[index].current = adc_value[index];
                    motor_step[index] = UNKNOW_STEP;
                }
            }
            osDelay(1500);
            for(index = 0 ; index < 8; index++){
                if(check_motor_gohome_control(index) != FAILED_){
                    ctrl_pwm_duty(index, release_duty_cycle[index]);
                    on_off_pwR(index, PIN_HIGH);
                    motor_infor[index].index = pl_control_motor[index].motor_index = index;
                    motor_infor[index].status = UNKNOW_;
                    motor_infor[index].duty = pl_control_motor[index].duty = release_duty_cycle[index];
                    motor_infor[index].current = adc_value[index];
                    motor_step[index] = UNKNOW_STEP;
                }
            }
            osDelay(1000);
            for(index = 0 ; index < PWM_NUM_CHANNEL; index++){
                if(check_motor_gohome_control(index) != FAILED_){
                    motor_infor[index].status = RELEASE;
                    if(index < 8){
                        on_off_pwR(index, PIN_LOW);
                    }
                }
            }
        }
        else{
            init_servo_motors();
            IsFirstTimeGoHome = false;
        }
        start_gohome = 0;
//        set_process_step();
        start_auto_process[GRIPPER_1] = STOP_AUTO_PROCESS;
        start_auto_process[GRIPPER_2] = STOP_AUTO_PROCESS;
        start_auto_process[GRIPPER_3] = STOP_AUTO_PROCESS;
        start_auto_process[GRIPPER_4] = STOP_AUTO_PROCESS;

    }
    else{
        if(calib_value.read_write == WRITE_CALIB){
            for(index = 0; index < 8; index++){
                if(calib_value.clamp_release == CALIB_RELEASE){
                    ctrl_pwm_duty(index, release_duty_cycle[index]);
                    on_off_pwR(index, PIN_HIGH);
                }
                else{
                    ctrl_pwm_duty(index, clamp_duty_cycle[index]);
                    on_off_pwR(index, PIN_HIGH);
                }
            }
            osDelay(2000);
            for(index = 0 ; index < 8; index++){
                on_off_pwR(index, PIN_LOW);
            }

            check_calib_gohome_ctrl_step = GO_HOME;
        }
        else{}
    }
}

