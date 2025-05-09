// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: ESP32Box

#include "ui.h"
#include "robot.h"
#include "wifi_config.h"
#include "lwip_demo.h"
void switchauto(lv_event_t * e)
{
	bool is_checked = lv_obj_has_state(ui_SwitchAuto, LV_STATE_CHECKED);
	// 根据状态进行操作
	//printf("is_checked=%d\n",is_checked);
	if (is_checked) {
		robot_cmd.auto_mode = 1;
		printf("自动模式启动\n");
	} else {
		robot_cmd.auto_mode = 0;
		printf("手动模式启动\n");
	}
}

void event_power(lv_event_t * e)
{
	robot_cmd.power_cnt ++;
	robot_cmd.gait_mode = 99;
	printf("power_switch\n");
	// Your code here
}

void event_powerr(lv_event_t * e)
{
	///printf("robot_cmd.power_cnt=%d\n",robot_cmd.power_cnt);
	robot_cmd.power_cnt = 0;
	robot_cmd.gait_mode = 0;
	printf("power_switch left\n");
	// Your code here
}
void Event_up(lv_event_t * e)
{
	int slider_value = lv_slider_get_value(ui_SliderSpeed);
	// 打印当前值
	//printf("当前滑块的值：%d\n", slider_value);
	robot_cmd.vel[0]=(float)slider_value/100.0;
	printf("vel=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void Event_upr(lv_event_t * e)
{
	robot_cmd.vel[0]=0;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void Event_l(lv_event_t * e)
{
	int slider_value = lv_slider_get_value(ui_SliderSpeed);
	// 打印当前值
	// printf("当前滑块的值：%d\n", slider_value);
	robot_cmd.vel[2]=0.3;//rad/s (float)slider_value/100.0/2;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void Event_lr(lv_event_t * e)
{
	robot_cmd.vel[2]=0;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void Event_down(lv_event_t * e)
{
	int slider_value = lv_slider_get_value(ui_SliderSpeed);
	robot_cmd.vel[0]=-(float)slider_value/100.0;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void Event_downr(lv_event_t * e)
{
	robot_cmd.vel[0]=0;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void Event_r(lv_event_t * e)
{
	int slider_value = lv_slider_get_value(ui_SliderSpeed);
	// 打印当前值
	// printf("当前滑块的值：%d\n", slider_value);
	robot_cmd.vel[2]=-0.3;//rad/s (float)slider_value/100.0;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void Event_rr(lv_event_t * e)
{
	robot_cmd.vel[2]=0;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void event_mid(lv_event_t * e)
{
	robot_cmd.gait_mode = 1;
	printf("event_mid\n");
	// Your code here
}

void event_midr(lv_event_t * e)
{
	robot_cmd.gait_mode = 0;
	printf("event_mid left\n");
}

void System_set(lv_event_t * e)
{
	//读取机器人模式
	int selected_index = lv_dropdown_get_selected(ui_Dropdown1);
	const char *options = lv_dropdown_get_options(ui_Dropdown1);
	robot_cmd.auto_mode = selected_index;

	//读取复选框
	bool is_checked = lv_obj_has_state(ui_CheckboxHead, LV_STATE_CHECKED);
	robot_cmd.head_mode = is_checked;

	is_checked = lv_obj_has_state(ui_CheckboxSW, LV_STATE_CHECKED);
	robot_cmd.step_mode = is_checked;

	//读取高度
	int slider_value = lv_slider_get_value(ui_SliderHeight);
	robot_cmd.pos[2] = (float)slider_value/100.0;
	printf("auto_mode=%d head_mode=%d step_mode=%d height=%f\n",robot_cmd.auto_mode,robot_cmd.head_mode,robot_cmd.step_mode,robot_cmd.pos[2]);
}


void keyinput(lv_event_t * e)
{
	//get key value
	if(robot_lvgl.keyboard_input_sel==0)//输入IP
		lv_keyboard_set_textarea(ui_KeyboardWifi, ui_TextAreaIP);
	else
		lv_keyboard_set_textarea(ui_KeyboardWifi, ui_TextAreaPwd);
}

void selectIP(lv_event_t * e)
{
	robot_lvgl.keyboard_input_sel=0;
	printf("selectIP\n");
}

void seletpwd(lv_event_t * e)
{
	robot_lvgl.keyboard_input_sel=1;
	printf("selectpwd\n");
}

void set_wifi_config(wifi_config_t *config, const char *ssid, const char *password) {
    // 复制SSID到config的ssid字段
    strncpy((char *)config->sta.ssid, ssid, sizeof(config->sta.ssid));
    config->sta.ssid[sizeof(config->sta.ssid) - 1] = '\0'; // 确保字符串以空字符结尾

    // 复制密码到config的password字段
    strncpy((char *)config->sta.password, password, sizeof(config->sta.password));
    config->sta.password[sizeof(config->sta.password) - 1] = '\0'; // 确保字符串以空字符结尾
}

void ConnetWifi(lv_event_t * e)
{
	//获取wifi名称和密码
    const char * text_pwd = lv_textarea_get_text(ui_TextAreaPwd);
    printf("pwd text: %s\n", text_pwd);

    const char * text_ip = lv_textarea_get_text(ui_TextAreaIP);
    printf("ip text: %s\n", text_ip);
	//获取SSID wifi id
	int selected_index_robot = lv_dropdown_get_selected(ui_DropdownRobot);
	int selected_index_robot_sel = lv_dropdown_get_selected(ui_DropdownRobotSel);
    char text_ssid[50]="LBm";
	if(selected_index_robot==0)
		snprintf(text_ssid, sizeof(text_ssid), "Tinker-2.4G-%d", selected_index_robot_sel);
 	if(selected_index_robot==1)
		snprintf(text_ssid, sizeof(text_ssid), "Butler-2.4G-%d", selected_index_robot_sel);
	if(selected_index_robot==2)
		snprintf(text_ssid, sizeof(text_ssid), "Qloong-2.4G-%d", selected_index_robot_sel);
	if(selected_index_robot==3)
		snprintf(text_ssid, sizeof(text_ssid), "Wloong-2.4G-%d", selected_index_robot_sel);
	 printf("wifi ssid: %s\n", text_ssid);
		
	//复制WIFI参数
	wifi_config_t wifi_config;
	set_wifi_config(&wifi_config, text_ssid, text_pwd);
	wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
	// 打印配置信息以验证
	printf("SSID: %s|\n", wifi_config.sta.ssid);
	printf("Password: %s|\n", wifi_config.sta.password);
	//切换登录
	wifi_sta_init(wifi_config);//wifi 客户端连接 成功后可以发送数据
	lwip_demo(text_ip,LWIP_DEMO_PORT);//udp demo 测试用例，需要注释掉上面的LVGL DEMO，否则会冲突  
}
void calleg(lv_event_t * e)
{
	robot_lvgl.cal_leg_trigger=1;
	//读取模式
	int selected_index = lv_dropdown_get_selected(ui_DropdownCalLeg1);
	if(selected_index==1)//cal all
		robot_cmd.cal_mode=10;
	if(selected_index==0)//cal sel
		robot_cmd.cal_mode=11;	
	
	int selected_index1 = lv_dropdown_get_selected(ui_DropdownCalLeg2);
	robot_cmd.cal_sel=selected_index1;//joint num
	printf("Leg cal_mode=%d robot_cmd.cal_sel=%d\n",robot_cmd.cal_mode,robot_cmd.cal_sel);
}

void calarm(lv_event_t * e)
{
	robot_lvgl.cal_arm_trigger=1;
	//读取模式
	int selected_index = lv_dropdown_get_selected(ui_DropdownCalArm1);
	if(selected_index==1)//cal all
		robot_cmd.cal_mode=20;
	if(selected_index==0)//cal sel
		robot_cmd.cal_mode=21;	

	int selected_index1 = lv_dropdown_get_selected(ui_DropdownCalArm2);
	robot_cmd.cal_sel=selected_index1;//joint num
	printf("Arm cal_mode=%d robot_cmd.cal_sel=%d\n",robot_cmd.cal_mode,robot_cmd.cal_sel);
}

void calimu(lv_event_t * e)
{
	// Cal Acc 0 
	// Cal Gyro 1
	// Cal Baro 2
	// Cal Mag 3
	robot_lvgl.cal_imu_trigger=1;
	//读取模式
	int selected_index = lv_dropdown_get_selected(ui_DropdownCalSensor);
	robot_cmd.cal_mode=30;
	robot_cmd.cal_sel=selected_index;//sensor id
	printf("Sensor cal_mode=%d robot_cmd.cal_sel=%d\n",robot_cmd.cal_mode,robot_cmd.cal_sel);
}

void ButtonUp(lv_event_t * e)
{
	int slider_value = lv_slider_get_value(ui_SliderSpeed);
	// 打印当前值
	//printf("当前滑块的值：%d\n", slider_value);
	robot_cmd.vel[0]=(float)slider_value/100.0;
	printf("vel=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void ButtonUpr(lv_event_t * e)
{
	robot_cmd.vel[0]=0;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void ButtonDown(lv_event_t * e)
{
	int slider_value = lv_slider_get_value(ui_SliderSpeed);
	robot_cmd.vel[0]=-(float)slider_value/100.0;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void ButtonDownr(lv_event_t * e)
{
	robot_cmd.vel[0]=0;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void ButtonRight(lv_event_t * e)
{
	int slider_value = lv_slider_get_value(ui_SliderSpeed);
	// 打印当前值
	// printf("当前滑块的值：%d\n", slider_value);
	robot_cmd.vel[2]=-0.3;//rad/s (float)slider_value/100.0;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void ButtonRightr(lv_event_t * e)
{
	robot_cmd.vel[2]=0;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void ButtonMid(lv_event_t * e)
{
	robot_cmd.gait_mode = 1;
	printf("event_mid\n");
	// Your code here
}

void ButtonMidr(lv_event_t * e)
{
	robot_cmd.gait_mode = 0;
	printf("event_mid left\n");
}

void ButtonLeft(lv_event_t * e)
{
	int slider_value = lv_slider_get_value(ui_SliderSpeed);
	// 打印当前值
	// printf("当前滑块的值：%d\n", slider_value);
	robot_cmd.vel[2]=0.3;//rad/s (float)slider_value/100.0/2;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void ButtonLeftr(lv_event_t * e)
{
	robot_cmd.vel[2]=0;
	printf("vel_left=%f %f\n",robot_cmd.vel[0],robot_cmd.vel[2]);
}

void ButtonRC0(lv_event_t * e)
{
	robot_cmd.gait_mode = 10;
}

void ButtonRC1(lv_event_t * e)
{
	robot_cmd.gait_mode = 11;
}

void ButtonRC2(lv_event_t * e)
{
	robot_cmd.gait_mode = 12;
}

void ButtonRC3(lv_event_t * e)
{
	robot_cmd.gait_mode = 13;
}

void GPTListen(lv_event_t * e)
{
	// Your code here
}

void GPTListenr(lv_event_t * e)
{
	// Your code here
}

void act_record(lv_event_t * e)
{
	// Your code here
}

void act_replay(lv_event_t * e)
{
	// Your code here
}

void act_perception(lv_event_t * e)
{
	// Your code here
}

void act_stop(lv_event_t * e)
{
	// Your code here
}

void Mission_home(lv_event_t * e)
{
	// Your code here
}

void Mission_stop(lv_event_t * e)
{
	// Your code here
}

void Mission_way(lv_event_t * e)
{
	// Your code here
}
