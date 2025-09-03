///*
// * camera.h
// *
// * Created on: 2024骞�6鏈�6鏃�
// * Author: Merlin_yang
// * Enhanced by: 鏅鸿兘杞﹁瑙夊姪鎵�
// */
//
// #ifndef CAMERA_H_
// #define CAMERA_H_
//
// #include "zf_common_typedef.h"
// #include "zf_device_mt9v03x.h"
// //#include "math_plus.h"
//#include "pid.h"
//
// #define  WHITE_COL_START_Y      (60)
// #define  WHITE_COL_END_Y        (5)
// #define  WHITE_COL_DIF_START    (8)
//
// // =================================================================
// // [ADD] 鏅鸿兘杞﹁瑙夊姪鎵�: 鍏冪礌璇嗗埆鐩稿叧鐨勫叕寮�鎺ュ彛
// // =================================================================
//
// // 1. 瀹氫箟璧涢亾鍏冪礌鏋氫妇
// // 鐩殑锛氳鎵�鏈夊寘鍚澶存枃浠剁殑鏂囦欢閮借兘鐞嗚В鍏冪礌鐘舵�佺殑鍚箟銆�
// typedef enum
// {
//     ELEMENT_NONE,
//     ELEMENT_ROUNDABOUT_RIGHT_IN,
//     ELEMENT_ROUNDABOUT_LEFT_IN,
//     ELEMENT_CROSSROAD,
//     ELEMENT_RAMP,
// } TrackElement_e;
//
// // 2. 澹版槑鍏冪礌鐘舵�佸彉閲�
// // 鐩殑锛氳鍏朵粬鏂囦欢鍙互閫氳繃 'extern' 鍏抽敭瀛楁潵璁块棶鍜岃鍙栧綋鍓嶈瘑鍒埌鐨勫厓绱犵姸鎬併��
// extern volatile TrackElement_e current_element;
//
// // =================================================================
//
// // --- 澶栭儴鍙橀噺澹版槑 ---
// extern float turn_angle;
// extern uint8 stop_flag;
// extern uint8 x_write_col;
// extern uint8 y_write_col;
// extern uint8 dead_zone;
// extern uint8 leftline[MT9V03X_H];
// extern uint8 rightline[MT9V03X_H];
// extern uint8 midline[MT9V03X_H];
// extern int16 sar_thre;
// extern int16 flag;
// extern IFX_ALIGN(4) uint8  original_image[MT9V03X_H][MT9V03X_W];
// extern pid_struct_t pid_turn_angle;
// // --- 鏍稿績澶勭悊鍑芥暟 ---
// void camera_mt9v03x_init(void);
// void save_image(void);
// void image_boundary_process(void);
//
// // --- 鍥惧儚棰勫鐞嗕笌杈圭嚎鏌ユ壘 ---
// void get_longest_write_col(void);
// void difsum_left(uint8 y,uint8 x);
// void difsum_right(uint8 y,uint8 x);
//
// // --- 鍏冪礌璇嗗埆涓庣姸鎬佹満 ---
// void analyze_lost_lines(void);
// int  crossroad_recognition_and_processing(void);
// //void island_processing(void);
//
// // --- 鍗曡皟鎬ф娴� (鐜矝杈呭姪鍑芥暟) ---
// int Monotonicity_Change_Right_Min(int start, int end);
// int Monotonicity_Change_Right_Max(int start, int end);
// int Monotonicity_Change_Left_Max(int start, int end);
// int Monotonicity_Change_Left_Min(int start, int end);
//
// // --- 鍙鍖栦笌璋冭瘯 ---
// void show_line(void);
//
// #endif /* CAMERA_H_ */
