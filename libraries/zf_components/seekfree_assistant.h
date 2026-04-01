/*********************************************************************************************************************
* CYT4BB Opensourec Library 鍗筹紙 CYT4BB 寮€婧愬簱锛夋槸涓€涓熀浜庡畼鏂?SDK 鎺ュ彛鐨勭涓夋柟寮€婧愬簱
* Copyright (c) 2022 SEEKFREE 閫愰绉戞妧
*
* 鏈枃浠舵槸 CYT4BB 寮€婧愬簱鐨勪竴閮ㄥ垎
*
* CYT4BB 寮€婧愬簱 鏄厤璐硅蒋浠?
* 鎮ㄥ彲浠ユ牴鎹嚜鐢辫蒋浠跺熀閲戜細鍙戝竷鐨?GPL锛圙NU General Public License锛屽嵆 GNU閫氱敤鍏叡璁稿彲璇侊級鐨勬潯娆?
* 鍗?GPL 鐨勭3鐗堬紙鍗?GPL3.0锛夋垨锛堟偍閫夋嫨鐨勶級浠讳綍鍚庢潵鐨勭増鏈紝閲嶆柊鍙戝竷鍜?鎴栦慨鏀瑰畠
*
* 鏈紑婧愬簱鐨勫彂甯冩槸甯屾湜瀹冭兘鍙戞尌浣滅敤锛屼絾骞舵湭瀵瑰叾浣滀换浣曠殑淇濊瘉
* 鐢氳嚦娌℃湁闅愬惈鐨勯€傞攢鎬ф垨閫傚悎鐗瑰畾鐢ㄩ€旂殑淇濊瘉
* 鏇村缁嗚妭璇峰弬瑙?GPL
*
* 鎮ㄥ簲璇ュ湪鏀跺埌鏈紑婧愬簱鐨勫悓鏃舵敹鍒颁竴浠?GPL 鐨勫壇鏈?
* 濡傛灉娌℃湁锛岃鍙傞槄<https://www.gnu.org/licenses/>
*
* 棰濆娉ㄦ槑锛?
* 鏈紑婧愬簱浣跨敤 GPL3.0 寮€婧愯鍙瘉鍗忚 浠ヤ笂璁稿彲鐢虫槑涓鸿瘧鏂囩増鏈?
* 璁稿彲鐢虫槑鑻辨枃鐗堝湪 libraries/doc 鏂囦欢澶逛笅鐨?GPL3_permission_statement.txt 鏂囦欢涓?
* 璁稿彲璇佸壇鏈湪 libraries 鏂囦欢澶逛笅 鍗宠鏂囦欢澶逛笅鐨?LICENSE 鏂囦欢
* 娆㈣繋鍚勪綅浣跨敤骞朵紶鎾湰绋嬪簭 浣嗕慨鏀瑰唴瀹规椂蹇呴』淇濈暀閫愰绉戞妧鐨勭増鏉冨０鏄庯紙鍗虫湰澹版槑锛?
*
* 鏂囦欢鍚嶇О          seekfree_assistant
* 鍏徃鍚嶇О          鎴愰兘閫愰绉戞妧鏈夐檺鍏徃
* 鐗堟湰淇℃伅          鏌ョ湅 libraries/doc 鏂囦欢澶瑰唴 version 鏂囦欢 鐗堟湰璇存槑
* 寮€鍙戠幆澧?         IAR 9.40.1
* 閫傜敤骞冲彴          CYT4BB
* 搴楅摵閾炬帴          https://seekfree.taobao.com/
* 
* 淇敼璁板綍
* 鏃ユ湡             浣滆€?            澶囨敞
* 2024-1-11        SeekFree         first version
********************************************************************************************************************/
#ifndef _seekfree_assistant_h_
#define _seekfree_assistant_h_


#include "zf_common_typedef.h"


// 1锛氫娇鑳藉弬鏁拌皟鑺傜殑鍔熻兘  0锛氬叧闂弬鏁拌皟鑺傜殑鍔熻兘
#define SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE      ( 1 )



// 瀹氫箟鎺ユ敹FIFO澶у皬
#define SEEKFREE_ASSISTANT_BUFFER_SIZE              ( 0x80 )
    
// 瀹氫箟绀烘尝鍣ㄧ殑鏈€澶ч€氶亾鏁伴噺 
#define SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT   ( 0x08 )
    
// 瀹氫箟鍙傛暟璋冭瘯鐨勬渶澶ч€氶亾鏁伴噺    
#define SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT       ( 0x08 )
    
// 瀹氫箟鍥惧儚杈圭嚎鏈€澶ф暟閲?  
#define SEEKFREE_ASSISTANT_CAMERA_MAX_BOUNDARY      ( 0x08 )
    
// 鍗曠墖鏈哄線涓婁綅鏈哄彂閫佺殑甯уご 
#define SEEKFREE_ASSISTANT_SEND_HEAD                ( 0xAA )
    
// 鎽勫儚澶寸被 
#define SEEKFREE_ASSISTANT_CAMERA_FUNCTION          ( 0x02 )
#define SEEKFREE_ASSISTANT_CAMERA_DOT_FUNCTION      ( 0x03 )
#define SEEKFREE_ASSISTANT_CAMERA_OSCILLOSCOPE      ( 0x10 )
    
// 涓婁綅鏈哄線鍗曠墖鏈哄彂閫佺殑甯уご 
#define SEEKFREE_ASSISTANT_RECEIVE_HEAD             ( 0x55 )
    
// 鍙傛暟璁剧疆绫?   
#define SEEKFREE_ASSISTANT_RECEIVE_SET_PARAMETER    ( 0x20 )


// 鎽勫儚澶寸被鍨嬫灇涓?
typedef enum
{
    // 鎸夌収鎽勫儚澶村瀷鍙峰畾涔?
    SEEKFREE_ASSISTANT_OV7725_BIN = 1,
    SEEKFREE_ASSISTANT_MT9V03X,
    SEEKFREE_ASSISTANT_SCC8660,

    // 鎸夌収鍥惧儚绫诲瀷瀹氫箟
    SEEKFREE_ASSISTANT_BINARY = 1,
    SEEKFREE_ASSISTANT_GRAY,
    SEEKFREE_ASSISTANT_RGB565,
}seekfree_assistant_image_type_enum;

// 鎽勫儚澶寸被鍨嬫灇涓?
typedef enum
{
    // 鎸夌収鎽勫儚澶村瀷鍙峰畾涔?
    X_BOUNDARY,     // 鍙戦€佺殑鍥惧儚涓竟鐣屼俊鎭彧鍖呭惈X锛屼篃灏辨槸鍙湁妯潗鏍囦俊鎭紝绾靛潗鏍囨牴鎹浘鍍忛珮搴﹀緱鍒?
    Y_BOUNDARY,     // 鍙戦€佺殑鍥惧儚涓竟鐣屼俊鎭彧鍖呭惈Y锛屼篃灏辨槸鍙湁绾靛潗鏍囦俊鎭紝妯潗鏍囨牴鎹浘鍍忓搴﹀緱鍒帮紝閫氬父寰堝皯鏈夎繖鏍风殑闇€姹?
    XY_BOUNDARY,    // 鍙戦€佺殑鍥惧儚涓竟鐣屼俊鎭寘鍚玐涓嶻锛岃繖鏍峰彲浠ユ寚瀹氱偣鍦ㄤ换鎰忎綅缃紝灏卞彲浠ユ柟渚挎樉绀哄嚭鍥炲集鐨勬晥鏋?
    NO_BOUNDARY,    // 鍙戦€佺殑鍥惧儚涓病鏈夎竟绾夸俊鎭?
}seekfree_assistant_boundary_type_enum;

typedef struct
{
    uint8 head;                                                 // 甯уご
    uint8 channel_num;                                          // 楂樺洓浣嶄负鍔熻兘瀛? 浣庡洓浣嶄负閫氶亾鏁伴噺
    uint8 check_sum;                                            // 鍜屾牎楠?
    uint8 length;                                               // 鍖呴暱搴?
    float data[SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT];      // 閫氶亾鏁版嵁
}seekfree_assistant_oscilloscope_struct;


typedef struct
{
    uint8 head;                                                 // 甯уご
    uint8 function;                                             // 鍔熻兘瀛?
    uint8 camera_type;                                          // 浣庡洓浣嶈〃绀鸿竟鐣屾暟閲?绗洓浣嶈〃绀烘槸鍚︽湁鍥惧儚鏁版嵁  渚嬪0x13锛氬叾涓?琛ㄧず涓€鍓浘鍍忔湁涓夋潯杈圭晫锛堥€氬父鏄乏杈圭晫銆佷腑绾裤€佸彸杈圭晫锛夈€?琛ㄧず娌℃湁鍥惧儚鏁版嵁
    uint8 length;                                               // 鍖呴暱搴︼紙浠呭寘鍚崗璁儴鍒嗭級
    uint16 image_width;                                         // 鍥惧儚瀹藉害
    uint16 image_height;                                        // 鍥惧儚楂樺害
}seekfree_assistant_camera_struct;          
            
            
typedef struct          
{           
    uint8 head;                                                 // 甯уご
    uint8 function;                                             // 鍔熻兘瀛?
    uint8 dot_type;                                             // 鐐圭被鍨? BIT5锛?锛氬潗鏍囨槸16浣嶇殑 0锛氬潗鏍囨槸8浣嶇殑    BIT7-BIT6锛?锛氬彧鏈塜鍧愭爣 1锛氬彧鏈塝鍧愭爣 2锛歑鍜孻鍧愭爣閮芥湁    BIT3-BIT0锛氳竟鐣屾暟閲?
    uint8 length;                                               // 鍖呴暱搴︼紙浠呭寘鍚崗璁儴鍒嗭級
    uint16 dot_num;                                             // 鐢荤偣鏁伴噺
    uint8  valid_flag;                                          // 閫氶亾鏍囪瘑
    uint8  reserve;                                             // 淇濈暀
}seekfree_assistant_camera_dot_struct;          
            
typedef struct          
{           
    void *image_addr;                                           // 鎽勫儚澶村湴鍧€
    uint16 width;                                               // 鍥惧儚瀹藉害
    uint16 height;                                              // 鍥惧儚楂樺害
    seekfree_assistant_image_type_enum camera_type;             // 鎽勫儚澶寸被鍨?
    void *boundary_x[SEEKFREE_ASSISTANT_CAMERA_MAX_BOUNDARY];   // 杈圭晫妯潗鏍囨暟缁勫湴鍧€
    void *boundary_y[SEEKFREE_ASSISTANT_CAMERA_MAX_BOUNDARY];   // 杈圭晫绾靛潗鏍囨暟缁勫湴鍧€
}seekfree_assistant_camera_buffer_struct;

typedef struct
{
    uint8 head;                                                 // 甯уご
    uint8 function;                                             // 鍔熻兘瀛?
    uint8 channel;                                              // 閫氶亾
    uint8 check_sum;                                            // 鍜屾牎楠?
    float data;                                                 // 鏁版嵁
}seekfree_assistant_parameter_struct;

typedef uint32 (*seekfree_assistant_transfer_callback_function) (const uint8 *buff, uint32 length);
typedef uint32 (*seekfree_assistant_receive_callback_function)  (uint8 *buff, uint32 length);

extern seekfree_assistant_oscilloscope_struct                   seekfree_assistant_oscilloscope_data;                                               // 铏氭嫙绀烘尝鍣ㄦ暟鎹?
extern float                                                    seekfree_assistant_parameter[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT];                // 淇濆瓨鎺ユ敹鍒扮殑鍙傛暟
extern vuint8                                                   seekfree_assistant_parameter_update_flag[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT];    // 鍙傛暟鏇存柊鏍囧織浣?
    
    
void    seekfree_assistant_oscilloscope_send                    (seekfree_assistant_oscilloscope_struct *seekfree_assistant_oscilloscope);
void    seekfree_assistant_camera_information_config            (seekfree_assistant_image_type_enum camera_type, void *image_addr, uint16 width, uint16 height);
void    seekfree_assistant_camera_boundary_config               (seekfree_assistant_boundary_type_enum boundary_type, uint16 dot_num, void *dot_x1, void *dot_x2, void *dot_x3, void *dot_y1, void *dot_y2, void *dot_y3);
void    seekfree_assistant_camera_send                          (void);
void    seekfree_assistant_data_analysis                        (void);



#endif
