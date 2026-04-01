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

#include "zf_common_debug.h"

#include "seekfree_assistant.h"


extern uint32 seekfree_assistant_transfer       (const uint8 *buff, uint32 length);
extern uint32 seekfree_assistant_receive        (uint8 *buff, uint32 length);

#if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)
#include "zf_common_fifo.h"
static uint8            seekfree_assistant_buffer[SEEKFREE_ASSISTANT_BUFFER_SIZE];                                  // FIFO缂撳啿鍖?
static fifo_struct  seekfree_assistant_fifo =                                                                   // FIFO缁撴瀯浣?
{   
    .buffer    = seekfree_assistant_buffer, 
    .execution = FIFO_IDLE, 
    .type      = FIFO_DATA_8BIT,    
    .head      = 0, 
    .end       = 0, 
    .size      = SEEKFREE_ASSISTANT_BUFFER_SIZE,    
    .max       = SEEKFREE_ASSISTANT_BUFFER_SIZE,    
};  
#endif

static seekfree_assistant_camera_struct         seekfree_assistant_camera_data;                                     // 鍥惧儚涓婁綅鏈哄崗璁暟鎹?
static seekfree_assistant_camera_dot_struct     seekfree_assistant_camera_dot_data;                                 // 鍥惧儚涓婁綅鏈烘墦鐐瑰崗璁暟鎹?
static seekfree_assistant_camera_buffer_struct  seekfree_assistant_camera_buffer;                                   // 鍥惧儚浠ュ強杈圭晫缂撳啿鍖轰俊鎭?

seekfree_assistant_transfer_callback_function   seekfree_assistant_transfer_callback = seekfree_assistant_transfer; // 鏁版嵁鍙戦€佸嚱鏁版寚閽?
seekfree_assistant_receive_callback_function    seekfree_assistant_receive_callback  = seekfree_assistant_receive;  // 鏁版嵁鎺ユ敹鍑芥暟鎸囬拡

seekfree_assistant_oscilloscope_struct          seekfree_assistant_oscilloscope_data;                               // 铏氭嫙绀烘尝鍣ㄦ暟鎹?
float   seekfree_assistant_parameter[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT] = {0};                                  // 淇濆瓨鎺ユ敹鍒扮殑鍙傛暟
vuint8  seekfree_assistant_parameter_update_flag[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT] = {0};                      // 鍙傛暟鏇存柊鏍囧織浣?

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    閫愰鍔╂墜姹傚拰鍑芥暟
// 鍙傛暟璇存槑     *buffer         闇€瑕佹牎楠岀殑鏁版嵁鍦板潃
// 鍙傛暟璇存槑     length          鏍￠獙闀垮害
// 杩斿洖鍙傛暟     uint8           鍜屽€?
// 浣跨敤绀轰緥
//-------------------------------------------------------------------------------------------------------------------
static uint8 seekfree_assistant_sum (uint8 *buffer, uint32 length)
{
    uint8 temp_sum = 0;

    while(length--)
    {
        temp_sum += *buffer++;
    }

    return temp_sum;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    閫愰鍔╂墜 鍥惧儚鍙戦€佸嚱鏁?
// 鍙傛暟璇存槑     camera_type     鎽勫儚澶寸被鍨?
// 鍙傛暟璇存槑     *image_addr     鍥惧儚棣栧湴鍧€
// 鍙傛暟璇存槑     boundary_num    鍥惧儚涓寘鍚竟鐣屾暟閲?
// 鍙傛暟璇存槑     width           鍥惧儚瀹藉害
// 鍙傛暟璇存槑     height          鍥惧儚楂樺害
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_camera_data_send (seekfree_assistant_image_type_enum camera_type, void *image_addr, uint8 boundary_num, uint16 width, uint16 height)
{
    uint32 image_size = 0;

    seekfree_assistant_camera_data.head           = SEEKFREE_ASSISTANT_SEND_HEAD;
    seekfree_assistant_camera_data.function       = SEEKFREE_ASSISTANT_CAMERA_FUNCTION;
    seekfree_assistant_camera_data.camera_type    = (camera_type << 5) | ((image_addr != NULL ? 0 : 1) << 4) | boundary_num;
    // 鍐欏叆鍖呴暱搴︿俊鎭紝浠呭寘鍚崗璁儴鍒?
    seekfree_assistant_camera_data.length         = sizeof(seekfree_assistant_camera_struct);
    seekfree_assistant_camera_data.image_width    = width;
    seekfree_assistant_camera_data.image_height   = height;

    // 棣栧厛鍙戦€佸抚澶淬€佸姛鑳姐€佹憚鍍忓ご绫诲瀷銆佷互鍙婂搴﹂珮搴︾瓑淇℃伅
    seekfree_assistant_transfer_callback((const uint8 *)&seekfree_assistant_camera_data, sizeof(seekfree_assistant_camera_struct));

    // 鏍规嵁鎽勫儚澶寸被鍨嬭绠楀浘鍍忓ぇ灏?
    switch(camera_type)
    {
        case SEEKFREE_ASSISTANT_OV7725_BIN:
        {
            image_size = width * height / 8;
        }break;

        case SEEKFREE_ASSISTANT_MT9V03X:
        {
            image_size = width * height;
        }break;

        case SEEKFREE_ASSISTANT_SCC8660:
        {
            image_size = width * height * 2;
        }break;
    }

    // 鍙戦€佸浘鍍忔暟鎹?
    if(NULL != image_addr)
    {
        seekfree_assistant_transfer_callback(image_addr, image_size);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    閫愰鍔╂墜 鍥惧儚杈圭嚎缁樺埗鍑芥暟
// 鍙傛暟璇存槑     boundary_id     杈圭嚎ID
// 鍙傛暟璇存槑     dot_num         鐐规暟閲?
// 鍙傛暟璇存槑     *dot_x          妯潗鏍囨暟鎹鍦板潃
// 鍙傛暟璇存槑     *dot_y          绾靛潗鏍囨暟鎹鍦板潃
// 鍙傛暟璇存槑     width           鍥惧儚瀹藉害
// 鍙傛暟璇存槑     height          鍥惧儚楂樺害
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_camera_dot_send (seekfree_assistant_camera_buffer_struct *buffer)
{
    uint8  i;
    uint16 dot_bytes = 0;   // 鐐瑰瓧鑺傛暟閲?

    dot_bytes = seekfree_assistant_camera_dot_data.dot_num;

    if(seekfree_assistant_camera_dot_data.dot_type & (1 << 5))
    {
        dot_bytes *= 2;
    }

    // 棣栧厛鍙戦€佸抚澶淬€佸姛鑳姐€佽竟鐣岀紪鍙枫€佸潗鏍囬暱搴︺€佺偣涓暟
    seekfree_assistant_transfer_callback((const uint8 *)&seekfree_assistant_camera_dot_data, sizeof(seekfree_assistant_camera_dot_struct));

    for(i=0; i < SEEKFREE_ASSISTANT_CAMERA_MAX_BOUNDARY; i++)
    {
        // 鍒ゆ柇鏄惁鍙戦€佹í鍧愭爣鏁版嵁
        if(NULL != buffer->boundary_x[i])
        {
            seekfree_assistant_transfer_callback((const uint8 *)buffer->boundary_x[i], dot_bytes);
        }

        // 鍒ゆ柇鏄惁鍙戦€佺旱鍧愭爣鏁版嵁
        if(NULL != buffer->boundary_y[i])
        {
            // 濡傛灉娌℃湁绾靛潗鏍囨暟鎹紝鍒欒〃绀烘瘡涓€琛屽彧鏈変竴涓竟鐣?
            // 鎸囧畾浜嗘í绾靛潗鏍囨暟鎹紝杩欑鏂瑰紡鍙互瀹炵幇鍚屼竴琛屽涓竟鐣岀殑鎯呭喌锛屼緥濡傛悳绾跨畻娉曡兘澶熸悳绱㈠嚭鍥炲集銆?
            seekfree_assistant_transfer_callback((const uint8 *)buffer->boundary_y[i], dot_bytes);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    閫愰鍔╂墜 铏氭嫙绀烘尝鍣ㄥ彂閫佸嚱鏁?
// 鍙傛暟璇存槑     *seekfree_assistant_oscilloscope  绀烘尝鍣ㄦ暟鎹粨鏋勪綋
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_oscilloscope_send (seekfree_assistant_oscilloscope_struct *seekfree_assistant_oscilloscope)
{
    uint8 packet_size;

    // 灏嗛珮鍥涗綅娓呯┖
    seekfree_assistant_oscilloscope->channel_num &= 0x0f;

    zf_assert(SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT >= seekfree_assistant_oscilloscope->channel_num);

    // 甯уご
    seekfree_assistant_oscilloscope->head         = SEEKFREE_ASSISTANT_SEND_HEAD;

    // 鍐欏叆鍖呴暱搴︿俊鎭?
    packet_size                         = sizeof(seekfree_assistant_oscilloscope_struct) - (SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT - seekfree_assistant_oscilloscope->channel_num) * 4;
    seekfree_assistant_oscilloscope->length       = packet_size;

    // 鍐欏叆鍔熻兘瀛椾笌閫氶亾鏁伴噺
    seekfree_assistant_oscilloscope->channel_num |= SEEKFREE_ASSISTANT_CAMERA_OSCILLOSCOPE;

    // 鍜屾牎楠岃绠?
    seekfree_assistant_oscilloscope->check_sum    = 0;
    seekfree_assistant_oscilloscope->check_sum    = seekfree_assistant_sum((uint8 *)seekfree_assistant_oscilloscope, packet_size);

    // 鏁版嵁鍦ㄨ皟鐢ㄦ湰鍑芥暟涔嬪墠锛岀敱鐢ㄦ埛灏嗛渶瑕佸彂閫佺殑鏁版嵁鍐欏叆seekfree_assistant_oscilloscope_data.data[]

    seekfree_assistant_transfer_callback((const uint8 *)seekfree_assistant_oscilloscope, packet_size);
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    閫愰鍔╂墜鍥惧儚淇℃伅閰嶇疆鍑芥暟
// 鍙傛暟璇存槑     camera_type     鍥惧儚绫诲瀷
// 鍙傛暟璇存槑     image_addr      鍥惧儚鍦板潃    濡傛灉浼犻€扤ULL鍙傛暟鍒欒〃绀哄彧鍙戦€佽竟绾夸俊鎭埌涓婁綅鏈?
// 鍙傛暟璇存槑     width           鍥惧儚瀹藉害
// 鍙傛暟璇存槑     height          鍥惧儚楂樺害
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥                     seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
// 澶囨敞淇℃伅
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_camera_information_config (seekfree_assistant_image_type_enum camera_type, void *image_addr, uint16 width, uint16 height)
{
    seekfree_assistant_camera_dot_data.head       = SEEKFREE_ASSISTANT_SEND_HEAD;
    seekfree_assistant_camera_dot_data.function   = SEEKFREE_ASSISTANT_CAMERA_DOT_FUNCTION;
    // 鍐欏叆鍖呴暱搴︿俊鎭?
    seekfree_assistant_camera_dot_data.length     = sizeof(seekfree_assistant_camera_dot_struct);

    seekfree_assistant_camera_buffer.camera_type  = camera_type;
    seekfree_assistant_camera_buffer.image_addr   = image_addr;
    seekfree_assistant_camera_buffer.width        = width;
    seekfree_assistant_camera_buffer.height       = height;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    閫愰鍔╂墜鍥惧儚杈圭嚎鍙戦€侀厤缃嚱鏁?
// 鍙傛暟璇存槑     boundary_type   杈圭晫绫诲瀷
// 鍙傛暟璇存槑     dot_num         涓€鏉¤竟鐣屾湁澶氬皯涓偣
// 鍙傛暟璇存槑     dot_x1          瀛樻斁杈圭嚎1妯潗鏍囩殑鍦板潃  濡傛灉浼犻€扤ULL鍙傛暟鍒欒〃绀轰笉鍙戦€佽竟绾?
// 鍙傛暟璇存槑     dot_x2          瀛樻斁杈圭嚎2妯潗鏍囩殑鍦板潃  濡傛灉浼犻€扤ULL鍙傛暟鍒欒〃绀轰笉鍙戦€佽竟绾?
// 鍙傛暟璇存槑     dot_x3          瀛樻斁杈圭嚎3妯潗鏍囩殑鍦板潃  濡傛灉浼犻€扤ULL鍙傛暟鍒欒〃绀轰笉鍙戦€佽竟绾?
// 鍙傛暟璇存槑     dot_y1          瀛樻斁杈圭嚎1绾靛潗鏍囩殑鍦板潃  濡傛灉浼犻€扤ULL鍙傛暟鍒欒〃绀轰笉鍙戦€佽竟绾?
// 鍙傛暟璇存槑     dot_y2          瀛樻斁杈圭嚎2绾靛潗鏍囩殑鍦板潃  濡傛灉浼犻€扤ULL鍙傛暟鍒欒〃绀轰笉鍙戦€佽竟绾?
// 鍙傛暟璇存槑     dot_y3          瀛樻斁杈圭嚎3绾靛潗鏍囩殑鍦板潃  濡傛灉浼犻€扤ULL鍙傛暟鍒欒〃绀轰笉鍙戦€佽竟绾?
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥                     seekfree_assistant_camera_config(X_BOUNDARY, MT9V03X_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL, NULL);     // 鍥惧儚鍙戦€佹椂鍖呭惈涓夋潯杈圭嚎锛岃竟绾垮彧鏈夋í鍧愭爣
// 浣跨敤绀轰緥                     seekfree_assistant_camera_config(Y_BOUNDARY, MT9V03X_W, NULL, NULL, NULL, y1_boundary, y2_boundary, y3_boundary);     // 鍥惧儚鍙戦€佹椂鍖呭惈涓夋潯杈圭嚎锛岃竟绾垮彧鏈夌旱鍧愭爣
// 浣跨敤绀轰緥                     seekfree_assistant_camera_config(XY_BOUNDARY, 160, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);   // 鍥惧儚鍙戦€佹椂鍖呭惈涓夋潯杈圭嚎锛岃竟绾垮寘鍚í绾靛潗鏍?
// 澶囨敞淇℃伅
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_camera_boundary_config (seekfree_assistant_boundary_type_enum boundary_type, uint16 dot_num, void *dot_x1, void *dot_x2, void *dot_x3, void *dot_y1, void *dot_y2, void *dot_y3)
{
    uint8 i = 0;
    uint8 boundary_num = 0;
    uint8 boundary_data_type = 0;

    // 妫€鏌ュ浘鍍忓彂閫佺紦鍐插尯鏄惁鍑嗗灏辩华, 璋冪敤姝ゅ嚱鏁颁箣鍓嶉渶瑕佸厛璋冪敤seekfree_assistant_camera_config璁剧疆濂藉浘鍍忎俊鎭?
    zf_assert(0 != seekfree_assistant_camera_buffer.camera_type);

    seekfree_assistant_camera_dot_data.dot_num    = dot_num;
    seekfree_assistant_camera_dot_data.valid_flag = 0;
    for(i = 0; i < 3; i++)
    {
        seekfree_assistant_camera_buffer.boundary_x[i] = NULL;
        seekfree_assistant_camera_buffer.boundary_y[i] = NULL;
    }

    switch(boundary_type)
    {
        case X_BOUNDARY:
        {
            if(NULL != dot_x1)
            {
                boundary_num++;
                seekfree_assistant_camera_dot_data.valid_flag |= 1 << 0;
                seekfree_assistant_camera_buffer.boundary_x[i++] = dot_x1;
            }
            if(NULL != dot_x2)
            {
                boundary_num++;
                seekfree_assistant_camera_dot_data.valid_flag |= 1 << 1;
                seekfree_assistant_camera_buffer.boundary_x[i++] = dot_x2;
            }
            if(NULL != dot_x3)
            {
                boundary_num++;
                seekfree_assistant_camera_dot_data.valid_flag |= 1 << 2;
                seekfree_assistant_camera_buffer.boundary_x[i++] = dot_x3;
            }

            if(255 < seekfree_assistant_camera_buffer.height)
            {
                boundary_data_type = 1;
            }
        }break;

        case Y_BOUNDARY:
        {
            if(NULL != dot_y1)
            {
                boundary_num++;
                seekfree_assistant_camera_dot_data.valid_flag |= 1 << 0;
                seekfree_assistant_camera_buffer.boundary_y[i++] = dot_y1;
            }
            if(NULL != dot_y2)
            {
                boundary_num++;
                seekfree_assistant_camera_dot_data.valid_flag |= 1 << 1;
                seekfree_assistant_camera_buffer.boundary_y[i++] = dot_y2;
            }
            if(NULL != dot_y3)
            {
                boundary_num++;
                seekfree_assistant_camera_dot_data.valid_flag |= 1 << 2;
                seekfree_assistant_camera_buffer.boundary_y[i++] = dot_y3;
            }

            if(255 < seekfree_assistant_camera_buffer.width)
            {
                boundary_data_type = 1;
            }
        }break;

        case XY_BOUNDARY:
        {
            if((NULL != dot_x1) && (NULL != dot_y1))
            {
                boundary_num++;
                seekfree_assistant_camera_dot_data.valid_flag |= 1 << 0;
                seekfree_assistant_camera_buffer.boundary_x[i]   = dot_x1;
                seekfree_assistant_camera_buffer.boundary_y[i++] = dot_y1;
            }
            if((NULL != dot_x2) && (NULL != dot_y2))
            {
                boundary_num++;
                seekfree_assistant_camera_dot_data.valid_flag |= 1 << 1;
                seekfree_assistant_camera_buffer.boundary_x[i]   = dot_x2;
                seekfree_assistant_camera_buffer.boundary_y[i++] = dot_y2;
            }
            if((NULL != dot_x3) && (NULL != dot_y3))
            {
                boundary_num++;
                seekfree_assistant_camera_dot_data.valid_flag |= 1 << 2;
                seekfree_assistant_camera_buffer.boundary_x[i]   = dot_x3;
                seekfree_assistant_camera_buffer.boundary_y[i++] = dot_y3;
            }

            if((255 < seekfree_assistant_camera_buffer.width) || (255 < seekfree_assistant_camera_buffer.height))
            {
                boundary_data_type = 1;
            }
        }break;

        case NO_BOUNDARY:break;
    }

    seekfree_assistant_camera_dot_data.dot_type   = (boundary_type << 6) | (boundary_data_type << 5) | boundary_num;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    閫愰鍔╂墜鍙戦€佹憚鍍忓ご鍥惧儚
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥
// 澶囨敞淇℃伅     鍦ㄨ皟鐢ㄥ浘鍍忓彂閫佸嚱鏁颁箣鍓嶏紝璇峰姟蹇呰皟鐢ㄤ竴娆eekfree_assistant_camera_config鍑芥暟锛屽皢瀵瑰簲鐨勫弬鏁拌缃ソ
//-------------------------------------------------------------------------------------------------------------------
void seekfree_assistant_camera_send (void)
{
    // 妫€鏌ュ浘鍍忓彂閫佺紦鍐插尯鏄惁鍑嗗灏辩华
    zf_assert(0 != seekfree_assistant_camera_buffer.camera_type);

    seekfree_assistant_camera_data_send(seekfree_assistant_camera_buffer.camera_type, seekfree_assistant_camera_buffer.image_addr, seekfree_assistant_camera_dot_data.dot_type & 0x0f, seekfree_assistant_camera_buffer.width, seekfree_assistant_camera_buffer.height);

    if(seekfree_assistant_camera_dot_data.dot_type & 0x0f)
    {
        seekfree_assistant_camera_dot_send(&seekfree_assistant_camera_buffer);
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    閫愰鍔╂墜瑙ｆ瀽鎺ユ敹鍒扮殑鏁版嵁
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     鍑芥暟鍙渶瑕佹斁鍒板懆鏈熻繍琛岀殑PIT涓柇鎴栬€呬富寰幆鍗冲彲
//-------------------------------------------------------------------------------------------------------------------
#if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)
void seekfree_assistant_data_analysis (void)
{
    uint8  temp_sum;
    uint32 read_length;
    seekfree_assistant_parameter_struct *receive_packet;

    // 杩欓噷浣跨敤uint32杩涜瀹氫箟锛岀洰鐨勬槸涓轰簡淇濊瘉鏁扮粍鍥涘瓧鑺傚榻?
    uint32  temp_buffer[SEEKFREE_ASSISTANT_BUFFER_SIZE / 4];

    // 灏濊瘯璇诲彇鏁版嵁, 濡傛灉涓嶆槸鑷畾涔夌殑浼犺緭鏂瑰紡鍒欎粠鎺ユ敹鍥炶皟涓鍙栨暟鎹?
    read_length = seekfree_assistant_receive_callback((uint8 *)temp_buffer, SEEKFREE_ASSISTANT_BUFFER_SIZE);
    if(read_length)
    {
        // 灏嗚鍙栧埌鐨勬暟鎹啓鍏IFO
        fifo_write_buffer(&seekfree_assistant_fifo, (uint8 *)temp_buffer, read_length);
    }

    while(sizeof(seekfree_assistant_parameter_struct) <= fifo_used(&seekfree_assistant_fifo))
    {
        read_length = sizeof(seekfree_assistant_parameter_struct);
        fifo_read_buffer(&seekfree_assistant_fifo, (uint8 *)temp_buffer, &read_length, FIFO_READ_ONLY);

        if(SEEKFREE_ASSISTANT_RECEIVE_HEAD != ((uint8 *)temp_buffer)[0])
        {
            // 娌℃湁甯уご鍒欎粠FIFO涓幓鎺夌涓€涓暟鎹?
            read_length = 1;
        }
        else
        {
            // 鎵惧埌甯уご
            receive_packet = (seekfree_assistant_parameter_struct *)temp_buffer;
            temp_sum = receive_packet->check_sum;
            receive_packet->check_sum = 0;
            if(temp_sum == seekfree_assistant_sum((uint8 *)temp_buffer, sizeof(seekfree_assistant_parameter_struct)))
            {
                // 鍜屾牎楠屾垚鍔熶繚瀛樻暟鎹?
                seekfree_assistant_parameter[receive_packet->channel - 1] = receive_packet->data;
                seekfree_assistant_parameter_update_flag[receive_packet->channel - 1] = 1;
            }
            else
            {
                read_length = 1;
            }
        }

        // 涓㈠純鏃犻渶浣跨敤鐨勬暟鎹?
        fifo_read_buffer(&seekfree_assistant_fifo, (uint8 *)temp_buffer, &read_length, FIFO_READ_AND_CLEAN);
    }
}
#endif



