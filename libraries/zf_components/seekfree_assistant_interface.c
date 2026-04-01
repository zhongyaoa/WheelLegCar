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
* 鏂囦欢鍚嶇О          seekfree_assistant_interface
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

#include "zf_common_typedef.h"
#include "zf_common_fifo.h"
#include "zf_common_debug.h"
#include "zf_driver_uart.h"
#include "zf_device_wireless_uart.h"
#include "zf_device_wifi_uart.h"
#include "zf_device_wifi_spi.h"
#include "zf_device_ble6a20.h"
#include "seekfree_assistant.h"

#include "seekfree_assistant_interface.h"


extern seekfree_assistant_transfer_callback_function   seekfree_assistant_transfer_callback;    // 鏁版嵁鍙戦€佸嚱鏁版寚閽?
extern seekfree_assistant_receive_callback_function    seekfree_assistant_receive_callback;     // 鏁版嵁鎺ユ敹鍑芥暟鎸囬拡


//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    閫愰鍔╂墜鍙戦€佸嚱鏁?
// 鍙傛暟璇存槑     *buff           闇€瑕佸彂閫佺殑鏁版嵁鍦板潃
// 鍙傛暟璇存槑     length          闇€瑕佸彂閫佺殑闀垮害
// 杩斿洖鍙傛暟     uint32          鍓╀綑鏈彂閫佹暟鎹暱搴?
// 浣跨敤绀轰緥
//-------------------------------------------------------------------------------------------------------------------
ZF_WEAK uint32 seekfree_assistant_transfer (const uint8 *buff, uint32 length)
{
    
    // 褰撻€夋嫨鑷畾涔夐€氳鏂瑰紡鏃?闇€瑕佽嚜琛屽畬鎴愭暟鎹彂閫佸姛鑳?
    return length;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    閫愰鍔╂墜鎺ユ敹鏁版嵁鍑芥暟
// 鍙傛暟璇存槑     *buff           闇€瑕佹帴鏀剁殑鏁版嵁鍦板潃
// 鍙傛暟璇存槑     length          瑕佹帴鏀剁殑鏁版嵁鏈€澶ч暱搴?
// 杩斿洖鍙傛暟     uint32          鎺ユ敹鍒扮殑鏁版嵁闀垮害
// 浣跨敤绀轰緥
//-------------------------------------------------------------------------------------------------------------------
ZF_WEAK uint32 seekfree_assistant_receive (uint8 *buff, uint32 length)
{
    // 褰撻€夋嫨鑷畾涔夐€氳鏂瑰紡鏃?闇€瑕佽嚜琛屽畬鎴愭暟鎹帴鏀跺姛鑳?
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    閫愰鍔╂墜鎺ュ彛 鍒濆鍖?
// 鍙傛暟璇存槑
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI); 浣跨敤楂橀€焀IFI SPI妯″潡杩涜鏁版嵁鏀跺彂
// 澶囨敞         闇€瑕佽嚜琛岃皟鐢ㄨ澶囩殑鍒濆鍖栵紝渚嬪浣跨敤鏃犵嚎杞覆鍙ｈ繘琛屾暟鎹殑鏀跺彂锛屽垯闇€瑕佽嚜琛岃皟鐢ㄦ棤绾胯浆涓插彛鐨勫垵濮嬪寲锛岀劧鍚庡啀璋冪敤seekfree_assistant_interface_init瀹屾垚閫愰鍔╂墜鐨勬帴鍙ｅ垵濮嬪寲
//-------------------------------------------------------------------------------------------------------------------
ZF_WEAK void seekfree_assistant_interface_init (seekfree_assistant_transfer_device_enum transfer_device)
{
    switch(transfer_device)
    {
        case SEEKFREE_ASSISTANT_DEBUG_UART:
        {
            seekfree_assistant_transfer_callback = debug_send_buffer;
            seekfree_assistant_receive_callback = debug_read_ring_buffer;
        }break;
        
        case SEEKFREE_ASSISTANT_WIRELESS_UART:
        {
            seekfree_assistant_transfer_callback = wireless_uart_send_buffer;
            seekfree_assistant_receive_callback = wireless_uart_read_buffer;
        }break;
        
        case SEEKFREE_ASSISTANT_BLE6A20:
        {
            seekfree_assistant_transfer_callback = ble6a20_send_buffer;
            seekfree_assistant_receive_callback = ble6a20_read_buffer;
        }break;
        
        case SEEKFREE_ASSISTANT_WIFI_UART:
        {
            seekfree_assistant_transfer_callback = wifi_uart_send_buffer;
            seekfree_assistant_receive_callback = wifi_uart_read_buffer;
        }break;
        
        case SEEKFREE_ASSISTANT_WIFI_SPI:
        {
            seekfree_assistant_transfer_callback = wifi_spi_send_buffer;
            seekfree_assistant_receive_callback = wifi_spi_read_buffer;
        }break;
        
        case SEEKFREE_ASSISTANT_CUSTOM:
        {         
            // 鏍规嵁鑷繁鐨勯渶姹?鑷瀹炵幇seekfree_assistant_transfer涓巗eekfree_assistant_receive鍑芥暟锛屽畬鎴愭暟鎹殑鏀跺彂
            
        }break;
    }
}


