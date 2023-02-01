/*
 Project: OOK UHF Receiver
 Date: 2023, Feb, 1
 Author: 7M4MON
 
 Description:
 [OOK UHF Transmitter] から送信された2値ASK信号を SYN480Rで受信するプログラムです。
 リアルタイム処理でクロック再生を実行し、メインループでUARTステートマシンを構成しています。
 任意の受信周波数を設定できるように、クロックジェネレータ[Si5351A]を使用しています。
 受信可能周波数は、438.100 MHz から 100kHzステップ 9波です。
 無線区間のUARTの仕様は Bit Rate:1000bps, Data bits:8, Parity: None, Stop bit: 1 です。
 PCとの通信は Bit Rate:115200bps, Data bits:8, Parity: None, Stop bit: 1 です。
*/

#include <FrequencyTimer2.h>
#include "si5351.h"
#include "Wire.h"

Si5351 si5351;
#define SI5351_CARIB_DATA -15000 // 周波数が許容偏差に入るように調整する

/* ピンの定義 */
#define PIN_CLK_OUT_PB  PB0      // PIN 8
#define PIN_CLK_OUT     8
#define PIN_DATA_OUT_PB PB1      // PIN 9
#define PIN_DATA_OUT    9
#define PIN_DATA_IN_PD  2
#define PIN_DATA_IN     2
#define PIN_MODE_SW     3
#define PIN_RX_LED      10
#define PIN_RX_LED_PB   PB2      // PIN 10
// CH設定用のロータリースイッチの接続先
#define PIN_HEX_SW1      4
#define PIN_HEX_SW2      6
#define PIN_HEX_SW4      5
#define PIN_HEX_SW8      7

/* brink_ch_morse() 用のモールス符号の定義 */
/* 0b01は短点、0b11は長点 */
static uint16_t ch_mose_code[11] = {
    0b1111111111, 0b0111111111, 0b0101111111, 0b0101011111, 0b0101010111,   // 0,1,2,3,4
    0b0101010101, 0b1101010101, 0b1111010101, 0b1111110101, 0b1111111101,   // 5,6,7,8,9
    0b1101111101                                                            // NG
};

/* 現在設定されたチャンネルをモールスコードを光らせて知らせる */
/* 例：5ch が設定されていたら短く5回点滅する */
void brink_ch_morse(uint16_t code) {
    uint16_t dot_len = 100;
    if (code != 0) {
        while ( (code & 0xC000) == 0 ) {
            code <<= 2; // 頭出し
        }
        while ( (code & 0xC000) != 0 ) {
            uint16_t t = (code & 0xC000) >> 14;     // t = 1 or 3;
            digitalWrite(PIN_RX_LED, HIGH);
            delay(dot_len * t);
            digitalWrite(PIN_RX_LED, LOW);
            delay(dot_len);
            code <<= 2;
        }
        delay(dot_len * 3);
    }
}


#define HEX_SW_POSITIVE     // 負論理スイッチの場合はコメントアウト
/* ロータリーコードスイッチの位置を読む */
int8_t read_hex_pos() {
    int8_t pos = digitalRead(PIN_HEX_SW1) + digitalRead(PIN_HEX_SW2) * 2 + digitalRead(PIN_HEX_SW4) * 4 + digitalRead(PIN_HEX_SW8) * 8;
#ifdef HEX_SW_POSITIVE
    pos = 15 - pos;         // 正論理のスイッチの場合は反転する。
#endif
    return pos;
}

/* 64bit型のprintlnは用意されてないので自作 */
void println_uint64(uint64_t num_64) {
    uint32_t num_h = (num_64 >> 32) & 0xFFFFFFFF;
    uint32_t num_l = num_64 & 0xFFFFFFFF;
    Serial.print(num_h);
    Serial.println(num_l);
}

/* 受信周波数を設定するための定数 */
#define FREQ_START 438000UL
#define FREQ_STEP     100UL
#define FREQ_STOP  (FREQ_START+FREQ_STEP*9) // 438.900MHz
#define XTAL_FREQ_START   680923736ULL      // 6.809 237 36MHz @ 438.0MHz , 1Hz = 100ULL
//#define XTAL_FREQ_START 681079198ULL      // 6.810 791 98MHz @ 438.1MHz , 1Hz = 100ULL
#define XTAL_FREQ_DIFF_KHZ 155462ULL        // 1554.62Hz@100kHz
//#define XTAL_FREQ_DIFF_KHZ 31092ULL       // 310.92Hz@20k

/* ロータリースイッチの状態を読んでクロックを設定する */
void set_rx_freq() {
    uint32_t freq_khz;
    int8_t hex_pos = read_hex_pos();
    Serial.print("hexpos:");
    Serial.println(hex_pos);
    if (hex_pos == 0){
        /* 438.000MHz はエッジのため設定不可にする */
        while(1){
            brink_ch_morse(ch_mose_code[hex_pos]);
            // （CHを変更してから）リセット待ち
            Serial.println("This channel is not parmitted, Change channel, then reset.");
        }
    }else{
        freq_khz = FREQ_START  + FREQ_STEP * (uint32_t)hex_pos;
        Serial.print("freq_khz:");
        Serial.println(freq_khz);
        uint64_t freq_xtal = XTAL_FREQ_START + XTAL_FREQ_DIFF_KHZ * hex_pos;
        println_uint64(freq_xtal);
        si5351.set_freq(freq_xtal, SI5351_CLK0);
        si5351.update_status();
        brink_ch_morse(ch_mose_code[hex_pos]);
    }
}

/* クロック再生関連の定数 */
#define OVSP_ODR        5                // オーバーサンプリングの倍数
#define SMBL_POINT      2                // OVSP_ODRの中点付近
#define OVSP_PERIOD_US  400              // (1/BIT_RATE * OVSP_ODR) の2倍の値を設定する
#define NO_EDGE_TIMER (OVSP_ODR*32)      // 32bit分エッジがなかったらno clock

/* クロック再生関連の変数 */
uint16_t ovsp_counter = 0;
uint16_t edge_counter = 0;
bool edge_det = false;
bool data_set = false;
uint8_t last_bit, rx_bit;

/* クロック再生（リアルタイム処理）*/
/* (BIT_RATE/OVSP_ODR) の周期で呼び出される。*/
/* この処理は実測4～5us程度で完了 */
void rt_clock_recovery(void) {
    uint8_t get_bit = (PIND & _BV(PIN_DATA_IN));    // ピンの状態を取得

    if (edge_counter > NO_EDGE_TIMER) {         // edge_counterのオーバーフローの防止
        edge_counter = NO_EDGE_TIMER;
        edge_det = false;
    }
    if (edge_counter > SMBL_POINT) {            // シンボルポイント後のエッジでクロックの引き込み
        if (get_bit != last_bit) {              // データ変化のエッジを検出したのでクロックを引き込む
            edge_counter = 0;
            ovsp_counter = 0;
            edge_det = true;
        }
    }
    if (ovsp_counter > OVSP_ODR - 1 ) {         // 1周期カウントしたらクリア（UARTはNRZなので）
        ovsp_counter = 0;
    }

    if (ovsp_counter == 0) {                    // クロックの立ち下がりポイント
        PORTB |=  _BV(PIN_CLK_OUT_PB) ;         // クロックL出力
    } else if (ovsp_counter == SMBL_POINT) {    // クロックの立ち上がりポイント（シンボル点）
        PORTB &= ~_BV(PIN_CLK_OUT_PB);          // クロックH出力
        rx_bit = get_bit ? 1 : 0;               // データの取り込み
        data_set = true;                        // main loop に通知
        PORTB = rx_bit ? PORTB | _BV(PIN_DATA_OUT_PB) : PORTB & ~_BV(PIN_DATA_OUT_PB);  // データを出力
    }
    ovsp_counter++;
    edge_counter++;
    last_bit = get_bit;                          // 結果を上書き
}


/* 電源投入時にピンやデバイスの初期化を行う */
void setup() {
    pinMode(PIN_HEX_SW1, INPUT_PULLUP);
    pinMode(PIN_HEX_SW2, INPUT_PULLUP);
    pinMode(PIN_HEX_SW4, INPUT_PULLUP);
    pinMode(PIN_HEX_SW8, INPUT_PULLUP);
    pinMode(PIN_CLK_OUT, OUTPUT);
    pinMode(PIN_DATA_OUT, OUTPUT);
    pinMode(PIN_MODE_SW, INPUT_PULLUP);
    pinMode(PIN_DATA_IN, INPUT_PULLUP);
    pinMode(PIN_RX_LED, OUTPUT);
    digitalWrite(PIN_RX_LED, LOW);

    Serial.begin(115200);   //PCとの通信用

    /* Si5351Aのセットアップ */
    bool si5351a_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
    if (!si5351a_found) {
        /* Si5351Aが見つからなかったら機能停止 */
        Serial.println("Si5351a not found");
        brink_ch_morse(ch_mose_code[10]);
        while (1) {
            ; //trap
        }
    } else {
        /* Si5351Aが見つかったら周波数を設定する */
        si5351.set_correction(SI5351_CARIB_DATA, SI5351_PLL_INPUT_XO);
        set_rx_freq();
    }

    /* リアルタイム処理のタイマ割り込みを開始 */
    FrequencyTimer2::setPeriod(OVSP_PERIOD_US);
    FrequencyTimer2::setOnOverflow(rt_clock_recovery);
    FrequencyTimer2::enable();
    Serial.println("RT START");

    delay(500);
}

#define START_H_LENGTH 10               // 開始ワードの最小H回数
#define SYNC_WORD 0b0111011001010000    // 同期ワードは D-STAR と同じにしてみた。

/* UARTステートマシンの状態 */
enum uart_state_list {
    STATE_IDLE ,    // 同期ワード検出待ち
    STATE_SYNC ,    // 開始ワード待ち
    STATE_START ,   // スタートビット待ち
    STATE_ACQ ,     // データ取り込み中
    STATE_STOP,     // ストップビット待ち
    STATE_BREAK     // 次のデータor終了ワード待ち
};

volatile uint16_t shift_reg = 0;        // 同期ワード検出用のシフトレジスタ
uint8_t uart_state = STATE_IDLE;        // ステートマシンのステート

uint16_t rcv_byte_counter = 0;
uint8_t acq_counter = 0;
uint8_t send_byte = 0;
uint8_t stop_bits = 1;

/* 受信結果通知用の送信バッファ */
#define MAX_BUFF_SIZE 63
uint8_t send_buff[MAX_BUFF_SIZE];

/* メインループ */
void loop() {
    if (data_set) {                                         // リアルタイム処理でシンボル点を検出した
        data_set = false;                                   // フラグをクリア
        switch (uart_state) {                               // ステートマシンに応じた処理
            case STATE_IDLE:                                // 同期ワード検出待ちの場合
                shift_reg <<= 1;                            // 同期ワードを格納していく
                shift_reg |= (uint16_t) rx_bit;
                if (shift_reg == SYNC_WORD) {               // 同期ワードが一致したら
                    uart_state = STATE_SYNC;                // 開始ワード検出待ちに移行
                    PORTB |=  _BV(PIN_RX_LED_PB) ;          // 受信LEDを光らせる
                    memset(send_buff, 0, MAX_BUFF_SIZE);    // 送信バッファを初期化
                }
                break;
            case STATE_SYNC:                                // 開始ワード検出待ちの場合
                if (rx_bit) {
                    acq_counter++;                          // Hだったら開始ワード検出を続行する
                } else {
                    shift_reg = 0;                          // Lだったら破棄してアイドル状態に戻る
                    uart_state = STATE_IDLE;
                    PORTB &= ~_BV(PIN_RX_LED_PB);           // 受信LEDを消灯
                    acq_counter = 0;
                }
                if (acq_counter > START_H_LENGTH) {         // 開始ワード検出が完了
                    uart_state = STATE_START;               // スタートビット検出待ちに移行
                    acq_counter = 0;
                }
                break;
            case STATE_START:
                if (!rx_bit) {                              // スタートビット検出待ちの場合
                    uart_state = STATE_ACQ;                 // スタートビット検出でデータ取得状態に移行
                    acq_counter = 0;
                    send_byte = 0;
                } else {
                    acq_counter++;
                    if (acq_counter > 100) {                // 一向にスタートビットが来ない場合はタイムアウト
                        shift_reg = 0;
                        uart_state = STATE_IDLE;            // 破棄してアイドル状態に戻る
                        PORTB &= ~_BV(PIN_RX_LED_PB);       // 受信LEDを消灯
                        acq_counter = 0;
                    }
                }
                break;
            case STATE_ACQ:                                 // データ取得中
                send_byte >>= 1;
                if (rx_bit) send_byte |= 0x80;              // LSBファーストで送ってくるので。上位ビットから格納していく。
                acq_counter++;
                if (acq_counter > 7) {                      // 8ビット分入れたらストップビット待ちに移行
                    send_buff[rcv_byte_counter] = send_byte;
                    rcv_byte_counter++;
                    send_byte = 0;
                    acq_counter = 0;
                    uart_state = STATE_STOP;
                }
                break;
            case STATE_STOP:                                // ストップビット待ち状態
                if (rx_bit) {
                    acq_counter++;                          // Hだったらストップビット検出
                }
                if (acq_counter >= stop_bits - 1) {         // 規定のビット数に達した
                    uart_state = STATE_BREAK;               // 終了ワードor次のデータを待つ
                    acq_counter = 0;
                }
                break;
            case STATE_BREAK:
                if (!rx_bit && (rcv_byte_counter < MAX_BUFF_SIZE)) {    // 次のスタートビットが来た
                    uart_state = STATE_ACQ;                             // データ取得状態に移行
                    acq_counter = 0;
                    send_byte = 0;
                } else {
                    acq_counter++;
                    if (acq_counter > 10) {                             // Hが続いたので送信終了とみなす
                        FrequencyTimer2::disable();                     // 一旦リアルタイム処理は停止
                        for (uint8_t i = 0; i < rcv_byte_counter; i++) {
                            Serial.write(send_buff[i]);                 // 受信したデータをPCに送信
                        }
                        rcv_byte_counter = 0;                           // 後始末
                        shift_reg = 0;
                        acq_counter = 0;
                        memset(send_buff, 0, MAX_BUFF_SIZE);
                        uart_state = STATE_IDLE;
                        PORTB &= ~_BV(PIN_RX_LED_PB);                   // 受信LEDを消灯
                        FrequencyTimer2::enable();                      // リアルタイム処理を再開
                    }
                }
                break;
            default:
                break;
        }
    }

    /* PCから '?' が来たらステートマシンの状態を知らせる（デバッグ用） */
    if ( Serial.available()) {
        byte commandChr = Serial.read();
        switch (commandChr) {
            case '?':
                Serial.print("uart_state: ");
                Serial.println(uart_state);
                break;
            default :
                break;
        }
    }
}
