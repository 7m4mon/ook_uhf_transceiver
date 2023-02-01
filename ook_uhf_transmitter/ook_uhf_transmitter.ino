/*
 Project: OOK UHF Transmitter
 Date: 2023, Feb, 1
 Author: 7M4MON

 Description:
 PC等から受け取った任意の63バイトまでのデータを SYN115 から2値ASKで送信します。
 受信側のプログラムは [OOK UHF Receiver] です。
 任意の送信周波数を設定できるように、クロックジェネレータ[Si5351A]を使用しています。
 送信可能周波数は、438.100 MHz から 100kHzステップ 9波です。
 無線区間のUARTの仕様は Bit Rate:1000bps, Data bits:8, Parity: None, Stop bit: 1 です。
 PCとの通信は Bit Rate:115200bps, Data bits:8, Parity: None, Stop bit: 1 です。
*/
#include <FrequencyTimer2.h>
#include <SoftwareSerial.h>
#include "si5351.h"
#include "Wire.h"

Si5351 si5351;
#define SI5351_CARIB_DATA -21000 // 周波数が許容偏差に入るように調整する

/* ピンの定義 */
#define PIN_CLK  8          // PB0 PNモード クロック出力
#define PIN_DATA 9          // PB1 PNモード データ出力
#define PIN_TX_LED PIN_CLK  // 送信LEDはクロックと同じピン
#define PIN_SER_TX PIN_DATA // UARTモードでのデータ送信 （PIN_DATAと同じピン）
#define PIN_SER_RX 10
// ON-OFF-ONのトグルスイッチ。OFFでUART送信
#define PIN_PRBS_SW 2
#define PIN_CWTX_SW 3
// CH設定用のロータリースイッチの接続先
#define PIN_HEX_SW1 4
#define PIN_HEX_SW2 6
#define PIN_HEX_SW4 5
#define PIN_HEX_SW8 7

/* brink_ch_morse() 用のモールス符号の定義 */
/* 0b01は短点、0b11は長点 */
static uint16_t ch_mose_code[11] = {
    0b1111111111, 0b0111111111, 0b0101111111, 0b0101011111, 0b0101010111, // 0,1,2,3,4
    0b0101010101, 0b1101010101, 0b1111010101, 0b1111110101, 0b1111111101, // 5,6,7,8,9
    0b1101111101                                                          // NG
};

/* 現在設定されたチャンネルをモールスコードを光らせて知らせる */
/* 例：5ch が設定されていたら短く5回点滅する */
void brink_ch_morse(uint16_t code) {
    uint16_t dot_len = 100;
    if (code != 0) {
        while ((code & 0xC000) == 0) {
            code <<= 2;
        } // 頭出し
        while ((code & 0xC000) != 0) {
            uint16_t t = (code & 0xC000) >> 14; // t = 1 or 3;
            digitalWrite(PIN_TX_LED, HIGH);
            delay(dot_len * t);
            digitalWrite(PIN_TX_LED, LOW);
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
    uint32_t num_h = (num_64 >> 32) % 0xFFFFFFFF;
    uint32_t num_l = num_64 % 0xFFFFFFFF;
    Serial.print(num_h);
    Serial.println(num_l);
}

/* 送信周波数を設定するための定数 */
#define FREQ_START 438000UL
#define FREQ_STEP  100UL

/* ロータリースイッチの状態を読んでクロックを設定する */
int8_t last_hex_pos = -1;
void set_tx_freq() {
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
        freq_khz = FREQ_START + FREQ_STEP * (uint32_t)hex_pos;
        Serial.print("freq_khz:");
        Serial.println(freq_khz);
        uint64_t freq_xtal = (uint64_t)freq_khz * 100000ULL;
        freq_xtal = (uint64_t)freq_xtal >> 5;       // 送信周波数の 32分の1 の値を設定する
        Serial.print("freq_xtal:");
        println_uint64(freq_xtal);                  
        si5351.set_freq(freq_xtal, SI5351_CLK0);
        si5351.update_status();
        brink_ch_morse(ch_mose_code[hex_pos]);
    }
}


/* 疑似ランダム符号のタップ位置の定義 (CCITT準拠) */
#define PN9
#if defined(PN9)    // CCITT O.153
#define TAP1 4
#define TAP2 8
#elif defined(PN11) // CCITT O.153
#define TAP1 10
#define TAP2 8
#elif defined(PN15) // CCITT O.151, 出力時にデータ反転が必要
#define TAP1 14
#define TAP2 13
#endif

/* 疑似ランダム符号を送信する */
/* ビットレートの半分で割り込みが入り、2回に1回、データをセットする */
bool tick = false;
volatile uint16_t shift_reg = 0xffff;

void set_prbs(void) {
    bool ex;
    if (tick) {
        ex = ((shift_reg & (1 << TAP1)) >> TAP1) ^ ((shift_reg & (1 << TAP2)) >> TAP2); // タップの位置のEXORをとる。
        shift_reg <<= 1;
        shift_reg |= (uint16_t)ex;
        PORTB &= ~_BV(0);           // クロックL出力
#ifdef PN15
        ex = !ex;
#endif
        PORTB = ex ? PORTB | _BV(1) : PORTB & ~_BV(1); // データを出力
    } else {
        PORTB |= _BV(0);            // クロックH出力
    }
    tick = !tick;
}


/* 送信するデータをバッファリングする */
/* 先にバッファリンスしておかないとしておかないとデータが欠けてしまうため */
#define MAX_BUFF_SIZE 63
uint8_t send_buff[MAX_BUFF_SIZE];
uint8_t rcv_bytes = 0;

void store_serial_data() {
    while (Serial.available() && (rcv_bytes < MAX_BUFF_SIZE)) {
        send_buff[rcv_bytes] = Serial.read();
        rcv_bytes++;
    }
}

/* ビット数を指定してビット列を送信する */
/* raw_word : 16bitの生データ, bit_len : 送信するビット数(MSBファースト)*/
void send_raw_word(uint16_t raw_word, uint8_t bit_len) {
    for (uint8_t i = 0; i < bit_len; i++) {
        PORTB = (raw_word & 0x8000) ? PORTB | _BV(1) : PORTB & ~_BV(1);
        delay(1); // このモデムは1000bpsなので1ms待てば良い。
        raw_word <<= 1;
    }
}


#define DATA_RATE_BPS    1000    // ビットレート
#define PERIOD_HALF_US   1000    // クロックの半分の周期の2倍の値を指定する。
SoftwareSerial mySerial(PIN_SER_RX, PIN_SER_TX, false); // RX, TX, INV

/* 電源投入時にピンやデバイスの初期化を行う */
void setup() {
    pinMode(PIN_PRBS_SW, INPUT_PULLUP);
    pinMode(PIN_CWTX_SW, INPUT_PULLUP);
    pinMode(PIN_HEX_SW1, INPUT_PULLUP);
    pinMode(PIN_HEX_SW2, INPUT_PULLUP);
    pinMode(PIN_HEX_SW4, INPUT_PULLUP);
    pinMode(PIN_HEX_SW8, INPUT_PULLUP);
    pinMode(PIN_DATA, OUTPUT);
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_DATA, LOW);
    digitalWrite(PIN_CLK, LOW);

    Serial.begin(115200); // PCとの通信用

    /* Si5351Aのセットアップ */
    bool si5351a_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
    if (!si5351a_found) {
        /* Si5351Aが見つからなかったら機能停止 */
        Serial.println("Si5351a not found");
        brink_ch_morse(ch_mose_code[10]);
        while (1) {
            ;
        } // trap
    } else {
        /* Si5351Aが見つかったら周波数を設定する */
        si5351.set_correction(SI5351_CARIB_DATA, SI5351_PLL_INPUT_XO);
        set_tx_freq();
    }

    Serial.print("MODE:");
    if (digitalRead(PIN_PRBS_SW) == LOW) {              // PN9送信
        FrequencyTimer2::setPeriod(PERIOD_HALF_US);
        FrequencyTimer2::setOnOverflow(set_prbs);
        Serial.println("PRBS");
        while (true) {
            ;
        } // タイマ割り込みで リセットするまでPRBSを送信し続ける
    } else if (digitalRead(PIN_CWTX_SW) == LOW) {      // 無変調送信
        Serial.println("CW");
        digitalWrite(PIN_DATA, HIGH);
        digitalWrite(PIN_CLK, HIGH);
        while (true) {
            ;
        } // リセットするまで無変調送信
    } else {                                            // UART 送信
        mySerial.begin(DATA_RATE_BPS);
        Serial.println("UART");
        si5351.output_enable(SI5351_CLK0, 0);           // 非送信時の信号漏れを回避
    }
}


#define TRAINING_TIMES 10                               // トレーニング信号(0b0101010101, 10bit)の送信回数
#define SYNC_WORD 0b0111011001010000                    // 同期ワードは D-STAR と同じにしてみた。

/* メインループ */
/* UARTで受信したものをSYN115から送信する　*/
void loop() {
    if (Serial.available()) {
        rcv_bytes = 0;
        digitalWrite(PIN_TX_LED, HIGH);
        si5351.output_enable(SI5351_CLK0, 1);       // Si5351A の出力を有効化
        
        delay(100);                                 
        store_serial_data();                        // 送信データが欠けるので送信データはバッファしておく

        for (uint8_t i = 0; i < TRAINING_TIMES; i++) {
            mySerial.write(0x55);                   // トレーニング信号を送出して受信側にレベルを知らせる
        }
        send_raw_word(SYNC_WORD, 16);               // 同期ワードを送信
        send_raw_word(0xFFFF, 16);                  // 開始ワードを送信
        for (uint8_t i = 0; i < rcv_bytes; i++) {   
            mySerial.write(send_buff[i]);           // データを送信（CRC等はアプリケーション側で付加）
        }
        send_raw_word(0xFFFE, 16);                  // 終了(ブレーク信号)を送信
        PORTB = PORTB & ~_BV(1);                    // Lにして終了
        rcv_bytes = 0;                              // 後始末
        si5351.output_enable(SI5351_CLK0, 0);       // 非送信時の信号漏れを回避
        digitalWrite(PIN_TX_LED, LOW);
    }
}
