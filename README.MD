## ook_uhf_transmitter
* PC等から受け取った任意の63バイトまでのデータを SYN115 から2値ASKで送信します。
* 受信側のプログラムは [OOK UHF Receiver] です。
* 任意の送信周波数を設定できるように、クロックジェネレータ[Si5351A]を使用しています。
* 送信可能周波数は、438.100 MHz から 100kHzステップ 9波です。
* 無線区間のUARTの仕様は Bit Rate:1000bps, Data bits:8, Parity: None, Stop bit: 1 です。
* PCとの通信は Bit Rate:115200bps, Data bits:8, Parity: None, Stop bit: 1 です。

### Transmitter Block
![](https://github.com/7m4mon/ook_uhf_transceiver/blob/main/ook_uhf_transmitter_block.png)

## ook_uhf_receiver
* [OOK UHF Transmitter] から送信された2値ASK信号を SYN480Rで受信するプログラムです。
* リアルタイム処理でクロック再生を実行し、メインループでUARTステートマシンを構成しています。
* 任意の受信周波数を設定できるように、クロックジェネレータ[Si5351A]を使用しています。
* 受信可能周波数は、438.100 MHz から 100kHzステップ 9波です。
* 無線区間のUARTの仕様は Bit Rate:1000bps, Data bits:8, Parity: None, Stop bit: 1 です。
*  PCとの通信は Bit Rate:115200bps, Data bits:8, Parity: None, Stop bit: 1 です。

### Receiver Block
![](https://github.com/7m4mon/ook_uhf_transceiver/blob/main/ook_uhf_receiver_block.png)

## Photo

![](https://github.com/7m4mon/ook_uhf_transceiver/blob/main/pic_outside.jpg)

![](https://github.com/7m4mon/ook_uhf_transceiver/blob/main/pic_inside.jpg)


## Circuits
![](https://github.com/7m4mon/ook_uhf_transceiver/blob/main/ook_uhf_transceiver_circuit.png)



