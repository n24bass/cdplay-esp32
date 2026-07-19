# cdplay-esp32

ESP32-S3にUSB接続のCD/DVD/BDドライブとI2S DACを接続し、音楽CDを再生するプロジェクトです。

USB Mass Storage Bulk-Only Transport（BOT）とSCSI/MMCコマンドをESP32-S3上で直接扱い、CD-DAの2352バイト生セクタを読み出してI2Sへ出力します。TOCは頭出しとトラック番号表示に使用し、通常再生はディスク全体を連続ストリームとして扱うため、曲間を途切れさせず再生できます。

Wi-Fi経由でfreedb日本語へ問い合わせ、アルバム名と曲名を取得してOLEDへ表示します。日本語UTF-8表示と長い曲名の横スクロールにも対応しています。

## 主な機能

- USB CD/DVD/BDドライブの検出
- SCSI INQUIRY、TEST UNIT READY、REQUEST SENSE
- READ TOCによるトラック情報取得
- READ CDによるCD-DA生セクタ読み出し
- 44.1kHz、16bit、ステレオのI2S出力
- ギャップレス連続再生
- 再生、一時停止、停止、前後トラック、5秒単位シーク
- 音量調整とミュート
- リピートOFF／全曲／1曲
- あらかじめ生成した曲順テーブルによるシャッフル再生
- 読み取り失敗セクタの無音補間
- 音声リングバッファによる振動・読み取り遅延対策
- ディスク挿入・排出の自動検出
- SH1106 128×64 OLEDのプレーヤー表示
- freedb日本語からのアルバム名・曲名取得
- 複数のCDDB候補からの手動選択
- 日本語曲名の横スクロール表示
- Wi-Fi接続状態とCDDB通信中アイコン

## 使用ハードウェア

- ESP32-S3-N8R2
- USB接続の光学ドライブ
  - 動作確認: BUFFALO BRXL-PC6VU2C
- 光学ドライブへの給電手段（パワースプリッターケーブルなど）
- I2S DAC
  - 動作確認: CJMCU-4344（CS4344）
- SH1106 128×64 I2C OLED
- プルアップされた4個のボタン

## 配線

### USB Host

| 信号 | ESP32-S3 |
|---|---:|
| USB D- | GPIO19 |
| USB D+ | GPIO20 |

光学ドライブはESP32-S3から直接給電しないこと。

### I2S DAC

| DAC | ESP32-S3 |
|---|---:|
| SDIN | GPIO4 |
| SCLK / BCLK | GPIO5 |
| LRCLK | GPIO6 |
| MCLK | GPIO7 |
| GND | GND |

### OLED

| OLED | ESP32-S3 |
|---|---:|
| SDA | GPIO8 |
| SCL | GPIO9 |
| GND | GND |

### ボタン

ボタン入力は`INPUT_PULLUP`です。ボタンを押したときにGPIOとGNDが接続されるように配線します。

| ボタン | ESP32-S3 | 用途 |
|---|---:|---|
| UP | GPIO10 | 前トラック、シーク、音量＋、設定変更 |
| DOWN | GPIO11 | 次トラック、シーク、音量－、設定変更 |
| ACTION | GPIO12 | 再生／一時停止、決定 |
| MODE | GPIO13 | 操作モード切り替え |

## 必要なソフトウェア

- Arduino IDE 2.x
- Arduino-ESP32 core
  - 3.3.10でコンパイル確認済み
- U8g2
  - 2.36.19で確認済み

`ESP_I2S`、`WiFi`、`Wire`、USB Host APIはArduino-ESP32 coreに含まれます。

## Arduino IDEのボード設定

基本設定例です。使用するESP32-S3ボードに合わせて調整してください。

- Board: `ESP32S3 Dev Module`
- Flash Size: `8MB`
- PSRAM: `OPI PSRAM`
- USB CDC On Boot: 使用する書き込み・シリアル接続方法に合わせて設定

本プロジェクトではESP32-S3-N8R2を使用しています。

## Wi-Fi設定

`secrets.example.h`を参考に、同じフォルダの`secrets.h`へ2.4GHz Wi-FiのSSIDとパスワードを設定します。

```cpp
#pragma once

static constexpr char WIFI_SSID[] = "your-2.4GHz-ssid";
static constexpr char WIFI_PASSWORD[] = "your-password";
```

`secrets.h`は`.gitignore`に登録されているため、通常はGitHubへ公開されません。コミット前に認証情報が含まれていないことを必ず確認してください。

ESP32-S3は5GHz Wi-Fiには接続できません。WEPにも対応する設定を含みますが、WEPは安全性が低いため、可能であればWPA2を使用してください。

## CDDB

TOCからfreedb形式のDisc IDを計算し、次のHTTP CDDBサーバーへ問い合わせます。

```text
freedbtest.dyndns.org:80/~cddb/cddb.cgi
```

問い合わせは音声再生やUSBホスト処理を止めないよう、別のFreeRTOSタスクで実行します。

検索候補が1件の場合は自動的に採用します。複数候補の場合も最初の候補を自動採用しますが、候補一覧を保持するため、CDDBモードから後で選び直せます。最大12候補を保持します。

CDDBは外部サービスのため、サーバー停止、ネットワーク障害、未登録ディスク、誤登録などにより情報を取得できない場合があります。曲名が取得できなくてもCD再生自体は継続します。

## 操作方法

MODEボタンを押すたびに、次の順番でモードが切り替わります。

```text
TRACK → VOLUME → OPT → CDDB → TRACK
```

### TRACKモード

| 操作 | 動作 |
|---|---|
| UP短押し | 前トラック。1曲目では1曲目の先頭へ戻る |
| DOWN短押し | 次トラック。最終曲では1曲目へ戻る |
| UP長押し | 5秒ずつ巻き戻し |
| DOWN長押し | 5秒ずつ早送り |
| ACTION短押し | 再生／一時停止 |
| ACTION長押し | 停止して1曲目へ戻る |

一時停止中もドライブの回転停止を防ぐため、次のセクタを定期的に読み捨てます。

### VOLUMEモード

| 操作 | 動作 |
|---|---|
| UP | 音量を5%上げる |
| DOWN | 音量を5%下げる |
| ACTION | ミュート切り替え |

### OPTモード

| 操作 | 動作 |
|---|---|
| UP | シャッフルON／OFF |
| DOWN | リピートOFF → 全曲 → 1曲 |
| ACTION | シャッフルとリピートを解除 |

シャッフルは曲を移動するたびに乱数を選ぶ方式ではなく、あらかじめランダムな曲順テーブルを生成し、その順番に沿って再生します。

### CDDBモード

CDDBの取得状態と候補一覧を表示します。

| 操作 | 動作 |
|---|---|
| UP / DOWN | CDDB候補を選択 |
| ACTION | 選択した候補を適用 |
| MODE | CDDBモードを終了 |

表示される主な状態:

- `IDLE`
- `WAITING WI-FI`
- `QUERYING`
- `READING ENTRY`
- `READY`
- `NO MATCH`
- `NETWORK ERROR`

## OLED表示

- トラック番号／総トラック数
- 再生、一時停止、停止アイコン
- トラック内の経過時間
- トラック進捗バー
- 音量
- シャッフル／リピート状態
- Wi-Fi接続済みアイコン
- CDDB通信中に点滅するデータベースアイコン
- CDDBから取得した日本語・英語曲名

長い曲名は横スクロールします。日本語フォント処理でUSB読み出しを妨げないよう、画面に必要な範囲だけを描画します。

## 読み取りエラーへの対応

CD-DAはリアルタイム再生を優先します。読めなかったセクタをESP32側で何度も再試行せず、そのセクタを無音で補間して次へ進みます。

音声用リングバッファは48セクタ、再生開始前のプリバッファは40セクタです。およそ0.5秒分の読み取り遅延を吸収します。

## シリアルモニター

ボーレートは`115200`です。USB列挙、SCSI応答、TOC、CDDB結果、Wi-Fi状態、読み取りエラーなどを確認できます。

起動時には周辺の2.4GHzアクセスポイントについて、SSID、RSSI、チャンネル、暗号方式を表示します。

## 注意事項

- 光学ドライブには十分な外部電源を用意してください。
- ESP32-S3とUSB機器、DAC、OLEDのGNDを共通にしてください。
- GPIO19/20はUSB Hostが使用します。
- CDDB通信は平文HTTPです。
- 市販CDの音声や取得したメタデータの利用は、各地域の法令や利用条件に従ってください。
- すべてのUSB光学ドライブでの動作を保証するものではありません。

## 現在のメモリ使用量

ESP32-S3 Dev Module、Flash 8MB、OPI PSRAM設定での参考値:

```text
Sketch: 約1.16MB（アプリ領域の約88%）
Global variables: 約55KB
```

日本語フォントがフラッシュ容量の比較的大きな部分を占めます。

## ファイル

- `cdplay-esp32.ino` — プレーヤー本体
- `secrets.example.h` — Wi-Fi設定例
- `secrets.h` — ローカルのWi-Fi設定（Git対象外）

## ライセンス

このプロジェクトは[MIT License](LICENSE)で公開します。
