# wh65lp_uecs

気象観測複合センサ **WH65LP**（misol / Fine Offset 系 RS485/UART 気象センサ）のバイナリパケットをデコードし，**UECS（農業情報創成通信標準仕様 ver 1.00-E10）** 形式の CCM（Common Control Message）電文として UDP ブロードキャスト送信（ポート: 16520）する Python スクリプトです．

---

## 特徴

- **WH65LP バイナリパケット解析**:
  - UART/RS485 経由で受信される 21 バイト固定長バイナリフレームをパース
  - 気温・湿度・風速・突風・積算雨量・UV・照度・気圧等の各物理値への変換に対応
  - CRC8（多項式 `0x31`）および 2 つの加算チェックサムによる厳密なデータ妥当性検証
- **UECS CCM ブロードキャスト**:
  - UECS-CCM（XML）電文を生成し，`16520/udp` にブロードキャスト送信
  - 既存の `uecsgw`（`recvdata.py`）等と同一ホストで共存可能な設計
- **マルチスレッド・キャッシュアーキテクチャ**:
  - WH65LP の計測間隔（約 16 秒）と UECS 送信周期（標準 10 秒 / A-10S-0）の差異をスレッドセーフなキャッシュで吸収
  - センサ切断・通信断時には古いデータを送信し続けないタイムアウト（`stale_after`）ガード機構
- **`uecsgw` 互換設定**:
  - `/usr/local/etc/uecsgw/config.ini` と連携し，ノード識別属性（Room, Region, Order）やブロードキャスト設定を共有可能

---

## センサ・通信仕様

### WH65LP UART 通信仕様
- **ボーレート**: 9600 bps
- **データ長**: 8 bit
- **パリティ**: なし (None)
- **ストップビット**: 1 bit
- **送信周期**: 約 16 秒間隔
- **パケット長**: 21 バイト固定

### パケット構造（21バイト）
| オフセット | 長さ | 項目 | 内容・換算式 |
|---|---|---|---|
| `idx 0` | 1 byte | ヘッダー | 固定値 `0x24` |
| `idx 1` | 1 byte | セキュリティ/チェックコード | 個体固有コード（参考情報） |
| `idx 2-4` | 3 bytes | 風向 / 気温 | 12bitずつニブルパック．気温: `(raw - 400) / 10.0` [℃] |
| `idx 5` | 1 byte | 湿度 | 生値 [%] |
| `idx 6` | 1 byte | 瞬間風速 raw | `(raw / 8) * 0.51` [m/s] |
| `idx 7` | 1 byte | 突風 (ガスト) raw | `raw * 0.51` [m/s] |
| `idx 8-9` | 2 bytes | 積算降水量 raw | `raw * 0.254` [mm] |
| `idx 10-11` | 2 bytes | UV raw | 生値 |
| `idx 12-14` | 3 bytes | 照度 raw | `raw / 10.0` [Lux] |
| `idx 15` | 1 byte | CRC8 | idx 0-14 に対する CRC8（多項式 0x31） |
| `idx 16` | 1 byte | チェックサム1 | idx 0-15 の 16 バイト加算和 mod 256 |
| `idx 17-19` | 3 bytes | 気圧 raw | `raw / 100.0` [hPa] |
| `idx 20` | 1 byte | チェックサム2 | idx 17-19 の 3 バイト加算和 mod 256 |

> **注記（風向データについて）**:
> 風向（度）への変換式は仕様書の記載に不整合があるため，現状は 12bit 生値（0-4095）を `WWindDirRaw.mOC` として送信します．運用環境に合わせて方位磁石等との実機校正を行ってください．

---

## UECS CCM 送信仕様

本プログラムは 1 回の観測データに対し，項目ごとに個別の UECS CCM 電文（XML）を生成して送信します．

| CCM タイプ | 区分 | 単位 / 精度 | 送信値 | 説明 |
|---|---|---|---|---|
| `WAirTemp` | 予約 | ℃（小数第1位） | `temperature_c` | 屋外気温 |
| `WAirHumid` | 予約 | %（整数） | `humidity_pct` | 屋外湿度 |
| `WWindSpeed` | 予約 | m/s（整数） | `round(wind_speed_ms)` | 屋外風速 |
| `WRainfall` | 予約 | mm（整数） | `round(rain_mm)` | 屋外積算雨量 |
| `WWindDirRaw.mOC` | 拡張 | raw (0-4095) | `wind_direction_raw` | 風向（生値・要校正） |
| `WWindGust.mOC` | 拡張 | m/s（小数第2位） | `gust_speed_ms` | 突風風速 |
| `UV.mOC` | 拡張 | raw | `uv_raw` | UV 生値 |
| `Illuminance.mOC` | 拡張 | Lux（小数第1位） | `light_lux` | 照度 |
| `AirPressure.mOC` | 拡張 | hPa（小数第2位） | `pressure_hpa` | 大気圧 |

- **送信レベル**: A-10S-0（10秒周期，値変化時の追加送信なし）
- **優先度 (Priority)**: 1
- **送信先**: ブロードキャストアドレス（既定: `config.ini` で指定されたアドレス）: 16520/udp

---

## 必要要件・セットアップ

### 動作要件
- Linux 環境（Ubuntu, Debian, AlmaLinux 等）
- Python 3.8 以上
- USB-RS485 アダプタ または シリアル変換ボード

### 依存パッケージのインストール
```bash
pip install pyserial
```

### シリアルポート権限の付与（Linux）
シリアルポート（/dev/ttyUSB0 等）へアクセスするため，実行ユーザーを dialout グループに追加します．
```bash
sudo usermod -a -G dialout $USER
# 反映のため再ログインまたは newgrp dialout
```

### 設定ファイル（config.ini）

既定では /usr/local/etc/uecsgw/config.ini を読み込みます．--config オプションで任意のパスを指定することも可能です．

```Ini, TOML
[uecs]
Room = 1
Region = 1
Order = 1
Port = 16520

[wh65lp]
# シリアルポート（コマンド引数で未指定の場合に使用）
SerialPort = /dev/ttyUSB0
# UECS ブロードキャスト宛先
BroadcastAddr = 192.168.1.255
# 送信間隔（秒）
SendInterval = 10
# この秒数以上新しいデータが来ない場合はセンサ異常とみなして送信停止
StaleAfter = 90
```

## 使い方

### 1. サンプルデータによるパース確認

引数なしで実行すると，仕様書記載のテストパケットを用いたデコードテストを行います．

```bash
python3 wh65lp_parser.py
```

### 2. シリアルポートからのデータ読み取り（標準出力モード）

実機から連続してパケットを読み取り，受信時刻とパース結果を 1 行 CSV 形式で標準出力に表示します（UECS 送信は行いません）．

```bash
python3 wh65lp_parser.py /dev/ttyUSB0
```

### 3. UECS ブロードキャストデーモンモード

実機からデータを取得し，UECS CCM 電文として UDP ブロードキャスト送信を行います．

```bash
# デフォルト設定ファイルを使用
python3 wh65lp_parser.py /dev/ttyUSB0 --broadcast

# 独自の設定ファイルを指定
python3 wh65lp_parser.py /dev/ttyUSB0 --broadcast --config /path/to/custom_config.ini
```

### コマンドライン引数一覧

```plaintext
usage: wh65lp_parser.py [-h] [--broadcast] [--config CONFIG] [--baudrate BAUDRATE] [port]

WH65LP気象センサーのパケットをパースし、必要に応じてUECS CCMとしてブロードキャスト送信する

positional arguments:
  port                 シリアルポート(例: /dev/ttyUSB0)。省略時はサンプルデータで動作確認のみ行う

options:
  -h, --help           ヘルプメッセージを表示して終了
  --broadcast          パース結果を標準出力に表示するだけでなく、UECS CCMとして16520/udpへブロードキャスト送信し続ける
  --config CONFIG      uecsgwのconfig.iniのパス(既定: /usr/local/etc/uecsgw/config.ini)
  --baudrate BAUDRATE  シリアル通信速度(既定: 9600)
  ```

### systemd サービス登録例
バックグラウンドサービスとして常駐運用する場合の設定例です．

```/etc/systemd/system/wh65lp-uecs.service:```

```Ini, TOML
[Unit]
Description=WH65LP Weather Sensor to UECS Broadcaster
After=network.target

[Service]
Type=simple
User=root
ExecStart=/usr/bin/python3 /opt/wh65lp_uecs/wh65lp_parser.py /dev/ttyUSB0 --broadcast --config /usr/local/etc/uecsgw/config.ini
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now wh65lp-uecs.service
```


# ライセンス

MIT License の予定．
