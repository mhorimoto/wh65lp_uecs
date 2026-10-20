#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WH65LP (misol) 気象センサー RS485/UART データパーサー

パケット仕様（マニュアル「WH65LP communication protocol」より）
--------------------------------------------------------------
- UART: 9600bps, 8bit, パリティなし, ストップビット1 (8N1)
- 送信間隔: 約16秒ごとに1パケット
- パケット長: 21バイト固定

バイトレイアウト（0-indexed）
--------------------------------------------------------------
 idx  0        : ヘッダー (固定 0x24)
 idx  1        : チェック/セキュリティコード
 idx  2- 4     : 風向(12bit) + 気温(12bit) をニブル単位でパック (3バイト)
 idx  5        : 湿度 (1バイト, そのまま%)
 idx  6        : 瞬間風速 raw (1バイト)
 idx  7        : 突風(ガスト)風速 raw (1バイト)
 idx  8- 9     : 積算降水量 raw (2バイト)
 idx 10-11     : UV raw (2バイト)
 idx 12-14     : 照度 raw (3バイト)
 idx 15        : CRC8 (idx0-14の15バイトに対する値, 多項式0x31)
 idx 16        : チェックサム1 (idx0-15の16バイトの合計を256で割った余り)
 idx 17-19     : 気圧 raw (3バイト)
 idx 20        : チェックサム2 (idx17-19の合計を256で割った余り)

以下の係数はサンプルデータ
  24 66 65 E2 4D 37 0D 03 00 16 00 00 00 5F 42 31 4D 01 8F 6A FA
で数値が一致することを確認済み:
  風速係数 : 0.51 (m/s / count)
  降水係数 : 0.254 (mm / count)
  気温     : (raw-400)/10 [℃]
  照度     : raw/10 [Lux]
  気圧     : raw/100 [hPa]

--------------------------------------------------------------
UECS CCM ブロードキャスト機能について
--------------------------------------------------------------
本スクリプトは、パース結果をUECS通信実用規約(version 1.00-E10)に則った
CCM電文(XML)として16520/udpにブロードキャスト送信するデーモンモードを
持っています。同一ホストでrecvdata.py(uecsgw)が動作していても、
recvdata.pyは受信専用でbindしているだけなので送信(sendto)とは競合せず、
かつ`uecsrelay`が有効であればこの電文もそのまま外部サーバへ中継されます。

UECSの規約上、<UECS>タグ内に書ける<DATA>タグは1つだけなので、WH65LPの
1回の観測値(気温・湿度・風速など)は項目ごとに別々のUDPパケットとして
送信します(CCM_DEFINITIONS参照)。

送信レベルはA-10S-0(基本送信周期10秒、値変化時の追加送信なし)に合わせ、
WH65LPの実測更新間隔(約16秒)より短い周期で「最新の保持値」を送り続けます。
一定時間(StaleAfter)以上新しい有効パケットが来ない場合は、センサ異常/
断線とみなして送信を止めます(古い値を生きたデータとして流し続けない)。

実行方法:
    python3 wh65lp_parser.py
                        → サンプルデータで動作確認
    python3 wh65lp_parser.py /dev/ttyUSB0
                        → 実機から読み込み、標準出力に表示するだけ
    python3 wh65lp_parser.py /dev/ttyUSB0 --broadcast
                        → 実機から読み込み、UECS CCMとして
                          16520/udpに送信し続ける(systemdサービス向け)

--------------------------------------------------------------
本日雨量(daily_rain_mm)の計算について【2026-10-20 バグ修正】
--------------------------------------------------------------
WH65LPが送ってくる積算降水量(rain_raw)は電源投入/センサーリセット以降
ずっと加算され続ける値なので、「本日何mm降ったか」を得るために、日付が
変わった瞬間のrain_rawを基準値として記録し、以降は「現在値-基準値」を
本日雨量として計算している(calculate_daily_rain関数)。

このロジックには重大なバグがあった: calculate_daily_rain()がCRC8や
チェックサムの検証(is_valid判定)より前の、parse_wh65lp()の中で無条件に
呼ばれていたため、たまたま日付が変わった直後に届いた1パケットが通信
ノイズ等でCRC不一致(無効)だった場合、その化けたrain_rawの値で「本日の
基準値」が確定してしまっていた。無効パケット自体はcache更新・送信は
されないが、基準値を記録するグローバル状態(_base_rain_raw, _base_rain_day)
は既に書き換わってしまっており、以降その日1日分の本日雨量計算が
「化けた基準値」を引きずったまま狂ってしまう(=次に来た正常パケットで
いきなり大きな差分が出てしまう)という不具合が実際に発生した。

対策として、calculate_daily_rain()の呼び出しをparse_wh65lp()から削除し、
is_validが確認できた後(_serial_reader_thread内)でのみ呼ぶように変更した。
これにより、本日の基準値は必ずCRC8・チェックサムが正しい、信頼できる
パケットのrain_rawからのみ記録されるようになる。
"""
VERSION = "2.32"
import os
import json
import configparser
import socket
import syslog
import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence
from datetime import datetime

PACKET_LENGTH = 21
WIND_SPEED_COEFF = 0.51   # m/s per count
RAIN_COEFF = 0.254        # mm per count

# チェックコード(idx1)について
# 実機検証の結果、check_codeは全機種共通の固定値ではなく、個体ごとに
# 割り当てられたID(マニュアルの言う"security code")らしいことが分かった。
# ある実機ではサンプルの0x66ではなく0x80が正常値だった。
# 異常受信時は0x5A, 0x64など化け方次第で任意の値になり得るため、
# 「既知の異常値をブラックリストする」方式では対応しきれない。
# → is_valid の判定は check_code を使わず、CRC8・チェックサム1・
#   チェックサム2 の3つ(all_checks_ok)のみで行う方針に変更。
# check_codeは参考情報として保持するのみ。


class WH65LPParseError(ValueError):
    """WH65LPパケットの解析エラー"""


@dataclass
class WH65LPReading:
    header: int
    check_code: int

    wind_direction_deg: int          # 0-359 (度)
    wind_direction_16: int           # 0-15 (16方位コード: 0=N, 1=NNE... 15=NNW)
    wind_direction_raw: int          # 補正前生角度 (0-359)
    temperature_c: float
    humidity_pct: int

    wind_speed_ms: float
    gust_speed_ms: float
    rain_mm: float
    rain_raw: int             # 積算降水量の生のtipカウント(基準値計算に使う)
    daily_rain_mm: float      # 本日雨量。is_validなパケットのみ呼び出し側で設定する
                               # (parse_wh65lp()内では0.0のプレースホルダのまま)
    uv_raw: int
    light_lux: float
    pressure_hpa: float

    crc8: int
    crc8_calculated: int
    crc8_ok: bool

    checksum1: int
    checksum1_calculated: int
    checksum1_ok: bool

    checksum2: int
    checksum2_calculated: int
    checksum2_ok: bool

    raw_hex: str

    @property
    def all_checks_ok(self) -> bool:
        """CRC8・チェックサム1・チェックサム2がすべて一致しているか"""
        return self.crc8_ok and self.checksum1_ok and self.checksum2_ok

    @property
    def is_valid(self) -> bool:
        """このパケットの計測値が信頼できるか。

        check_codeは個体固有IDらしく異常値を網羅的に列挙できないため判定に
        使わず、CRC8・チェックサム1・チェックサム2(all_checks_ok)のみで判定する。
        """
        return self.all_checks_ok


def _crc8(data: Sequence[int], poly: int = 0x31, init: int = 0x00) -> int:
    """
    CRC8計算 (多項式 0x31, MSBファースト, 非反転)

    マニュアルには「crc8, Polynomial_hex: 31, data reverse: MSB first」としか
    記載がなく、初期値(init)は明記されていません。実データで crc8_ok が常に
    False になる場合は、init を 0xFF 等に変えて試してください。
    """
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def _checksum8(data: Sequence[int]) -> int:
    """単純な加算チェックサム (合計を256で割った余り)"""
    return sum(data) & 0xFF

# 日雨量計算用の基準値保持（モジュールレベル変数）
# 【重要】この状態を更新してよいのは、CRC8・チェックサムが正しい(is_valid=True)
# パケットから得たrain_rawに対してのみ。無効パケットのデータでこの状態を
# 汚染すると、その日1日分の本日雨量が丸ごと狂う(2026-10-20に実機で発生した
# 不具合、詳細はモジュールdocstring参照)。そのため呼び出しは必ず
# _serial_reader_thread内のis_validチェック後からのみ行うこと。
_base_rain_raw = None
_base_rain_day = None

# 基準値の永続化(サービス再起動で本日雨量が0に戻るのを防ぐ)。
# 保存内容: {"day": "YYYY-MM-DD", "base_raw": int}
# 起動後の最初の calculate_daily_rain() 呼び出し時に1回だけ読み戻し、
# 保存された日付が今日と一致する場合のみ基準値として採用する。
BASE_RAIN_STATE_FILE = "/var/lib/wh65lp/daily_rain.json"
_base_rain_loaded = False


def _save_base_rain() -> None:
    """基準値を一時ファイル経由で原子的に保存する(失敗しても動作は継続)。"""
    try:
        os.makedirs(os.path.dirname(BASE_RAIN_STATE_FILE), exist_ok=True)
        tmp = BASE_RAIN_STATE_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"day": _base_rain_day.isoformat(), "base_raw": _base_rain_raw}, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, BASE_RAIN_STATE_FILE)
    except Exception as e:
        syslog.syslog(syslog.LOG_WARNING, f"WH65LP: 基準値の保存に失敗: {e}")


def _load_base_rain() -> None:
    """保存済みの基準値が今日のものなら復元する。"""
    global _base_rain_raw, _base_rain_day
    try:
        with open(BASE_RAIN_STATE_FILE) as f:
            d = json.load(f)
        day = datetime.strptime(d["day"], "%Y-%m-%d").date()
        base = int(d["base_raw"])
        if day == datetime.now().date():
            _base_rain_raw, _base_rain_day = base, day
            syslog.syslog(syslog.LOG_INFO, f"WH65LP: 本日の基準値を復元しました: base_raw={base} day={day}")
    except FileNotFoundError:
        pass
    except Exception as e:
        syslog.syslog(syslog.LOG_WARNING, f"WH65LP: 基準値の読み込みに失敗(無視して続行): {e}")

def calculate_daily_rain(current_raw: int) -> float:
    """基準値引き算方式による本日雨量計算。

    呼び出し元は、is_valid(CRC8・チェックサム正常)なパケットのrain_rawの
    みをここに渡すこと。無効なパケットの値を渡すと、本日の基準値が
    不正な値で確定してしまい、以降その日のdaily_rain_mmが丸ごと狂う。
    """
    global _base_rain_raw, _base_rain_day, _base_rain_loaded
    if not _base_rain_loaded:
        _base_rain_loaded = True
        _load_base_rain()
    today = datetime.now().date()

    # 日付が変わった、または初回起動時に本日の基準値（ベースライン）を記録
    if _base_rain_day != today or _base_rain_raw is None:
        if current_raw > 0:
            _base_rain_raw = current_raw
            _base_rain_day = today
            _save_base_rain()
            return 0.0
        else:
            return 0.0

    # 本日の基準値からの増分を計算
    if current_raw >= _base_rain_raw:
        delta_tips = current_raw - _base_rain_raw
    else:
        # 16bit自然オーバーフロー (65535 → 0) を日中に跨いだ場合
        if _base_rain_raw > 60000 and current_raw < 5000:
            delta_tips = (65536 - _base_rain_raw) + current_raw
        else:
            # 電池交換や本体リセットで0に戻った場合は、現在の値を新しい基準値とする
            _base_rain_raw = current_raw
            _save_base_rain()
            delta_tips = 0

    return round(delta_tips * RAIN_COEFF, 1)


def sd_notify(state: str) -> bool:
    """
    systemd に通知を送る汎用関数（外部ライブラリ不要）
    例:
        sd_notify("READY=1")                  # 起動完了を通知
        sd_notify("STATUS=Version 2.10 稼働中") # systemctl status の文字列を更新
        sd_notify("WATCHDOG=1")               # ウォッチドッグの生存通知
    """
    sock_path = os.environ.get("NOTIFY_SOCKET")
    if not sock_path:
        return False  # systemd配下で動いていない時はスキップ

    if sock_path.startswith("@"):
        sock_path = "\0" + sock_path[1:]

    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.sendto(state.encode("utf-8"), sock_path)
            return True
    except Exception:
        return False


def parse_wh65lp(raw: bytes) -> WH65LPReading:
    """
    WH65LPの21バイトパケットを解析して WH65LPReading を返す。

    【重要】本関数はCRC8・チェックサムの検証結果(is_valid)に関わらず、
    フォーマットさえ合っていれば(長さ21バイト・ヘッダー0x24)パース処理を
    行う。そのため、本関数の中では「本日の基準値」のような、呼び出し
    回数や結果が後々まで影響する副作用のある処理(calculate_daily_rain等)
    を絶対に行わないこと。無効なパケットでそれらの状態が汚染されるのを
    防ぐため、daily_rain_mmは0.0のプレースホルダのまま返し、実際の計算は
    呼び出し側がis_validを確認した後に行う。

    Parameters
    ----------
    raw : bytes
        21バイトの生データ（ヘッダー0x24から末尾チェックサムまで）

    Raises
    ------
    WH65LPParseError
        長さが21バイトでない、またはヘッダーが0x24でない場合
    """
    if len(raw) != PACKET_LENGTH:
        raise WH65LPParseError(
            f"WH65LPパケットは{PACKET_LENGTH}バイトである必要があります"
            f"(受信: {len(raw)}バイト): {raw.hex(' ').upper()}"
        )

    if raw[0] != 0x24:
        raise WH65LPParseError(
            f"ヘッダーが不正です(期待値:0x24, 受信:0x{raw[0]:02X})。"
            f"バイト列がズレている可能性があります。"
        )

    header = raw[0]
    check_code = raw[1]

    # --- 風向 (9bit: 0〜359度) ---
    # Byte 2 (下位8bit) と Byte 3 の最上位bit (Bit 7: 9bit目) を結合
    raw_dir = raw[2] | ((raw[3] & 0x80) << 1)

    # 実測オフセット補正（真北に対する物理的な取り付けズレを補正）
    # ※生の角度のままで良い場合は 0 にしてください
    OFFSET_DEG = 0
    wind_direction_deg = (raw_dir - OFFSET_DEG + 360) % 360

    # UECS標準の16方位コード (0=N, 1=NNE, ..., 15=NNW)
    wind_direction_16 = int((wind_direction_deg + 11.25) / 22.5) % 16
    wind_direction_raw = raw_dir

    # --- 気温 (11bit: -40.0〜60.0 ℃) ---
    # Byte 3の下位3bit (Bit 0〜2) が上位3bit、Byte 4 が下位8bit
    # ※Bit 3はローバッテリーフラグのためマスク(& 0x07)
    temp_raw = ((raw[3] & 0x07) << 8) | raw[4]
    temperature_c = (temp_raw - 400) / 10.0

    # --- 湿度 ---
    humidity_pct = raw[5]

    # --- 風速 / 突風 ---
    wind_speed_raw = raw[6]
    gust_raw = raw[7]
    # マニュアルの計算例では 瞬間風速=raw/8*0.51、突風=raw*0.51 と
    # 係数の掛け方が異なる（原資料どおり）
    wind_speed_ms = round((wind_speed_raw / 8) * WIND_SPEED_COEFF, 2)
    gust_speed_ms = round(gust_raw * WIND_SPEED_COEFF, 2)

    # --- 積算降水量 ---
    rain_raw = (raw[8] << 8) | raw[9]
    rain_mm = round(rain_raw * RAIN_COEFF, 2)
    # daily_rain_mmはここでは計算しない(上記docstring参照)。
    # is_validな場合のみ、呼び出し側(_serial_reader_thread)が
    # calculate_daily_rain(reading.rain_raw)で計算して上書きする。
    daily_rain_mm = 0.0

    # --- UV ---
    uv_raw = (raw[10] << 8) | raw[11]

    # --- 照度 ---
    light_raw = (raw[12] << 16) | (raw[13] << 8) | raw[14]
    light_lux = round(light_raw / 10.0, 1)

    # --- CRC8 (idx0-14 の15バイトに対する値、idx15に格納) ---
    crc8_calculated = _crc8(raw[0:15])
    crc8 = raw[15]

    # --- チェックサム1 (idx0-15 の16バイトの合計、idx16に格納) ---
    checksum1_calculated = _checksum8(raw[0:16])
    checksum1 = raw[16]

    # --- 気圧 (idx17-19) ---
    pressure_raw = (raw[17] << 16) | (raw[18] << 8) | raw[19]
    pressure_hpa = round(pressure_raw / 100.0, 2)

    # --- チェックサム2 (idx17-19 の合計、idx20に格納) ---
    checksum2_calculated = _checksum8(raw[17:20])
    checksum2 = raw[20]

    return WH65LPReading(
        header=header,
        check_code=check_code,
        wind_direction_deg=wind_direction_deg,
        wind_direction_16=wind_direction_16,
        wind_direction_raw=wind_direction_raw,
        temperature_c=round(temperature_c, 1),
        humidity_pct=humidity_pct,
        wind_speed_ms=wind_speed_ms,
        gust_speed_ms=gust_speed_ms,
        rain_mm=rain_mm,
        rain_raw=rain_raw,
        daily_rain_mm=daily_rain_mm,
        uv_raw=uv_raw,
        light_lux=light_lux,
        pressure_hpa=pressure_hpa,
        crc8=crc8,
        crc8_calculated=crc8_calculated,
        crc8_ok=(crc8 == crc8_calculated),
        checksum1=checksum1,
        checksum1_calculated=checksum1_calculated,
        checksum1_ok=(checksum1 == checksum1_calculated),
        checksum2=checksum2,
        checksum2_calculated=checksum2_calculated,
        checksum2_ok=(checksum2 == checksum2_calculated),
        raw_hex=raw.hex(" ").upper(),
    )


def calibrate_wind_direction(raw_value: int, known_degrees: float) -> None:
    """
    実機の風向raw値(wind_direction_raw)と、その時の実際の方位(度)を
    突き合わせて記録するための補助関数。

    複数の (raw_value, known_degrees) の組を集めて自分で変換式を
    推定してください（例: 線形か、モジュロ演算が絡むか等）。
    """
    print(f"raw=0x{raw_value:03X} ({raw_value:4d})  実測方位={known_degrees}°")


# ----------------------------------------------------------------------
# UECS CCM ブロードキャスト機能
# ----------------------------------------------------------------------

# uecsgw(recvdata.py)が読み込むconfig.iniと同じファイルを共有する
DEFAULT_CONFIG_PATH = "/usr/local/etc/uecsgw/config.ini"

# UECS標準規約(1.00-E10) 表5-6「予約CCM識別子の項目名」および
# 第Ⅳ章の命名規約(独自拡張は"項目名.ノード種別名", OC=室外気象測定装置)に
# 基づくCCM識別子と、WH65LPReadingからの値の取り出し方の対応表。
#
# 予約CCM識別子(単位・精度が規約で固定されているもの):
#   WAirTemp    : 屋外気温    [C]      小数点下1桁
#   WAirHumid   : 屋外湿度    [%]      整数
#   WWindSpeed  : 屋外風速    [m/s]    整数
#   WRainfall   : 屋外降雨    [mm]     整数 (本日雨量=daily_rain_mmを使用)
# 独自拡張識別子(UECS未定義の項目、またはWWindDir16の校正が未確定なため):
#   WWindDir.mOC     : 風向 度数(0-359)
#   WWindDir16       : UECS標準16方位コード(0-15)
#   WWindGust.mOC    : 突風風速[m/s]
#   UV.mOC           : UV 生値
#   Illuminance.mOC  : 照度[Lux]
#   AirPressure.mOC  : 気圧[hPa]
CCM_DEFINITIONS = (
    ("WAirTemp", lambda r: r.temperature_c),
    ("WAirHumid", lambda r: r.humidity_pct),
    ("WWindSpeed", lambda r: round(r.wind_speed_ms)),
    ("WRainfall", lambda r: round(r.daily_rain_mm)),
    ("WWindDir.mOC", lambda r: r.wind_direction_deg),   # 360度数 (0〜359)
    ("WWindDir16", lambda r: r.wind_direction_16),      # UECS標準 16方位コード (0〜15)
    ("WWindGust.mOC", lambda r: r.gust_speed_ms),
    ("UV.mOC", lambda r: r.uv_raw),
    ("Illuminance.mOC", lambda r: r.light_lux),
    ("AirPressure.mOC", lambda r: r.pressure_hpa),
)


class LatestReading:
    """
    直近の有効なWH65LPReadingを保持するスレッドセーフなキャッシュ。

    WH65LPの実測更新間隔(約16秒)よりも短い周期(例: 10秒)でUECS CCMを
    送信し続けるため、シリアル受信スレッドが書き込み、送信スレッド(または
    メインループ)が読み出す形で使う。stale_after秒以上更新がない場合は
    「センサが死んでいる」と判断し、get_if_fresh()はNoneを返す。
    """

    def __init__(self, stale_after: float = 90.0):
        self._lock = threading.Lock()
        self._reading: Optional["WH65LPReading"] = None
        self._received_at = 0.0
        self.stale_after = stale_after

    def update(self, reading: "WH65LPReading") -> None:
        with self._lock:
            self._reading = reading
            self._received_at = time.monotonic()

    def get_if_fresh(self) -> Optional["WH65LPReading"]:
        with self._lock:
            if self._reading is None:
                return None
            if time.monotonic() - self._received_at > self.stale_after:
                return None
            return self._reading


def load_gateway_config(path: str = DEFAULT_CONFIG_PATH) -> configparser.ConfigParser:
    """uecsgw(recvdata.py)と共通のconfig.iniを読み込む。"""
    cfg = configparser.ConfigParser()
    read_files = cfg.read(path)
    if not read_files:
        raise FileNotFoundError(f"config.iniが見つかりません: {path}")
    return cfg


def get_own_ip(probe_host: str, probe_port: int = 16520) -> str:
    """
    <IP>タグに載せる自ホストのIPアドレスを調べる。

    UDPソケットをprobe_host:probe_portに"connect"すると、実際に
    パケットを送出せずにOSがそのルーティングで使うであろう出力インター
    フェースのIPアドレスをgetsockname()で取得できる(よく使われる手法)。

    【重要】probe_hostには、実際にCCMを送信する先(=ブロードキャスト
    アドレスそのもの、例: 192.168.179.255)を渡すこと。VPN等で複数の
    ネットワークインターフェースを持つホスト(例: LAN用のend0とUECS
    リレー用のVPN)では、ルーティング判断はあくまで「どこに向けて
    connectしたか」で決まる。以前はconfig.iniの[uecsrelay] host(VPN側の
    アドレス)を渡していたため、実際のブロードキャスト送信に使われる
    インターフェース(LAN側)とは異なるインターフェース(VPN側)のIPが
    <IP>タグに入ってしまう不具合があった。ブロードキャスト宛先を
    probe_hostにすることで、実送信と同じルーティング判断となり一致する。
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            probe.connect((probe_host, probe_port))
            return probe.getsockname()[0]
    except OSError:
        return "192.168.179.152"


def build_ccm_xml(ccm_type: str, value, room: str, region: str, order: str,
                   priority: int, my_ip: str) -> bytes:
    """UECS CCM(データ送信)のXML電文を1件分組み立てる(ASCIIバイト列で返す)。"""
    xml = (
        '<?xml version="1.0"?>'
        '<UECS ver="1.00-E10">'
        f'<DATA type="{ccm_type}" room="{room}" region="{region}" '
        f'order="{order}" priority="{priority}">{value}</DATA>'
        f'<IP>{my_ip}</IP>'
        '</UECS>'
    )
    return xml.encode("ascii")


def broadcast_reading(sock: socket.socket, reading: "WH65LPReading",
                       bcast_addr: str, port: int,
                       room: str, region: str, order: str,
                       my_ip: str, priority: int = 15,
                       inter_packet_delay: float = 0.05) -> None:
    """WH65LPReading 1件分を、項目ごとに複数のCCM電文としてブロードキャスト送信する。

    項目ごとにtry/exceptで囲み、1項目の値取得や送信に失敗しても残りの項目の
    送信を続ける(1個の異常で周期全体が欠落するのを防ぐ)。戻り値は実際に
    送信できた件数(呼び出し元でのログ・欠落検知に使う)。
    """
    sent_count = 0
    for ccm_type, extractor in CCM_DEFINITIONS:
        try:
            value = extractor(reading)
            xml_bytes = build_ccm_xml(ccm_type, value, room, region, order, priority, my_ip)
            sock.sendto(xml_bytes, (bcast_addr, port))
            sent_count += 1
        except Exception as e:
            syslog.syslog(syslog.LOG_ERR, f"WH65LP UECS send error ({ccm_type}): {e}")
        if inter_packet_delay:
            time.sleep(inter_packet_delay)
    return sent_count


def _serial_reader_thread(port: str, cache: LatestReading, baudrate: int = 9600) -> None:
    """
    シリアルポートを読み続け、有効なパケットが来るたびcacheを更新するスレッド関数。

    シリアル通信自体が例外を出した場合(ケーブル抜けなど)は5秒待って
    read_from_serial()自体を呼び直す(ポートの再オープンからやり直す)。

    【重要】calculate_daily_rain()は、ここでis_validを確認した後にのみ
    呼び出すこと。parse_wh65lp()の中では呼ばない(詳細はモジュール
    docstring、および calculate_daily_rain() 自体のdocstring参照)。
    """
    while True:
        try:
            for reading in read_from_serial(port, baudrate=baudrate):
                if reading.is_valid:
                    # 本日雨量は、CRC8・チェックサムが正しいと確認できた
                    # このパケットのrain_rawからのみ計算する。
                    reading.daily_rain_mm = calculate_daily_rain(reading.rain_raw)
                    cache.update(reading)
                else:
                    syslog.syslog(
                        syslog.LOG_WARNING,
                        f"WH65LP: 無効パケットを受信(チェック不一致): {reading.raw_hex}",
                    )
        except Exception as e:
            syslog.syslog(syslog.LOG_ERR, f"WH65LP: シリアル読み込みエラー: {e}. 5秒後に再試行します")
            time.sleep(5)


def run_uecs_broadcast_daemon(serial_port: Optional[str], config_path: str = DEFAULT_CONFIG_PATH,
                               baudrate: int = 9600) -> None:
    """
    シリアル受信スレッドを起動し、config.iniの設定に従って一定周期でUECS CCMを
    ブロードキャスト送信し続けるデーモン本体。Ctrl+C(KeyboardInterrupt)以外では
    終了しない前提(systemdでのサービス運用を想定)。

    serial_port : コマンドライン引数で指定されたシリアルポート。
        - 指定されていれば(CLI優先)そちらを使う。
        - Noneの場合はconfig.iniの[wh65lp] SerialPortを使う。
        - CLI引数とconfig.iniの両方が指定されていて値が食い違う場合は、
          サイレントに片方を無視せず、警告を出したうえでCLI引数を優先する
          (「両方書いてあるのに違う」状態に気づけるようにするため)。
    """
    cfg = load_gateway_config(config_path)

    uecs_sec = cfg["uecs"]
    room = uecs_sec.get("Room", "0")
    region = uecs_sec.get("Region", "0")
    order = uecs_sec.get("Order", "0")
    port = int(uecs_sec.get("Port", "16520"))

    wh65_sec = cfg["wh65lp"] if cfg.has_section("wh65lp") else {}
    bcast_addr = wh65_sec.get("BroadcastAddr", "255.255.255.255")
    send_interval = float(wh65_sec.get("SendInterval", "10"))
    # WH65LPの実測周期(約16秒)より十分長く、UECS A-10S-0の受信有効期間(30秒)より
    # 長い値をデフォルトにしておく(=多少の取りこぼしでは送信を止めない)
    stale_after = float(wh65_sec.get("StaleAfter", "90"))

    # --- シリアルポートの決定: CLI引数 > config.ini、食い違いは警告 ---
    config_serial_port = wh65_sec.get("SerialPort") if wh65_sec else None
    if serial_port and config_serial_port and serial_port != config_serial_port:
        msg = (
            f"シリアルポートの指定がコマンド引数({serial_port})とconfig.ini"
            f"({config_serial_port})で食い違っています。コマンド引数を優先します。"
            "意図しない場合はどちらかを修正してください。"
        )
        print(f"[WARN] {msg}")
        syslog.syslog(syslog.LOG_WARNING, f"WH65LP: {msg}")
    elif serial_port is None:
        serial_port = config_serial_port or "/dev/ttyUSB0"

    # <IP>タグに載せる自ホストIPは、実際にCCMを送信する宛先(ブロードキャスト
    # アドレスそのもの)への経路から求める。config.iniの[uecsrelay] host(VPN等の
    # 別インターフェース宛先)を使うと、そちらのインターフェースのIPを誤って
    # 拾ってしまうことがあるため使わない。
    my_ip = get_own_ip(bcast_addr, port)

    syslog.syslog(
        syslog.LOG_INFO,
        f"WH65LP UECS broadcaster START: serial={serial_port} "
        f"bcast={bcast_addr}:{port} room={room} region={region} order={order} "
        f"myip={my_ip} interval={send_interval}s stale_after={stale_after}s",
    )
    print(f"UECS CCMブロードキャスト開始: {bcast_addr}:{port} (自IP={my_ip}, "
          f"room={room} region={region} order={order}, {send_interval}秒毎)")

    cache = LatestReading(stale_after=stale_after)

    reader = threading.Thread(
        target=_serial_reader_thread, args=(serial_port, cache, baudrate), daemon=True
    )
    reader.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sd_notify(f"STATUS=v{VERSION} running")
    sd_notify("READY=1")  # Type=notify の場合、systemctl start の待機を解除
    try:
        while True:
            reading = cache.get_if_fresh()
            if reading is not None:
                sent_count = broadcast_reading(sock, reading, bcast_addr, port, room, region, order, my_ip)
                total = len(CCM_DEFINITIONS)
                syslog.syslog(
                    syslog.LOG_INFO,
                    f"WH65LP CCM送信: {sent_count}/{total}件 "
                    f"T={reading.temperature_c} H={reading.humidity_pct} "
                    f"WS={reading.wind_speed_ms} Rain={reading.rain_mm} "
                    f"DailyRain={reading.daily_rain_mm} "
                    f"Dir(deg)={reading.wind_direction_deg} UV={reading.uv_raw} "
                    f"Lux={reading.light_lux} Press={reading.pressure_hpa}",
                )
                if sent_count < total:
                    syslog.syslog(
                        syslog.LOG_WARNING,
                        f"WH65LP: {total - sent_count}件のCCM送信に失敗しました(上のERRORログ参照)",
                    )
            else:
                syslog.syslog(
                    syslog.LOG_WARNING,
                    f"WH65LP: {stale_after}秒以上有効なデータが無いため送信をスキップします"
                    "(センサ未接続/断線の可能性)",
                )
            time.sleep(send_interval)
    finally:
        sock.close()


# ----------------------------------------------------------------------
# シリアルポートから連続受信するサンプル (pyserial が必要: pip install pyserial)
# ----------------------------------------------------------------------
def read_from_serial(port: str, baudrate: int = 9600, timeout: float = 20.0):
    """
    RS485-USBアダプタ等からWH65LPパケットを連続読み込みするジェネレータ。

    ヘッダーバイト 0x24 を探して同期を取り、21バイトずつ読み出す簡易実装。
    """
    import serial  # pyserial

    with serial.Serial(port, baudrate=baudrate, bytesize=8,
                        parity="N", stopbits=1, timeout=timeout) as ser:
        buf = bytearray()
        while True:
            chunk = ser.read(64)
            if not chunk:
                continue
            buf.extend(chunk)

            while True:
                # ヘッダー(0x24)の位置を探す
                idx = buf.find(0x24)
                if idx == -1:
                    buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < PACKET_LENGTH:
                    break

                packet = bytes(buf[:PACKET_LENGTH])
                del buf[:PACKET_LENGTH]

                try:
                    yield parse_wh65lp(packet)
                except WH65LPParseError as e:
                    print(f"[WARN] パース失敗: {e}")


def _print_reading(reading: "WH65LPReading") -> None:
    print("=== WH65LP パース結果 ===")
    print(f"生データ        : {reading.raw_hex}")
    print(f"ヘッダー        : 0x{reading.header:02X}")
    print(f"チェックコード  : 0x{reading.check_code:02X}")
    print(f"風向            : {reading.wind_direction_deg} ° (16方位={reading.wind_direction_16})")
    print(f"気温            : {reading.temperature_c} ℃")
    print(f"湿度            : {reading.humidity_pct} %")
    print(f"風速            : {reading.wind_speed_ms} m/s")
    print(f"突風            : {reading.gust_speed_ms} m/s")
    print(f"積算降水量      : {reading.rain_mm} mm")
    print(f"UV(raw)         : {reading.uv_raw}")
    print(f"照度            : {reading.light_lux} Lux")
    print(f"気圧            : {reading.pressure_hpa} hPa")
    print()
    print("--- 検証 ---")
    print(f"CRC8       : 受信=0x{reading.crc8:02X}  計算=0x{reading.crc8_calculated:02X}"
          f"  OK={reading.crc8_ok}")
    print(f"Checksum1  : 受信=0x{reading.checksum1:02X}  計算=0x{reading.checksum1_calculated:02X}"
          f"  OK={reading.checksum1_ok}")
    print(f"Checksum2  : 受信=0x{reading.checksum2:02X}  計算=0x{reading.checksum2_calculated:02X}"
          f"  OK={reading.checksum2_ok}")
    print(f"総合判定(is_valid, CRC8+Checksum1+Checksum2): {reading.is_valid}")
    if not reading.is_valid:
        print(f"⚠ 無効パケット（check_code=0x{reading.check_code:02X} も参考情報として異常値の可能性）")
    print("-" * 40)

def _print_reading_1line(reading: "WH65LPReading") -> None:
    from datetime import datetime
    # 現在日時を取得
    now = datetime.now()

    # YYYY-MM-DD HH:MM:SS 形式にフォーマット
    formatted_date = now.strftime('%Y-%m-%d %H:%M:%S')

    print(f"{formatted_date},0x{reading.header:02X},0x{reading.check_code:02X},{reading.wind_direction_raw},"
          f"{reading.temperature_c},{reading.humidity_pct},{reading.wind_speed_ms},"
          f"{reading.gust_speed_ms},{reading.rain_mm},{reading.uv_raw},{reading.light_lux},"
          f"{reading.pressure_hpa},{reading.crc8_ok},{reading.checksum1_ok},{reading.checksum2_ok},"
          f"{reading.is_valid}")


# ----------------------------------------------------------------------
# 実行方法:
#   python3 wh65lp_parser.py                          → サンプルデータで1回だけ確認
#   python3 wh65lp_parser.py /dev/ttyUSB0             → 実機から読み込み、標準出力に表示するだけ
#   python3 wh65lp_parser.py /dev/ttyUSB0 --broadcast → 実機から読み込み、UECS CCMを16520/udpへ
#                                                        送信し続ける(systemdサービス向け常駐モード)
#       --config PATH で config.ini の場所を変更できる(既定: /usr/local/etc/uecsgw/config.ini)
# ----------------------------------------------------------------------
if __name__ == "__main__":
    import argparse
    import sys

    arg_parser = argparse.ArgumentParser(
        description="WH65LP気象センサーのパケットをパースし、必要に応じてUECS CCMとしてブロードキャスト送信する"
    )
    arg_parser.add_argument(
        "port", nargs="?", default=None,
        help="シリアルポート(例: /dev/ttyUSB0)。省略時はサンプルデータで動作確認のみ行う",
    )
    arg_parser.add_argument(
        "--broadcast", action="store_true",
        help="パース結果を標準出力に表示するだけでなく、UECS CCMとして16520/udpへブロードキャスト送信し続ける",
    )
    arg_parser.add_argument(
        "--config", default=DEFAULT_CONFIG_PATH,
        help=f"uecsgwのconfig.iniのパス(既定: {DEFAULT_CONFIG_PATH})",
    )
    arg_parser.add_argument(
        "--baudrate", type=int, default=9600, help="シリアル通信速度(既定: 9600)",
    )
    args = arg_parser.parse_args()

    if args.broadcast:
        # --- 実機から読み込み、UECS CCMとして送信し続けるデーモンモード ---
        # シリアルポートはコマンド引数(args.port)を優先し、省略時は
        # config.iniの[wh65lp] SerialPortを使う(run_uecs_broadcast_daemon内で解決)。
        try:
            run_uecs_broadcast_daemon(args.port, config_path=args.config, baudrate=args.baudrate)
        except KeyboardInterrupt:
            print("\n終了します。")
        except FileNotFoundError as e:
            print(f"[ERROR] {e}")
            print("config.iniのパスを--configで指定してください。")
        except Exception as e:
            print(f"[ERROR] UECSブロードキャストデーモンの起動に失敗しました: {e}")

    elif args.port is None:
        # --- マニュアル記載のサンプルデータで動作確認 ---
        sample_hex = "24 66 65 E2 4D 37 0D 03 00 16 00 00 00 5F 42 31 4D 01 8F 6A FA"
        sample_raw = bytes.fromhex(sample_hex.replace(" ", ""))

        reading = parse_wh65lp(sample_raw)
        _print_reading(reading)
        print()
        print("期待値: 気温18.9℃ / 湿度55% / 風速0.83 / 突風1.53 / "
              "雨量5.59 / 照度2438.6 / 気圧1022.50")

    else:
        # --- 実機のシリアルポートから読み込み、標準出力に表示するだけ(送信しない) ---
        print(f"=== {args.port} からWH65LPパケットを待ち受け中 (Ctrl+Cで終了) ===")
        try:
            for reading in read_from_serial(args.port, baudrate=args.baudrate):
                #_print_reading(reading)  # is_validと個別チェックの警告は関数内で表示済み
                _print_reading_1line(reading)  # is_validと個別チェックの警告は関数内で表示済み
        except KeyboardInterrupt:
            print("\n終了します。")
        except Exception as e:
            print(f"[ERROR] シリアル読み込みに失敗しました: {e}")
            print("ポート名や権限(dialoutグループ等)、ケーブル接続を確認してください。")
