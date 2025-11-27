# DAMIAO_Control_Arduino

## 概要

このライブラリは、CANバス通信を介してDAMIAO製BLDCモーターを制御します。モーターの制御モード切替、コマンド送信、フィードバックの受信、各種パラメータの読み書きといった機能を提供します。

複数のモーターを管理するための`MotorManager`クラスも含まれています。

動作検証はDM3519とDM2325によって行っています。

## 対応ハードウェアと依存ライブラリ

このライブラリは、arduino::HardwareCANに準拠しています。CAN通信を行うための別途ライブラリが必要です。使用するボードに合わせて、適切なCANライブラリをインストールしてください。

|マイコンボード|CANライブラリ|
|---|---|
|Arduino (Uno R4 WiFi / Uno R4 Minima / Nano R4)|Arduino_CAN (ビルトイン)|
|Raspberry Pi Pico (RP2040/RP2350)|[RP2040PIO_CAN](https://github.com/eyr1n/RP2040PIO_CAN)|
|ESP32|[ESP32_TWAI](https://github.com/eyr1n/ESP32_TWAI)|

**注意:** CANトランシーバーモジュールが別途必要です。

## インストール

<!--
### Arduino IDE ライブラリマネージャー

1.  Arduino IDE を開きます。
2.  `スケッチ > ライブラリをインクルード > ライブラリを管理...` に移動します。
3.  "DAMIAO_Control" を検索し、最新バージョンをインストールします。
-->

### 手動インストール

#### Arduino IDE の「.ZIPライブラリをインポート」を使用

1.  [GitHubリポジトリの最新リリース](https://github.com/Suzu-Gears/DAMIAO_Control_Arduino/releases/latest)を `.zip` ファイルでダウンロードします。
2.  Arduino IDE で、`スケッチ > ライブラリをインクルード > .ZIPライブラリをインポート...` に移動します。
3.  ダウンロードした `.zip` ファイルを選択します。
4.  Arduino IDE を再起動します。

#### 直接配置

1.  [GitHubリポジトリの最新リリース](https://github.com/Suzu-Gears/DAMIAO_Control_Arduino/releases/latest)を `.zip` ファイルでダウンロードします。
2.  ダウンロードしたファイルを解凍し、フォルダをArduinoのライブラリディレクトリ（例: `~/Documents/Arduino/libraries/`）に配置します。
3.  Arduino IDE を再起動します。

## 基本的な使い方

基本的な使用方法や詳細なコードについては、`examples` フォルダ内のサンプルスケッチを参照してください。
各スケッチが特定の機能（単一モーター制御、複数モーター制御、モード切替、パラメータ読み書きなど）を実演しています。

## 参考資料

DAMIAOモーターに関する公式情報や技術ドキュメントは、以下のリンクを参照してください。

- **DAMIAO Gitee リポジトリ**: [https://gitee.com/kit-miao/damiao](https://gitee.com/kit-miao/damiao)
- **DAMIAO Wiki**: [https://gl1po2nscb.feishu.cn/wiki/MZ32w0qnnizTpOkNvAZcJ9SlnXb](https://gl1po2nscb.feishu.cn/wiki/MZ32w0qnnizTpOkNvAZcJ9SlnXb)
- **MIT制御の説明 (Wiki)**: [https://gl1po2nscb.feishu.cn/wiki/Y3OEwMr4GivZU9kZqkjctmGinye](https://gl1po2nscb.feishu.cn/wiki/Y3OEwMr4GivZU9kZqkjSlnXb)
- **CAN IDの説明（Wiki）**: [https://gl1po2nscb.feishu.cn/wiki/Se1Dw464piCcERktZuOco7IPnTb](https://gl1po2nscb.feishu.cn/wiki/Se1Dw464piCcERktZuOco7IPnTb)
