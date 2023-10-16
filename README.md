# hakoniwa-px4sim

 本リポジトリは、PX4 on SITL と接続可能な箱庭シミュレータの試作を目的としています。

 将来的には、TOPPERSリポジトリに移動する予定です。

 ## 前提

以下のセットアップが終了していること。

 https://github.com/tmori/px4-build

動作環境も上記に従います。

次に、以下のURLから、`capture.bin` をダウンロードしてください。

https://github.com/tmori/hakoniwa-px4sim/releases/tag/v1.0.0

ダウンロードしたファイルは、`cmake-build` 直下に配置してください。


## 箱庭シミュレータのビルド方法

```
cd cmake-build
```

```
cmake ..
```

```
make
```

 
## 箱庭シミュレータの起動方法

現時点では、箱庭シミュレータは、jMAVSimのキャプチャパケットのリプレイのみをサポートしています。

将来的には、Unityとの連携もしていく予定です。

まず、px4-build`の PX4 on SITL を起動してください。

次に、以下のコマンドで箱庭シミュレータを起動します。

```
cd cmake-build
```

```
./src/hako-px4sim 127.0.0.1 4560 replay
```

成功すると、PX4 on SITL から受信した MAVLinkパケットのログメッセージが大量に出力されます。



