# Create Area Panel Rviz Plugin

**動作確認動画**


https://github.com/norman-nhan/my_rviz_plugins/assets/90228548/ba8f49bc-fb98-4763-8af4-c7b20164ad22




## ビルド

```bash
cd <ros_ws>/src # replace <ros_ws> with your ROS workspace 
git clone https://github.com/norman-nhan/create-area-panel-rviz-plugin.git
rosdep install -iry --from-path ./
catkin build my_rviz_plugins
source ~/.bashrc
```

## テスト

```bash
roslaunch my_rviz_plugins map.launch
```

## Pluginの使い方

`CreateAreaPanel`の上に以下の機能がある。
- `Save File Name`は保存されるファイル名を指定する。
- `Area Name`はエリア名を指定する。
- `Save`ボタンはファイルを保存する。
- `Delete All`ボタンは指定したポイントを全部削除する。

### 流れ

- ファイル名とエリア名を入力する。
- Rviz Toolの`Publish Point`を使って記録したいポイントを指定する。
- ポイントを全部指定した後、 `Save` ボタンを押すと入力したエリア名が入力したファイル名の中に追加される。
- エリアを保存した後、**新しいエリアを作成する前に** `Delete All`を押して前のポイントを全部削除すること。
