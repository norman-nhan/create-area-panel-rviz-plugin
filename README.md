# Create Area Panel - Rviz Plugin
A panel for creating area by choosing point via Publish Point tool in Rviz and save it to `.yaml` file
## ビルド
```bash
cd <ros_ws>/src # <ros_ws> is where your current workingspace with ROS is 
git clone https://github.com/norman-nhan/create-area-panel-rviz-plugin.git
rosdep install -iry --from-path ./
catkin build my_rviz_plugins
source ~/.bashrc
```
## launchファイルを実行してみる
```bash
roslaunch my_rviz_plugins map.launch
```
## パネルの使い方
- Rvizの`Publish Point`を使って記録したいポイントを指定する。
- `Save`ボタンはファイルを保存する。
- `Delete All`ボタンは指定したポイントを全部削除する。
### その他：
- エリアを保存するファイルが存在しない場合、ファイルが新しく作成される。
- ファイルが既に存在している場合、上書きせずにエリアが追加される。そのため、要らないエリアを手動で削除する必要がある。
