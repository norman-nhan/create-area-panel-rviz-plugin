# Create Area Panel - Rviz Plugin
a panel for creating area by choosing point via Publish Point tool in Rviz and save it to .yaml file
## ビルド
```bash
cd <ros_ws>/src # <ros_ws> is where your current workingspace with ROS is 
git clone https://github.com/norman-nhan/create-area-panel-rviz-plugin.git
rosdep install -iry --from-path ./
catkin build my_rviz_plugins
source ~/.bashrc
```
## demoを実行してみる
```bash
roslaunch map.launch
```
## パネルの使い方
- `Publish Point` Toolを使って記録したいポイントを指定する。
- 指定したポイントを記録するのには`Add Point`をクリックする。
- ファイルを保存するのには`Save`をクリックする。
- `Delete All`を押すと全ての記録したポイントを削除する。
### **注意**：
- ファイルを保存する前に、`Save File Name`かつ`Area Name`を指定する必要がある。
- `Save File Name`と`Area Name`の入力後、`Enter`キーを押さないと更新されない。
### **その他**：
- ファイルが存在しない場合、ファイルを新しく作成される。ファイルが既に存在した場合、上書きせずにエリアが追加される。そのため、要らないものを手動に削除する必要がある。
