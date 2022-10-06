# RSIM_KMT

## 準備
### PLYデータの準備
~~~
cd model
./stl2ply.py
~~~
STLから、丸胴と立板の.plyが作成される。

### バーチャルカメラ

~~~
git clone https://github.com/lucasw/rviz_camera_stream.git
~~~

### 照明

~~~
git clone https://github.com/mogumbo/rviz_lighting
~~~

### Rviz設定  
ファイルmain.rvizにカメラ、照明の設定が追加されているので、viewer.launchを起動するだけで バーチャルカメラが有効になります。設定の変更はDisplayパネルにて行います。  
参考にしたURL  
- https://roboticsknowledgebase.com/wiki/tools/stream-rviz/


world
## 起動
~~~
roslaunch rsim_ktm start.launch
~~~
または起動アイコンで起動


## TF
|TF名|親TF|用途|
|:----|:----|:----|
|base|world|ロボットの基準座標系|
|marud|world|丸胴の設置位置の座標系|
|tool0_controller|base|ロボットの機械エンド(フランジ)|
|uf0|base|ユーザ座標系0はbaseと同じにします|
|uf1|base|ユーザ座標系1は、丸胴の設置位置に合わせています|
|uf2|base||
|uf9|base|VTで使う|

## プログラム
|名前|用途|ドキュメント|
|:----|:----|:----|
|vscene.py|3Dシーンを作成する||
|vrobo.py|カメラを設定された軌道に従って移動します||
|vcam.py|点群カメラをシミュレートします||
|positioner.py|回転対称形の位相合わせを行う。2D,3D双方の処理を行う||
|????????|軸拘束ICP||
|autoseq.py|VTのシーケンスを実行する|[Link](autoseq.md)|
