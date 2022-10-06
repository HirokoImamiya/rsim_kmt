# autoseq
シュミレータ上で、対象物を配置し、VTの一連の動作であるレシピの選択、撮影、解析（マッチング）を行う。<br>
対象物ごとにモデル（model.yamlファイル）を登録し、対象物を配置する位置や選択するレシピ名などを指定する。<br>
このモデルをリスト化し、シーケンスとして登録する。<br>
セットアップ画面の「VT開始」ボタンをクリックすると、作成したシーケンスを実行する。<br>
シーケンスを連続で実行する場合は、シーケンス実行回数を指定する。

## Parameter
### ~config
|name|type|description|
|:----|:----|:----|
|sub_timeout|int|各イベントのtopicの応答タイムアウト（秒）|
|base_frame_id|str|解析結果の基準座標のフレーム名|
|source_frame_id|str|解析結果のマスタのフレーム名|
|target_frame_id|str|解析結果のシーン中のマスターに一致するフレーム名|

### ~param
|name|type|description|
|:----|:----|:----|
|seq_retry|int|シーケンス実行回数|
|solve_end_wait|int|解析後のウェイト時間（秒）|
|solve_retry|int|解析のリトライ回数|
|seq_model|list(str)|モデル名のリスト。シーケンス開始にて、リスト順に実行する|
|modelmove|bool|対象物配置時の移動有無。False:マスタ作成時の位置姿勢、True:モデルにて指定した位置姿勢|
|initpos/uf|str|シーケンス開始位置姿勢の基準座標のフレーム名|
|initpos/xyz|[x,y,z]|シーケンス開始位置姿勢　座標|
|initpos/rpy|[x,y,z]|シーケンス開始位置姿勢　回転角度[deg]|

## Topics
### To subscribe
|name|type|description|
|:----|:----|:----|
|~loaded|Bool|レシピ選択要求に対する応答|
|~rmoved|Bool|１箇所のカメラ移動要求に対する応答|
|~inpos|Bool|範囲、複数箇所のカメラ移動要求に対する応答|
|~solved|Bool|解析（マッチング）要求に対する応答|
|~cleared|Bool|初期化要求に対する応答|

### To publish
|name|type|description|remarks|
|:----|:----|:----|:----|
|~load|String|レシピ選択要求||
|~rmove|Bool|１箇所のカメラの移動要求。True:移動後に撮影|移動位置姿勢は、rosparamにて設定|
|~path_interval|Bool|範囲指定のカメラの移動要求。True:移動ごとに撮影|範囲、間隔は、rosparamにて設定|
|~path_point|Bool|複数箇所のカメラの移動要求。True:移動ごとに撮影|回転角度、Z軸位置は、rosparamにて設定|
|~solve|Bool|解析（マッチング）要求||
|~clear|Bool|初期化要求||
|~mupdate|Bool|対象物（モデル）の位置姿勢情報の更新要求||
|/report|String|レポート出力||

## recipe
シュミレータ上では、レシピにマスタ作成時の撮影位置姿勢を登録する。<br>
画面からは設定できないため、param.yamlを直接編集する。

|name|type|description|
|:----|:----|:----|
|campos|dict|マスタ作成時の撮影情報。複数の撮影情報登録可|
|campos/num|int|撮影情報数|
|campos/posX|dict|１つの撮影情報。Xは任意の文字列|
|campos/posX/pos|int|撮影番号。番号が小さい順に撮影する|
|campos/posX/uf|string|撮影時の基準座標のフレーム名。本フレームを基準に移動や回転する|
|campos/posX/type|string|撮影タイプ。move,path_interval,path_point。撮影タイプにより、撮影位置姿勢の指定方法が異なる|


### 撮影タイプ

撮影タイプは、１箇所、範囲、複数箇所指定の３タイプがあり、範囲、複数箇所指定には、Z軸の回転、移動の２タイプがある。<br>
撮影位置姿勢は、撮影時の基準座標のフレーム（campos/posX/uf）を基準に指定する。<br>
Z軸の回転や移動は、撮影時の基準座標のフレーム（campos/posX/uf）のZ軸を基準に行う。

|campos/posX/type|campos/posX/path/ip|description|
|:----|:----|:----|
|move|-|１箇所の撮影をする|
|path_interval|rot_z|特定の範囲をZ軸まわりに一定の間隔ごとに撮影する|
|path_interval|mov_z|特定の範囲をZ軸方向に一定の間隔ごとに撮影する|
|path_point|rot_z|Z軸まわりに指定した角度ごとに撮影する。角度は複数指定可|
|path_point|mov_z|Z軸方向に指定した位置ごとに撮影する。位置は複数指定可|


* **１箇所撮影（campos/posX/type=move）**<br>
「撮影位置姿勢」にて撮影する

|name|type|description|
|:----|:----|:----|
|campos/posX/xyz|[x,y,z]|撮影位置姿勢　座標|
|campos/posX/rpy|[x,y,z]|撮影位置姿勢　回転角度[deg]|

* **範囲指定　Z軸回転撮影（campos/posX/type=path_interval＆campos/posX/path/ip=rot_z）**<br>
「回転範囲」の範囲を基準座標のZ軸まわりに「回転間隔」ごとに、「撮影位置姿勢」にて撮影する

|name|type|description|
|:----|:----|:----|
|campos/posX/path/ip|string|移動タイプ（rot_z）|
|campos/posX/path/xyz|[x,y,z]|撮影位置姿勢　座標|
|campos/posX/path/rpy|[x,y,z]|撮影位置姿勢　回転角度[deg]|
|campos/posX/path/var|[a,b]|回転範囲|
|campos/posX/path/pitch|int|回転間隔[deg]|

* **範囲指定　Z軸移動撮影（campos/posX/type=path_interval＆campos/posX/path/ip=mov_z）**<br>
「Z軸範囲」の範囲を基準座標のZ軸方向に「Z軸間隔」ごとに、「撮影位置姿勢」にて撮影する

|name|type|description|
|:----|:----|:----|
|campos/posX/path/ip|string|移動タイプ（mov_z）|
|campos/posX/path/xyz|[x,y,z]|撮影位置姿勢　座標|
|campos/posX/path/rpy|[x,y,z]|撮影位置姿勢　回転角度[deg]|
|campos/posX/path/var|[a,b]|Z軸範囲|
|campos/posX/path/pitch|int|Z軸間隔|

* **複数箇所　Z軸回転撮影（campos/posX/type=path_point＆campos/posX/path/ip=rot_z）**<br>
基準座標のZ軸まわりに「回転角度」ごとに、「撮影位置姿勢」にて撮影する

|name|type|description|
|:----|:----|:----|
|campos/posX/path/ip|string|移動タイプ（rot_z）|
|campos/posX/path/xyz|[x,y,z]|撮影位置姿勢　座標|
|campos/posX/path/rpy|[x,y,z]|撮影位置姿勢　回転角度[deg]|
|campos/posX/path/var|[a,b,c,...]|回転角度|

* **複数箇所　Z軸移動撮影（campos/posX/type=path_point＆campos/posX/path/ip=mov_z）**<br>
基準座標のZ軸方向に「Z軸位置」ごとに、「撮影位置姿勢」にて撮影する

|name|type|description|
|:----|:----|:----|
|campos/posX/path/ip|string|移動タイプ（mov_z）|
|campos/posX/path/xyz|[x,y,z]|撮影位置姿勢　座標|
|campos/posX/path/rpy|[x,y,z]|撮影位置姿勢　回転角度[deg]|
|campos/posX/path/var|[a,b,c,...]|Z軸位置|

## model

対象物ごとにモデル（yamlファイル）を登録し、対象物を配置する位置や選択するレシピ名などを指定する。<br>
model.dフォルダ配下にモデル名のフォルダを作成し、model.yamlに下記を登録する。<br>
ファイル読み込み時には、rosparamには反映せず、スクリプト内で必要なパラメータ、タイミングでrosparamに反映する。

|name|type|description|
|:----|:----|:----|
|model|string|モデル名|
|modelpos/frame|string|モデルのフレーム名|
|modelpos/xyz|[x,y,z]|モデル位置姿勢　座標　※1|
|modelpos/rpy|[x,y,z]|モデル位置姿勢　回転角度[deg]　※1|
|modelpos/range_x|int|モデル位置姿勢からのX軸の移動範囲　※2|
|modelpos/range_y|int|モデル位置姿勢からのY軸の移動範囲　※2|
|modelpos/range_z|int|モデル位置姿勢からのZ軸の移動範囲　※2|
|modelpos/range_rx|int|モデル位置姿勢からのX軸の回転範囲　※3|
|modelpos/range_ry|int|モデル位置姿勢からのY軸の回転範囲　※3|
|modelpos/range_rz|int|モデル位置姿勢からのZ軸の回転範囲　※3|
|recipe|string|レシピ名|
|end_show|bool|シーケンス実行後のモデル消去有無|
|solve_save|bool|解析結果の保持有無|
|axis_frame_id|string|モデルの中心軸のフレーム名。解析結果の保持が有効の場合に更新|
|axis_base_frame_id|string|モデルの中心軸の基準座標のフレーム名|
|campos_change|dict|マスタ作成時の撮影位置姿勢から変更する位置姿勢。レシピに登録している撮影位置姿勢と同じ名称を置き換える|

※1　モデルの基準フレームに対する位置姿勢を指定する。指定されていない場合は、現在の位置とする<br>
※2　モデルの移動範囲が指定されている場合は、指定範囲内でランダムに位置姿勢の座標を決定する<br>
※3　モデルの回転範囲が指定されている場合は、指定範囲内でランダムに位置姿勢の回転を決定する
