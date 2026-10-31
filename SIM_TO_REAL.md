# 実機SG2へ移す境界

今回のsimは「同型SG2で計画・実行できる経路と座標契約」を確認するサンプルである。実機のホワイトボード消去を、そのまま成功させる完成制御器ではない。オフィスの機体がSG2であることは依頼者確認済み。

## 共通化したもの

board frameは原点、板面外向きnormal、横u、縦vで定義する。3軸の有限値・単位長・直交性・右手系を検査する。工具のEE→contact変換、寸法、板からの距離、strokeの両端と各帯を設定にする。

export_transfer.pyはこれらから接触点とEEのbase座標・quaternionを出力する。pad寸法と帯間隔、宣言した板寸法の範囲を検査し、速度を提案値として出力へ含める。板寸法を指定しない場合はこのsimの36×30cmを使う。実機用にはboard.width_m/height_mも実測値へ置換する。指定帯の被覆範囲を調べるもので、板全体や任意文字の完全消去を保証しない。

    python3 tools/export_transfer.py --config <PRIVATE_CALIBRATED_CONFIG> --output <PRIVATE_PATHS_JSON>

これは座標変換のみであり、ROS/Zenoh/実機への送信機能はない。工具寸法や速度は移植先planner/controllerで再確認して使う。simと同じ姿勢を実機の絶対座標としてコピーしない。

## 工具姿勢で分かったこと

SG2の既定EEはend_effector_r_link。右腕groupはarm_r。標準グリッパはEEのlocal -Zへ伸びる。

初期のidentity姿勢と横方向+X取付案は、下側の帯でarm_r_link7とcamera_right_linkが板に衝突するため棄却した。衝突判定を無効化して通していない。

採用した仮想末端工具はEE→contact=[0,0,-0.08]m、接触frameの相対回転Ry(+90度)。EEをRy(-90度)にすると、接触frameの+Xが板方向のworld+Xに一致する。EE x=.435に対してcontact x=.515、板面x=.520で5mm手前となる。実機の取付金具・消し具厚さ・TCPは未実測なので、8cmという値を実機へ流用してはならない。

## 実機で追加する確認と順序

1. 実SG2の更新状態、URDF・関節順・制限・FollowJointTrajectoryなどのcontroller契約を確認する。simのmodel引数はsg2を明示したが、実機の契約が同一とはまだ確認していない。
2. 消し具を安定して保持・固定し、実測TCP、姿勢、寸法、重量とcollisionをモデルに登録する。板も固定する。
3. 板の角またはタグとカメラ/ロボット座標の校正からboard frameを求める。画角の変化や板の移動を検出できない間は、固定した小領域で扱う。
4. 同じ幾何パスをまず実機のplanning sceneでplan-only検証し、板の手前で非接触dry-runする。手首カメラ・腕・机などを含む干渉を確認する。
5. 接触には、工具の受動コンプライアンス、または確認済みの力覚・admittance等の適切な制御を別途用意する。MoveItで軌道を計画できたことは、板へ適切な力で押し付けられることを意味しない。機体の力覚センサ有無、利用可能なvendor制御、停止手段を確認して試験条件を決める。
6. 実画像から消し残しを測り、必要な帯だけ再計画する。今回のsimセル状態を実カメラのafter maskへ置き換える。実カメラによる認識と消し残しの再計画は次段階であり、このサンプルには含めない。

ROS 2のadmittance controllerにもTCP、力覚センサframe、運動学plugin等の設定が必要であり、SG2でそのまま利用できると確認したものではない。[公式admittance資料](https://control.ros.org/jazzy/doc/ros2_controllers/admittance_controller/doc/userdoc.html)

実機試験は今回行っていない。simの100%という数値は、顔料の落ちやすさ・摩擦・板のたわみ・把持滑りを評価していない。
