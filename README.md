# AIWorker-SG2 ホワイトボード消去シミュレーション

ROBOTIS AIWorker-SG2の公式Gazeboモデルで、Q-learningが選んだ順序に従い、MoveItで右腕を動かしてホワイトボードの「AI」を消すテストサンプルです。

**[説明レポート PDF](REPORT.pdf) · [レポート本文](REPORT.md) · [実演動画 MP4](media/whiteboard_wipe.mp4)**

動画は **95.75秒・等速**、実際に動作したGazebo画面だけです。GitHub上で再生できない場合は、動画ファイルの「View raw」からダウンロードしてください。

| 実測項目 | 公開動画のrun（wipe06） |
| --- | --- |
| 対象 | 36×30cmの板、27個の文字セル |
| 実行 | 3帯、15本のCartesian経路、全fraction=1.0 |
| 消去 | 27/27セル、残存0 |
| 実行時間 | 101.90秒 |
| 動画 | 1280×640、24fps、95.75秒、1.0倍 |

これは**幾何学的な消去代理モデル**です。実測TFが新しく、工具の位置・姿勢・被覆の条件を満たしたセルだけをGazeboから削除します。工具表示はTFに追従するvisual proxyで、接触点は板の5mm手前です。顔料・摩擦・押付け力は再現しておらず、実機での清掃成功を示すものではありません。

AIは固定した3帯の順序を選ぶtabular Q-learningです。状態は現在帯と訪問済み集合、報酬は移動距離・最初の帯の優先度・終了報酬です。seed=20261030、4000 episodeで学習します。画像認識・消し残しに応じた適応制御は含みません。

## 収録物

- `REPORT.pdf` / `REPORT.md`: 目的、設計、実測、失敗と改善、加点要素との対応。
- `media/`: 実演MP4と編集区間・速度・ハッシュのmanifest。
- `tools/`: シーン生成、MoveIt実行、消去判定、録画・編集、監査、PDF生成。
- `src/gazebo_aiworker_patrol/`: 同梱したROS非依存のQ-learningと状態定義。元の汎用モジュール名を保持しています。
- `config/` / `transfer_paths.example.json`: 板・工具座標と幾何パスの例。
- `test/`: 消去条件、座標変換、学習器のテスト。
- `evidence/wipe06/` / `RESULTS.json`: 公開動画の実測ログ・mask・方策と集計。開発中の失敗も集計へ残しています。
- [SIM_TO_REAL.md](SIM_TO_REAL.md): 実機SG2へ移すための校正と未検証事項。
- [VALIDATION.md](VALIDATION.md): 公開版の実行検証記録。

## 実行環境

Linux/WSL2のDocker Engineで実行します。公式ROBOTIS imageはamd64、ROS 2 Jazzy・Gazebo Harmonic・MoveIt入りの2.3.1をdigestで固定しています。録画用Xvfb/ffmpeg、日本語PDF用ReportLab/IPAフォントはDockerfileへ明記しました。CPU描画を使い、GPUは必須ではありません。image展開後は約7GBに加え、build・録画用の空き領域が必要です。

このリポジトリ単独で実行でき、別リポジトリや実機ネットワークへの接続は不要です。専用コンテナ・ROS domain64を使い、host network、実機デバイス、公開ポートは使いません。GUIはコンテナ内の仮想画面へ描画し、録画をファイルに保存します。

### package.xmlがない理由

このリポジトリの独自部分は `tools/` のPythonスクリプトと、`PYTHONPATH=/project/src` から読み込む学習器です。独自のROSパッケージとしてビルドしないため、ここに `package.xml` や `colcon build` は必要ありません。`ros2 launch` / `ros2 run` で使う `ffw_bringup`、`ffw_moveit_config`、`rmw_zenoh_cpp` は公式Dockerイメージ内に、各 `package.xml` を含めて導入済みです。`tools/ros_env.sh` がそのROS環境を読み込みます。ホストへROSを直接導入して使う構成は、今回の再現手順には含めません。

### 起動手順

```bash
git clone https://github.com/choconium/physical-ai-advanced1.git
cd physical-ai-advanced1
bash tools/container.sh build
bash tools/container.sh start
bash tools/container.sh exec python3 -m unittest discover -s test -v
bash tools/container.sh exec bash tools/start_sim.sh
```

最後の起動コマンドは継続させます。別ターミナルで同じディレクトリを開き、約30秒待ってから実行します。

```bash
bash tools/container.sh exec python3 tools/record_screen.py \
  --output-dir artifacts/capture_my_wipe -- \
  python3 tools/run_wipe.py --run-id my_wipe --execute-sim
```

約2分で完了します。run ID・録画先の上書きを拒否します。再試行は `stop` → `remove` → `start` → `start_sim.sh` で初期状態へ戻し、新しいrun IDを使います。

```bash
bash tools/container.sh exec python3 tools/video_motion.py \
  --samples artifacts/my_wipe/tool_samples.jsonl \
  --output artifacts/my_wipe/motion.jsonl --skip-first-seconds 5
bash tools/container.sh exec python3 tools/assemble_video.py \
  --capture-dir artifacts/capture_my_wipe \
  --motion-log artifacts/my_wipe/motion.jsonl \
  --output artifacts/my_wipe.mp4 --target-seconds 120
bash tools/container.sh exec python3 tools/audit_run.py \
  --run-dir artifacts/my_wipe --output artifacts/my_wipe_audit.json
```

編集は実際の動作区間を残し、1分未満の素材を引き伸ばしたり複製したりしません。生成結果の時間・速度は隣接するJSONで確認してください。公開動画・集計を上書きせず、新しい結果は `artifacts/` に保存します。

公開済みログの照合とPDF再生成（再監査のJSONには、その環境の `artifacts/wipe*/summary.json` の試行だけが列挙されます。開発時の試行一覧は公開 `RESULTS.json` を参照）:

```bash
bash tools/container.sh exec python3 tools/audit_run.py \
  --run-dir evidence/wipe06 --output artifacts/published_run_audit.json
bash tools/container.sh exec python3 tools/render_report.py \
  --input REPORT.md --output artifacts/REPORT.pdf
```

計画だけの確認・座標出力:

```bash
bash tools/container.sh exec python3 tools/moveit_adapter.py
bash tools/container.sh exec python3 tools/export_transfer.py \
  --config config/transfer.example.json --output artifacts/transfer_paths.json
```

`moveit_adapter.py` は `--execute-sim` を付けなければ計画だけを行います。`export_transfer.py` はROSを使わない座標変換で、実機へ命令を送りません。実行runnerはhash固定のsim profile以外を拒否します。

終了:

```bash
bash tools/container.sh stop
bash tools/container.sh remove
```

## 実機への移行

板・工具の座標契約を共有しますが、実機SG2のソフト更新、controller契約、工具固定・TCP・板面校正、接触制御、実画像の消し残し評価は追加作業です。今回のサンプルに実機実行モードはありません。詳細は [SIM_TO_REAL.md](SIM_TO_REAL.md) に記載しました。

## ライセンス・参照

本リポジトリの独自コード・文書・動画は [Apache-2.0](LICENSE) で公開します。公式ROBOTIS image・モデル・依存パッケージは各権利者のライセンスに従います。モデル/sourceの複製は同梱していません。詳細は [NOTICE.md](NOTICE.md) を参照してください。

- [ROBOTIS Gazebo/MoveIt documentation](https://docs.robotis.com/docs/systems/aiworker/simulation/gazebo/)
- [実行image内のai_worker source revision](https://github.com/ROBOTIS-GIT/ai_worker/tree/691db567f873edca340eea01f190a28935782412)
