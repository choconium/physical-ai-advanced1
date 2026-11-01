# 公開版の検証記録

検証日: 2026-11-01（JST）。WSL2 / Docker Desktop、CPUによる描画。
公開版を新しい作業フォルダと専用コンテナで検証した。実機へは接続していない。

## 単独実行の確認

元の提出物から、ホワイトボードタスクと必要な学習器・録画ツールだけを集約した。
モーション・消去・座標変換のアルゴリズム、設定、動画の内容は維持している。
変更点はDocker定義と起動スクリプトの独立化、import探索先、公開文書・PDF metadata、
学習器の説明、末尾空行の整形、および学習器のテスト追加。

実際に実行した主要コマンド:

```bash
bash tools/container.sh build
bash tools/container.sh start
bash tools/container.sh exec python3 -m unittest discover -s test -v
bash tools/container.sh exec bash tools/start_sim.sh
# 別ターミナル。初期化後:
bash tools/container.sh exec python3 tools/run_wipe.py --run-id wipe_publication --execute-sim
bash tools/container.sh exec python3 tools/audit_run.py --run-dir artifacts/wipe_publication --output artifacts/publication_audit.json
bash tools/container.sh exec python3 tools/audit_run.py --run-dir evidence/wipe06 --output artifacts/published_run_audit.json
bash tools/container.sh exec python3 tools/export_transfer.py --config config/transfer.example.json --output artifacts/transfer_paths.json
bash tools/container.sh exec python3 tools/render_report.py --input REPORT.md --output REPORT.pdf
```

単体テストは **13件すべて成功**。公開レポートの10件は録画実験時の件数で、公開版では
同梱した学習器に対する3件（順序・重複拒否・異なるmission拒否）を追加した。
最終整形後も13件を再実行し成功した。

公開版の独立した再実行 `wipe_publication` の実測:

| 指標 | 結果 |
| --- | --- |
| 成功 | true |
| 完了帯 | 3 / 3 |
| Cartesian経路 | 15本、全fraction=1.0 |
| 消去 | 27 / 27セル、残存0 |
| mask画素 | 5292 → 0 |
| 実行時間 | 99.769695052秒 |
| 有効TFサンプル | 1986 |
| TF拒否 | 2、拒否時の消去なし |

[再実行の監査結果](evidence/publication_audit.json) と
[その実測ログ](evidence/wipe_publication/) を収録した。
公開動画は以前の録画run `wipe06` の実画面であり、この再実行の録画ではない。
複数のrunをつないだ動画にはしていない。

## 成果物・静的検査

次も実行した:

- `python3 -m compileall -q tools src test`（コンテナ内）。
- 全 `tools/*.sh` への `bash -n`、`git diff --cached --check`。
- `ffprobe -v error -show_entries format=duration:stream=codec_name,codec_type,width,height,r_frame_rate,nb_frames -of json media/whiteboard_wipe.mp4`。
- `ffmpeg -v error -i media/whiteboard_wipe.mp4 -f null -` による全体decode。
- `pdfinfo REPORT.pdf`、`pdftoppm` による全3頁の画像化と目視。
- RESULTSの6つのSHA-256とwipe06原本、動画・motion manifestのSHA-256を照合。
- 座標exportの出力が公開 `transfer_paths.example.json` とbyte一致することを確認。
- 追跡対象の文書・コード・ログを、旧リポジトリ依存、不要な話題、個人情報、認証情報の観点で確認。

動画はH.264、1280×640、24fps、2298フレーム、**95.75秒・等速**。
SHA-256は `2aa22a97dddde88d24547536e1e1b1d6ef02170455bc8231a45aac0c657b11c1`。
PDFはA4・3頁・IPAフォント埋込み、Authorは公開用OmniCampus account表記（chocopan）。

## 3回のレビュー

1. 依存レビュー: 同梱した学習器と録画ツール、Docker/ROS環境を照合し、別リポジトリへの依存解消を確認した。
2. 内容レビュー: 公開範囲、不要な記述・個人情報、成果値、実機未検証という限界の記述を確認した。
3. 成果物レビュー: PDF全頁、MP4 metadataとdecode、実測原本とmanifestのハッシュ一致を別担当が確認した。

指摘された検証記録の欠落を本書で補い、学習器の説明をホワイトボードの帯選択へ更新した。

## 残る制約

幾何学的消去の代理モデルであり、顔料・摩擦・接触力・工具の物理的把持は評価していない。
実機のcontroller契約、工具と板面の校正、接触制御、実画像による消し残し認識は未検証。
最終モデルの少数試行は、未知の板位置や文字への一般化性能を示すものではない。
接続断・停止要求の全故障ケースも検証していない。

ROBOTIS imageはdigestで固定しているが、Dockerfileの追加aptパッケージは個別のversion固定ではない。
別ホスト/別OS/別CPU構成での実行は未検証。

## レポートの表記・レイアウト確認

公開用アカウント・メール・提出日・テーマを指定値へ更新し、ホワイトボード表記に統一した。
PDFのタイトルは「Physical AI 応用1 最終課題」、Authorはchocopan、Subjectはテーマとした。
第6章の観点列は本文9ptの日本語3文字と左右余白12ptを合わせた39ptとし、
見出しと表全体を同じページへ配置する。

`tools/render_report.py` の実行、`pdfinfo`、`pdftotext -layout`、`pdftoppm` による
再生成・metadata・指定項目・全3頁の目視確認を行い、別担当も表幅と内容を照合した。

## ロボット図版・パッケージ構成の確認

第2章へユーザー提供のAI Worker外観参考とGazebo SG2の図を追加し、MarkdownとPDFの
双方に同じ画像・キャプションを収録した。画像は縦横比を維持して横並びに配置し、
`pdfimages -list REPORT.pdf` で600×589および3191×1850の2画像の埋込みを確認した。
再生成したPDF全3頁を目視し、第6章の細い観点列と表の同頁配置も再確認した。

専用コンテナで `ros2 pkg prefix ffw_bringup`、`ros2 pkg prefix ffw_moveit_config`、
`ros2 pkg prefix rmw_zenoh_cpp` を実行し、各prefixの `share/<package>/package.xml` の
存在を確認した。前2者は `/root/ros2_ws/install/` 配下、RMWは `/opt/ros/jazzy` 配下にある。
`tools/container.sh exec python3 -c` で同梱の `Mission` と `train` のimportも成功した。
独自部分はPythonスクリプトとして直接実行するため、このrepoに独自のpackage.xmlは不要。
理由と実行境界はREADMEの実行環境にも記載した。

## 説明資料の追加要件レビュー（2026-11-02）

| 要件 | 対応 |
| --- | --- |
| 課題の概要・目的 | 第1章「課題の概要・目的と結論」に対象タスクとsimでの検証目的を明記 |
| 工夫した点 | 第4章「工夫した点と設計変更」、第6章の観点別整理 |
| PDF形式の簡単な説明資料 | 図2点・実測結果を含むA4・3ページのPDF |
| ファイル名 | 説明資料_PAI最終課題_chocopan.pdf |

提出用PDFは `tools/render_report.py --input REPORT.md` で指定名へ生成した。
同一内容を既存リンク用 `REPORT.pdf` へコピーし、byte一致を確認した。
`pdfinfo`、`pdfimages -list`、`pdftotext -layout`、`pdftoppm` による形式・画像・本文・
全3頁の表示を確認し、別担当も要件の対応と指定ファイル名を再レビューした。
