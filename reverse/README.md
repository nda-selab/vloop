# Reverse Rosbag Round-Trip Generator

片道走行を記録したROS 1 rosbagから、次の2種類のrosbagを生成するPythonスクリプトです。

- **Round-trip rosbag**  
  元のメッセージ列を往路として保持し、その後ろに時間反転した仮想復路を接続する
- **Reverse-only rosbag**  
  時間反転した仮想復路だけを格納する

本スクリプトは、非ループ環境で取得した片道データから仮想的な復路データを作成し、LiDAR–inertial SLAMの往復処理や軌跡最適化の入力データを準備することを目的としています。

## Features

- ROS 1 bag形式の入力に対応
- 往路＋仮想復路のround-trip bagを生成
- 仮想復路のみのreverse-only bagを生成
- 入力bagの一部区間だけを抽出可能
- 除外トピックと静的トピックを指定可能
- メッセージのbag時刻と`header.stamp`を再構成
- `/velodyne_packets`のスキャン内パケット順序を保持
- Velodyneパケット間の相対時刻を保持
- IMU角速度・加速度の符号反転方法を選択可能
- `none`、`lz4`、`bz2`圧縮に対応
- round-trip bagとreverse-only bagを同時生成可能

## File

```text
reverse_rosbag_roundtrip.py
```

## Requirements

### Recommended environment

- Ubuntu
- ROS 1 Noetic
- Python 3

ROS Melodicなどを使用する場合も、Python 3からROSの`rosbag`および`rospy`を読み込める環境が必要です。

### ROS packages

通常、ROS Noeticがインストールされていれば必要な基本パッケージは利用できます。

```bash
sudo apt update
sudo apt install ros-noetic-rosbag ros-noetic-rospy
```

入力bagに含まれるメッセージ型に対応するROSパッケージも必要です。例えば、`/velodyne_packets`を含むbagを扱う場合は、Velodyneメッセージ定義をインストールします。

```bash
sudo apt install ros-noetic-velodyne-msgs
```

ROS環境を読み込んでください。

```bash
source /opt/ros/noetic/setup.bash
```

ワークスペース固有のメッセージ型を使用する場合は、そのワークスペースも読み込みます。

```bash
source ~/catkin_ws/devel/setup.bash
```

## Usage

基本形式：

```bash
python3 reverse_rosbag_roundtrip.py \
  --input INPUT_BAG \
  --mode MODE
```

`MODE`には次のいずれかを指定します。

| モード | 内容 |
|---|---|
| `roundtrip` | 往路＋仮想復路のbagを生成 |
| `reverse_only` | 仮想復路だけのbagを生成 |
| `both` | 上記2種類を同時に生成 |

デフォルトは`roundtrip`です。

## Quick Start

### Round-trip bagを生成

```bash
python3 reverse_rosbag_roundtrip.py \
  --input input.bag \
  --mode roundtrip
```

出力：

```text
input_roundtrip.bag
```

### Reverse-only bagを生成

```bash
python3 reverse_rosbag_roundtrip.py \
  --input input.bag \
  --mode reverse_only
```

出力：

```text
input_reverse_only.bag
```

### 2種類のbagを同時生成

```bash
python3 reverse_rosbag_roundtrip.py \
  --input input.bag \
  --mode both
```

出力：

```text
input_roundtrip.bag
input_reverse_only.bag
```

## Selecting a Time Range

`--start-offset`と`--end-offset`を使うことで、入力bagの一部区間だけを使用できます。

例えば、入力bag開始から75秒間を使用する場合：

```bash
python3 reverse_rosbag_roundtrip.py \
  --input input.bag \
  --mode both \
  --start-offset 0 \
  --end-offset 75
```

入力bag開始から30秒後から75秒後までを使用する場合：

```bash
python3 reverse_rosbag_roundtrip.py \
  --input input.bag \
  --mode both \
  --start-offset 30 \
  --end-offset 75
```

### Offsetの基準

オフセットは、デフォルトで除外されるトピックを読み飛ばした後の、**最初の有効なbagメッセージ時刻**を基準に計算されます。

デフォルトの除外トピック：

```text
/clock
/rosout
/rosout_agg
```

`--end-offset`は`--start-offset`より大きい値にする必要があります。

## Output File Names

### Single mode

`roundtrip`または`reverse_only`を単独で使用する場合は、`--output`で出力先を指定できます。

```bash
python3 reverse_rosbag_roundtrip.py \
  --input input.bag \
  --mode roundtrip \
  --output output/sequence_roundtrip.bag
```

```bash
python3 reverse_rosbag_roundtrip.py \
  --input input.bag \
  --mode reverse_only \
  --output output/sequence_reverse_only.bag
```

### Both mode

`both`では、それぞれの出力先を指定します。

```bash
python3 reverse_rosbag_roundtrip.py \
  --input input.bag \
  --mode both \
  --output-roundtrip output/sequence_roundtrip.bag \
  --output-reverse-only output/sequence_reverse_only.bag
```

出力先ディレクトリは自動作成されないため、事前に作成してください。

```bash
mkdir -p output
```

## Example for SLAM Experiments

入力bagの開始から75秒間を使用し、round-trip bagとreverse-only bagをLZ4圧縮で生成する例です。

```bash
mkdir -p generated_bags

python3 reverse_rosbag_roundtrip.py \
  --input data/subt_multifloor.bag \
  --mode both \
  --start-offset 0 \
  --end-offset 75 \
  --output-roundtrip generated_bags/subt_multifloor_roundtrip.bag \
  --output-reverse-only generated_bags/subt_multifloor_reverse_only.bag \
  --imu-topic /imu/data \
  --reverse-imu gyro \
  --compression lz4
```

## Timestamp Conversion

## Round-Trip Bag

round-trip bagでは、選択区間のメッセージを元の順序・時刻のまま往路として書き込み、その後ろに逆順のメッセージを仮想復路として接続します。

元メッセージのbag時刻を \(t\)、選択区間の終端時刻を \(t_{\mathrm{end}}\) とすると、仮想復路のbag時刻は次式で計算されます。

\[
t_{\mathrm{new}}
=
t_{\mathrm{end}}
+
\left(
t_{\mathrm{end}}-t
\right)
\]

これにより、元の終端側のメッセージから順に、往路終端の後ろへ単調増加する時刻で配置されます。

各トピックの`header.stamp`についても、そのトピックの終端header時刻 \(h_{\mathrm{end}}\) を基準に変換します。

\[
h_{\mathrm{new}}
=
h_{\mathrm{end}}
+
\left(
h_{\mathrm{end}}-h
\right)
\]

### 往路終端の重複回避

round-trip bagの仮想復路では、各トピックについて、選択区間内の最後のメッセージを除外します。

これは、往路終端と仮想復路開始に同一メッセージが重複して書き込まれることを防ぐためです。

## Reverse-Only Bag

reverse-only bagでは、元の終端メッセージが出力bagの先頭側に来るように時刻を変換します。

元メッセージのbag時刻を \(t\)、選択区間の開始時刻を \(t_{\mathrm{start}}\)、終端時刻を \(t_{\mathrm{end}}\) とすると、出力bag時刻は次式です。

\[
t_{\mathrm{new}}
=
t_{\mathrm{start}}
+
\left(
t_{\mathrm{end}}-t
\right)
\]

各トピックの`header.stamp`は、そのトピックの開始・終了header時刻を使って次のように変換します。

\[
h_{\mathrm{new}}
=
h_{\mathrm{start}}
+
\left(
h_{\mathrm{end}}-h
\right)
\]

これにより、仮想復路のメッセージは、選択区間の開始時刻付近から単調増加する時刻を持ちます。

## Velodyne Packet Handling

`/velodyne_packets`は特別に処理されます。

VelodyneScanメッセージについては、次の方針を採用します。

- スキャン単位の順序を反転する
- 1スキャン内部のpacket配列の順序は反転しない
- 各packetとスキャンheaderとの相対時刻を保持する
- 絶対時刻だけを仮想復路の時刻へ移動する

元のスキャンheader時刻を \(h_{\mathrm{old}}\)、新しいheader時刻を \(h_{\mathrm{new}}\)、元のpacket時刻を \(p_{\mathrm{old}}\) とすると、新しいpacket時刻は次式です。

\[
p_{\mathrm{new}}
=
h_{\mathrm{new}}
+
\left(
p_{\mathrm{old}}-h_{\mathrm{old}}
\right)
\]

この処理により、1スキャン内の取得順序とpacket間の時間差を保持します。

`/velodyne_packets`以外のトピック名を使用している場合、この特別処理は適用されません。必要に応じて、ソースコード内のトピック判定を変更してください。

## IMU Transformation

仮想復路では、`--imu-topic`で指定したIMUトピックの値を変換できます。

デフォルト：

```text
--imu-topic /imu/data
--reverse-imu gyro
```

### Available modes

| モード | 角速度 | 加速度X/Y | 加速度Z |
|---|---:|---:|---:|
| `none` | 変更しない | 変更しない | 変更しない |
| `gyro` | 符号反転 | 変更しない | 変更しない |
| `gyro_acc` | 符号反転 | 符号反転 | 符号反転 |
| `gyro_acc_xy` | 符号反転 | 符号反転 | 変更しない |

角速度のみを反転する例：

```bash
--reverse-imu gyro
```

角速度と加速度の全成分を反転する例：

```bash
--reverse-imu gyro_acc
```

角速度と加速度X・Yだけを反転し、重力成分を含む可能性があるZを保持する例：

```bash
--reverse-imu gyro_acc_xy
```

IMU値を変更しない場合：

```bash
--reverse-imu none
```

### Important note

本スクリプトが変更するのは、指定モードに対応する`angular_velocity`および`linear_acceleration`の値です。

次の項目は変更しません。

- IMU姿勢`orientation`
- 各種共分散行列
- センサ外部パラメータ
- IMUバイアス
- 座標軸の定義

使用するSLAMフレームワーク、IMUの座標系、重力の扱いに応じて、適切な反転方法を選択してください。

## Topic Handling

### Skip topics

`--skip-topics`に指定したトピックは、入力読み込み時点で除外されます。

デフォルト：

```text
/clock
/rosout
/rosout_agg
```

独自に指定する例：

```bash
python3 reverse_rosbag_roundtrip.py \
  --input input.bag \
  --mode roundtrip \
  --skip-topics /clock /rosout /rosout_agg /diagnostics
```

`--skip-topics`を明示すると、デフォルト値は置き換えられます。デフォルトの除外トピックも必要な場合は、すべて列挙してください。

### Static topics

`--static-topics`に指定したトピックは、仮想復路の通常反転対象から除外されます。

デフォルト：

```text
/tf_static
```

round-trip bagでは、静的トピックは往路部分にのみ元のまま含まれます。

reverse-only bagでは、選択区間内に存在する静的トピックのメッセージが、出力bagの先頭時刻に書き込まれます。通常、`/tf_static`は少数のlatchedメッセージですが、入力bag内に複数存在する場合は、それらが同じbag時刻で書き込まれます。

独自の静的トピックを追加する例：

```bash
--static-topics /tf_static /robot_description_static
```

## Compression

`--compression`には次の値を指定できます。

| 値 | 内容 |
|---|---|
| `none` | 圧縮なし |
| `lz4` | LZ4圧縮 |
| `bz2` | BZ2圧縮 |

デフォルトは`lz4`です。

```bash
--compression lz4
```

LZ4を利用できない場合、スクリプトは警告を表示し、圧縮なしへフォールバックします。

BZ2でエラーが発生した場合は自動フォールバックしないため、環境を確認するか`none`を指定してください。

## Command-Line Options

| オプション | デフォルト | 内容 |
|---|---|---|
| `--input` | 必須 | 入力rosbag |
| `--mode` | `roundtrip` | `roundtrip`、`reverse_only`、`both` |
| `--output` | 未指定 | 単一モード時の出力bag |
| `--output-roundtrip` | 未指定 | round-trip bagの出力先 |
| `--output-reverse-only` | 未指定 | reverse-only bagの出力先 |
| `--start-offset` | `0.0` | 入力基準時刻からの開始オフセット [s] |
| `--end-offset` | bag終端 | 入力基準時刻からの終了オフセット [s] |
| `--skip-topics` | `/clock /rosout /rosout_agg` | 入力から除外するトピック |
| `--static-topics` | `/tf_static` | 反転しない静的トピック |
| `--imu-topic` | `/imu/data` | IMUトピック名 |
| `--reverse-imu` | `gyro` | IMU値の反転方法 |
| `--compression` | `lz4` | 出力bagの圧縮方式 |

ヘルプ表示：

```bash
python3 reverse_rosbag_roundtrip.py --help
```

## Output Verification

生成後は、`rosbag info`で内容を確認できます。

```bash
rosbag info input_roundtrip.bag
```

```bash
rosbag info input_reverse_only.bag
```

再生例：

```bash
rosbag play input_roundtrip.bag --clock
```

```bash
rosbag play input_reverse_only.bag --clock
```

再生速度を下げる場合：

```bash
rosbag play input_roundtrip.bag --clock -r 0.5
```

SLAMの処理が点群入力に追いつかない場合は、再生速度を下げて検証してください。

## Recommended Workflow

1. 入力bagのトピックと時間範囲を確認する

```bash
rosbag info input.bag
```

2. 必要な区間とIMU反転方法を決める

3. round-trip bagとreverse-only bagを生成する

```bash
python3 reverse_rosbag_roundtrip.py \
  --input input.bag \
  --mode both \
  --start-offset 0 \
  --end-offset 75 \
  --reverse-imu gyro
```

4. 出力bagのトピック数・メッセージ数・durationを確認する

```bash
rosbag info input_roundtrip.bag
rosbag info input_reverse_only.bag
```

5. 低速再生で各トピックが正常に出力されるか確認する

```bash
rosbag play input_reverse_only.bag --clock -r 0.2
```

6. SLAMを実行して、往路・仮想復路の軌跡CSVを取得する

7. 軌跡最適化プログラムへ入力する

## Example Repository Structure

```text
.
├── README.md
├── reverse_rosbag_roundtrip.py
├── data
│   └── input.bag
└── generated_bags
    ├── input_roundtrip.bag
    └── input_reverse_only.bag
```

## Important Notes

### Entire selected range is loaded into memory

スクリプトは、選択した時間範囲内のメッセージを一度リストへ読み込んだ後、出力bagを書き込みます。

そのため、長時間または大容量のrosbagでは、多くのメモリを使用する可能性があります。

必要に応じて、`--start-offset`と`--end-offset`で対象範囲を短くしてください。

### Message definitions must be installed

入力bagを読み込む環境に、bag内で使用されているROSメッセージ型が存在しない場合、デシリアライズに失敗する可能性があります。

独自メッセージを含む場合は、対応するcatkinワークスペースをビルドしてから`setup.bash`を読み込んでください。

### Existing output files

出力bagは書き込みモードで開かれます。既存ファイルを誤って置き換えないよう、出力先を確認してください。

### `/clock` is excluded by default

入力bag内の`/clock`はデフォルトで除外されます。

`rosbag play --clock`を使用すると、rosbag playerが再生時刻に基づく`/clock`を新たに配信します。

### Topic-specific header timestamps

`header.stamp`の変換は各トピックの開始・終了header時刻を基準に行われます。

トピック間でheader時刻の基準が大きく異なる場合、出力bagのbag時刻とheader時刻の差もトピックごとに異なる可能性があります。

### Headerを持たないメッセージ

`header.stamp`を持たないメッセージでは、bag時刻だけが反転されます。

### TF messages

`TFMessage`のように`transforms`配列を持つメッセージでは、各`transform.header.stamp`も更新されます。

`/tf_static`はデフォルトで静的トピックとして扱われるため、通常は仮想復路用の時刻更新対象になりません。

## Troubleshooting

### `ModuleNotFoundError: No module named 'rosbag'`

ROS環境が読み込まれていない、またはROSのPythonパッケージが不足しています。

```bash
source /opt/ros/noetic/setup.bash
python3 -c "import rosbag; print(rosbag.__file__)"
```

### `ModuleNotFoundError` for a message package

入力bagで使用されているメッセージ定義をインストールするか、対応するワークスペースを読み込んでください。

例：

```bash
source ~/catkin_ws/devel/setup.bash
```

### `有効なメッセージがありません`

次の項目を確認してください。

- `--input`が正しいか
- `--skip-topics`で必要な全トピックを除外していないか
- `--start-offset`がbagのdurationを超えていないか
- `--end-offset`が開始時刻より前になっていないか

入力bagの情報：

```bash
rosbag info input.bag
```

### `--end-offset は --start-offset より大きい値にしてください`

例えば、次の指定は無効です。

```bash
--start-offset 75 --end-offset 30
```

終了オフセットを開始オフセットより大きくしてください。

```bash
--start-offset 30 --end-offset 75
```

### LZ4 unavailable

LZ4圧縮が利用できない場合、スクリプトは圧縮なしへフォールバックします。

明示的に圧縮なしを指定することもできます。

```bash
--compression none
```

### SLAMで点群が出力されない

次の項目を確認してください。

- 点群・IMU・TFの`header.stamp`が単調増加しているか
- SLAMが期待するトピック名と一致しているか
- IMU反転方法がSLAMの運動モデルと整合しているか
- `/tf_static`が再生開始時に利用可能か
- 再生速度が速すぎないか
- LiDARメッセージ型に応じた内部時刻が適切か

まず低速で再生してください。

```bash
rosbag play input_reverse_only.bag --clock -r 0.1
```

### Output directory does not exist

出力ディレクトリは自動作成されません。

```bash
mkdir -p generated_bags
```

## Limitations

- ROS 2 bagには対応していません。
- すべての選択メッセージをメモリへ保持します。
- `/velodyne_packets`以外のLiDAR固有内部時刻は特別処理されません。
- IMU姿勢と共分散は反転・変換されません。
- センサの物理的な時間反転を完全に再現するものではありません。
- 出力bagの妥当性は、使用するSLAMフレームワークとセンサモデルに依存します。
- 入力bagに不規則な時刻、重複時刻、異なる時刻基準が含まれる場合は、追加処理が必要になることがあります。

## License

公開時には、用途に応じたライセンスを設定してください。

例：

```text
MIT License
```
