# Reverse Rosbag Round-Trip Generator

片道走行を記録したROS 1 rosbagから，次の2種類のrosbagを生成するPythonスクリプトである．

- **Round-trip rosbag**  
  元のメッセージ列を往路として保持し，その後ろに時間反転した仮想復路を接続する
- **Reverse-only rosbag**  
  時間反転した仮想復路だけを格納する

本スクリプトは，非ループ環境で取得した片道データから仮想的な復路データを作成し，LiDAR–inertial SLAMの往復処理や軌跡最適化の入力データを準備することを目的とする．

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
- `none`，`lz4`，`bz2`圧縮に対応
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

ROS Melodicなどを使用する場合も，Python 3からROSの`rosbag`および`rospy`を読み込める環境が必要である．

## Usage

基本形式：

```bash
python3 reverse_rosbag_roundtrip.py \
  --input INPUT_BAG \
  --mode MODE
```

`MODE`には次のいずれかを指定する．

| モード | 内容 |
|---|---|
| `roundtrip` | 往路＋仮想復路のbagを生成（デフォルト）|
| `reverse_only` | 仮想復路だけのbagを生成 |
| `both` | 上記2種類を同時に生成 |

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

`--start-offset`と`--end-offset`を使うことで，入力bagの一部区間だけを使用できる．

例えば，入力bag開始から75秒間を使用する場合：

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

オフセットは，デフォルトで除外されるトピックを読み飛ばした後の，**最初の有効なbagメッセージ時刻**を基準に計算される．

デフォルトの除外トピック：

```text
/clock
/rosout
/rosout_agg
```

`--end-offset`は`--start-offset`より大きい値にする必要がある．

## Output File Names

### Single mode

`roundtrip`または`reverse_only`を単独で使用する場合は，`--output`で出力先を指定できる．

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

`both`では，それぞれの出力先を指定する．

```bash
python3 reverse_rosbag_roundtrip.py \
  --input input.bag \
  --mode both \
  --output-roundtrip output/sequence_roundtrip.bag \
  --output-reverse-only output/sequence_reverse_only.bag
```

## Example for SLAM Experiments

入力bagの開始から75秒間を使用し，round-trip bagとreverse-only bagをLZ4圧縮で生成する例である．

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

round-trip bagでは，選択区間のメッセージを元の順序・時刻のまま往路として書き込み，その後ろに逆順のメッセージを仮想復路として接続する．

元メッセージのbag時刻を \(t\)，選択区間の終端時刻を \(t_{\mathrm{end}}\) とすると，仮想復路のbag時刻は次式で計算される．

\[
t_{\mathrm{new}}
=
t_{\mathrm{end}}
+
\left(
t_{\mathrm{end}}-t
\right)
\]

これにより，元の終端側のメッセージから順に，往路終端の後ろへ単調増加する時刻で配置される．

各トピックの`header.stamp`についても，そのトピックの終端header時刻 \(h_{\mathrm{end}}\) を基準に変換する．

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

round-trip bagの仮想復路では，各トピックについて，選択区間内の最後のメッセージを除外する．

これは，往路終端と仮想復路開始に同一メッセージが重複して書き込まれることを防ぐためである．

## Reverse-Only Bag

reverse-only bagでは，元の終端メッセージが出力bagの先頭側に来るように時刻を変換する．

元メッセージのbag時刻を \(t\)，選択区間の開始時刻を \(t_{\mathrm{start}}\)，終端時刻を \(t_{\mathrm{end}}\) とすると，出力bag時刻は次式である．

\[
t_{\mathrm{new}}
=
t_{\mathrm{start}}
+
\left(
t_{\mathrm{end}}-t
\right)
\]

各トピックの`header.stamp`は，そのトピックの開始・終了header時刻を使って次のように変換する．

\[
h_{\mathrm{new}}
=
h_{\mathrm{start}}
+
\left(
h_{\mathrm{end}}-h
\right)
\]

これにより，仮想復路のメッセージは，選択区間の開始時刻付近から単調増加する時刻を持つ．

## Velodyne Packet Handling

`/velodyne_packets`は特別に処理される．

VelodyneScanメッセージについては，次の方針を採用する．

- スキャン単位の順序を反転する
- 1スキャン内部のpacket配列の順序は反転しない
- 各packetとスキャンheaderとの相対時刻を保持する
- 絶対時刻だけを仮想復路の時刻へ移動する

元のスキャンheader時刻を \(h_{\mathrm{old}}\)，新しいheader時刻を \(h_{\mathrm{new}}\)，元のpacket時刻を \(p_{\mathrm{old}}\) とすると，新しいpacket時刻は次式である．

\[
p_{\mathrm{new}}
=
h_{\mathrm{new}}
+
\left(
p_{\mathrm{old}}-h_{\mathrm{old}}
\right)
\]

この処理により，1スキャン内の取得順序とpacket間の時間差を保持する．

`/velodyne_packets`以外のトピック名を使用している場合，この特別処理は適用されない．必要に応じて，ソースコード内のトピック判定を変更する．

## IMU Transformation

仮想復路では，`--imu-topic`で指定したIMUトピックの値を変換できる．

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

角速度と加速度X・Yだけを反転し，重力成分を含む可能性があるZを保持する例：

```bash
--reverse-imu gyro_acc_xy
```

IMU値を変更しない場合：

```bash
--reverse-imu none
```

### Important note

本スクリプトが変更するのは，指定モードに対応する`angular_velocity`および`linear_acceleration`の値である．

次の項目は変更しない．

- IMU姿勢`orientation`
- 各種共分散行列
- センサ外部パラメータ
- IMUバイアス
- 座標軸の定義

使用するSLAMフレームワーク，IMUの座標系，重力の扱いに応じて，適切な反転方法を選択する必要がある．

## Topic Handling

### Skip topics

`--skip-topics`に指定したトピックは，入力読み込み時点で除外される．

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

`--skip-topics`を明示すると，デフォルト値は置き換えられる．デフォルトの除外トピックも必要な場合は，すべて列挙する必要がある．

### Static topics

`--static-topics`に指定したトピックは，仮想復路の通常反転対象から除外される．

デフォルト：

```text
/tf_static
```

round-trip bagでは，静的トピックは往路部分にのみ元のまま含まれる．

reverse-only bagでは，選択区間内に存在する静的トピックのメッセージが，出力bagの先頭時刻に書き込まれる．
通常，`/tf_static`は少数のlatchedメッセージであるが，入力bag内に複数存在する場合は，それらが同じbag時刻で書き込まれる．

独自の静的トピックを追加する例：

```bash
--static-topics /tf_static /robot_description_static
```

## Compression

`--compression`には次の値を指定できる．

| 値 | 内容 |
|---|---|
| `none` | 圧縮なし |
| `lz4` | LZ4圧縮（デフォルト） |
| `bz2` | BZ2圧縮 |

```bash
--compression lz4
```

LZ4を利用できない場合，スクリプトは警告を表示し，圧縮なしへフォールバックする．

BZ2でエラーが発生した場合は自動フォールバックしないため，環境を確認するか`none`を指定する必要がある．

## Command-Line Options

| オプション | デフォルト | 内容 |
|---|---|---|
| `--input` | 必須 | 入力rosbag |
| `--mode` | `roundtrip` | `roundtrip`，`reverse_only`，`both` |
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
