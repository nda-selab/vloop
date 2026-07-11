# Virtual Round-Trip Trajectory Optimization

往路軌跡と仮想復路軌跡を用いて、往路軌跡をオフラインで最適化するPythonスクリプト群です。

非ループ環境では、実際のループ閉じ込みを利用できない場合があります。本リポジトリでは、往路のセンサデータを時間反転して生成した仮想復路を利用し、往路・復路から得られる相対位置姿勢拘束を統合することで、往路軌跡の累積誤差低減を試みます。

## Scripts

| ファイル | 入力 | 主な拘束 | 想定する座標系 |
|---|---|---|---|
| `optimize_forward_reverse_relative.py` | 往路CSVと仮想復路CSV | 往路相対拘束、復路相対拘束、任意の仮想ループ拘束 | 往路と復路は別々の座標系でも使用可能 |
| `optimize_roundtrip_absolute_relative.py` | 往路と仮想復路を接続してSLAMした往復CSV | 往路相対拘束、復路相対拘束、対応ノード間の絶対拘束、任意の仮想ループ拘束 | 往路と復路が同一座標系 |

## Method Overview

### 1. 分割型：相対拘束による最適化

`optimize_forward_reverse_relative.py`は、往路と仮想復路を別々にSLAMして得た軌跡を使用します。

別々に推定した軌跡は、原点や初期姿勢が異なる可能性があるため、仮想復路の絶対位置姿勢は使用せず、隣接ノード間の相対変化量を拘束として利用します。

目的関数の概念は次のとおりです。

\[
\min_{\{T_i\}}
\sum_i
\left\|
\operatorname{Log}
\left(
(Z_i^{\mathrm{fwd}})^{-1}T_i^{-1}T_{i+1}
\right)
\right\|_{\Sigma_{\mathrm{fwd}}^{-1}}^2
+
\sum_i
\left\|
\operatorname{Log}
\left(
(Z_i^{\mathrm{rev}})^{-1}T_i^{-1}T_{i+1}
\right)
\right\|_{\Sigma_{\mathrm{rev}}^{-1}}^2
\]

オプションで、仮想復路が示す終端から始点への相対変換を、仮想ループ拘束として追加できます。

### 2. 非分割型：相対拘束と絶対拘束による最適化

`optimize_roundtrip_absolute_relative.py`は、往路と仮想復路を一つのrosbagとして連続再生し、同一のSLAM実行から得た往復軌跡を使用します。

往路区間と復路区間が同一座標系で表されるため、相対拘束に加えて、対応する往路ノードと仮想復路ノードの絶対位置姿勢差を拘束として使用できます。

目的関数は、概念的には次の拘束で構成されます。

1. 往路の隣接ノード間相対拘束
2. 仮想復路の隣接ノード間相対拘束
3. 対応する往復ノード間の絶対位置拘束
4. 任意の絶対姿勢拘束
5. 任意の仮想ループ拘束

## Requirements

- Python 3
- NumPy
- pandas
- SciPy

インストール例：

```bash
python3 -m pip install numpy pandas scipy
```

## Input CSV Format

両スクリプトとも、次の列を持つCSVファイルを入力とします。

```text
timestamp,
p_w_b_x,
p_w_b_y,
p_w_b_z,
q_w_b_x,
q_w_b_y,
q_w_b_z,
q_w_b_w
```

| 列名 | 内容 |
|---|---|
| `timestamp` | 軌跡の時刻 |
| `p_w_b_x` | world座標系におけるbody位置のX成分 |
| `p_w_b_y` | world座標系におけるbody位置のY成分 |
| `p_w_b_z` | world座標系におけるbody位置のZ成分 |
| `q_w_b_x` | bodyからworldへの姿勢クォータニオンX成分 |
| `q_w_b_y` | bodyからworldへの姿勢クォータニオンY成分 |
| `q_w_b_z` | bodyからworldへの姿勢クォータニオンZ成分 |
| `q_w_b_w` | bodyからworldへの姿勢クォータニオンW成分 |

読み込み時には、欠損行の除去、時刻順の並べ替え、クォータニオンの正規化が行われます。

## Usage

### Separate Forward and Reverse Trajectories

往路と仮想復路を別々にSLAMして得たCSVを使用する場合：

```bash
python3 optimize_forward_reverse_relative.py \
  --forward-csv forward.csv \
  --reverse-csv reverse.csv \
  --output-prefix results/relative
```

仮想ループ拘束を追加する場合：

```bash
python3 optimize_forward_reverse_relative.py \
  --forward-csv forward.csv \
  --reverse-csv reverse.csv \
  --output-prefix results/relative_loop \
  --use-loop
```

復路CSVの時刻がすでに往路の元時間軸に対応している場合：

```bash
python3 optimize_forward_reverse_relative.py \
  --forward-csv forward.csv \
  --reverse-csv reverse.csv \
  --reverse-time-mode as_is
```

通常、時間反転したreverse-only rosbagから得たCSVには、デフォルトの`invert`を使用します。

```bash
--reverse-time-mode invert
```

### Connected Round-Trip Trajectory

往路と仮想復路を接続して一つのSLAM実行から得たCSVを使用する場合：

```bash
python3 optimize_roundtrip_absolute_relative.py \
  --roundtrip-csv roundtrip.csv \
  --split-time 1660000075.0 \
  --output-prefix results/roundtrip
```

サンプル番号で往路と復路を分割する場合：

```bash
python3 optimize_roundtrip_absolute_relative.py \
  --roundtrip-csv roundtrip.csv \
  --split-index 1500 \
  --output-prefix results/roundtrip
```

`--split-index`と`--split-time`を両方指定した場合は、`--split-index`が優先されます。

分割位置を指定しない場合は、入力CSVの最小時刻と最大時刻の中央が使用されます。ただし、往路と復路の長さが異なるデータでは、明示的に分割位置を指定することを推奨します。

絶対位置拘束だけを使用し、絶対姿勢拘束を無効にする場合：

```bash
python3 optimize_roundtrip_absolute_relative.py \
  --roundtrip-csv roundtrip.csv \
  --split-index 1500 \
  --abs-position-only
```

絶対拘束をすべて無効にする場合：

```bash
python3 optimize_roundtrip_absolute_relative.py \
  --roundtrip-csv roundtrip.csv \
  --split-index 1500 \
  --no-abs
```

仮想ループ拘束も追加する場合：

```bash
python3 optimize_roundtrip_absolute_relative.py \
  --roundtrip-csv roundtrip.csv \
  --split-index 1500 \
  --use-loop
```

## Main Options

### Common Options

| オプション | デフォルト | 内容 |
|---|---:|---|
| `--node-step` | `5` | 最適化に使うノードの間引き間隔 |
| `--sigma-fwd-pos` | `0.05` | 往路相対位置拘束の標準偏差 [m] |
| `--sigma-fwd-rot-deg` | `1.0` | 往路相対姿勢拘束の標準偏差 [deg] |
| `--sigma-rev-pos` | `0.20` | 復路相対位置拘束の標準偏差 [m] |
| `--sigma-rev-rot-deg` | `3.0` | 復路相対姿勢拘束の標準偏差 [deg] |
| `--use-loop` | 無効 | 仮想ループ拘束を追加 |
| `--sigma-loop-pos` | `0.50` | 仮想ループ位置拘束の標準偏差 [m] |
| `--sigma-loop-rot-deg` | `10.0` | 仮想ループ姿勢拘束の標準偏差 [deg] |
| `--max-nfev` | `100` | `least_squares`の最大関数評価回数 |
| `--output-prefix` | スクリプトごとに異なる | 出力ファイル名の接頭辞 |

### `optimize_forward_reverse_relative.py`

| オプション | 内容 |
|---|---|
| `--forward-csv` | 往路軌跡CSV |
| `--reverse-csv` | 仮想復路軌跡CSV |
| `--reverse-time-mode invert` | 復路の時間軸を反転し、往路の時間範囲へ対応付ける |
| `--reverse-time-mode as_is` | 復路CSVの時刻をそのまま使用する |

### `optimize_roundtrip_absolute_relative.py`

| オプション | デフォルト | 内容 |
|---|---:|---|
| `--roundtrip-csv` | 必須 | 往路と仮想復路を含む往復軌跡CSV |
| `--split-time` | 未指定 | 往路と復路を分割する時刻 |
| `--split-index` | 未指定 | 往路と復路を分割する行番号 |
| `--no-abs` | 無効 | 対応ノード間の絶対拘束を無効化 |
| `--abs-position-only` | 無効 | 絶対位置拘束だけを使用 |
| `--sigma-abs-pos` | `0.30` | 絶対位置拘束の標準偏差 [m] |
| `--sigma-abs-rot-deg` | `10.0` | 絶対姿勢拘束の標準偏差 [deg] |

## Constraint Weights

各`--sigma-*`オプションは、対応する拘束の標準偏差として使用されます。

- 小さい値：拘束が強くなる
- 大きい値：拘束が弱くなる

デフォルトでは、往路の推定結果を仮想復路より信頼する設定になっています。

```text
forward position sigma = 0.05 m
reverse position sigma = 0.20 m

forward rotation sigma = 1.0 deg
reverse rotation sigma = 3.0 deg
```

絶対拘束や仮想ループ拘束を強くしすぎると、局所的な軌跡形状が不自然に変形する可能性があります。最適化後の誤差CSVと軌跡を確認しながら調整してください。

## Optimization

最適化には、SciPyの`scipy.optimize.least_squares`を使用します。

主な設定は次のとおりです。

```python
least_squares(
    residual_func,
    x0,
    loss="huber",
    f_scale=1.0,
    max_nfev=args.max_nfev,
    verbose=2
)
```

外れ値の影響を抑えるため、Huber損失を使用します。

最初のノード \(X_0\) は固定され、2番目以降のノード位置と姿勢が最適化変数になります。姿勢は回転ベクトルで最適化されます。

## Time Correspondence

### Separate Trajectories

`invert`モードでは、仮想復路CSVの先頭が往路終端、仮想復路CSVの終端が往路始点に対応すると仮定し、時刻を線形反転します。

\[
t_{\mathrm{orig}}
=
t_{\mathrm{fwd,min}}
+
(t_{\mathrm{rev,max}}-t_{\mathrm{rev}})
\frac{
t_{\mathrm{fwd,max}}-t_{\mathrm{fwd,min}}
}{
t_{\mathrm{rev,max}}-t_{\mathrm{rev,min}}
}
\]

### Connected Round Trip

往復CSVでは、復路区間の時刻範囲を往路区間の時刻範囲へ線形反転して対応付けます。

その後、復路の位置を線形補間し、姿勢をSLERP補間して、往路の各時刻に対応する仮想復路姿勢を生成します。

## Output Files

### `optimize_forward_reverse_relative.py`

`--output-prefix results/relative`を指定した場合：

```text
results/relative_optimized_forward.csv
results/relative_optimized_nodes.csv
results/relative_relative_errors.csv
results/relative_comparison.csv
```

| ファイル | 内容 |
|---|---|
| `*_optimized_forward.csv` | 全往路時刻に補間した最適化後軌跡 |
| `*_optimized_nodes.csv` | 最適化に使用したノードの位置姿勢 |
| `*_relative_errors.csv` | 最適化後の相対変化と往路・復路拘束との差 |
| `*_comparison.csv` | 最適化前後の往路位置と位置変化量 |

### `optimize_roundtrip_absolute_relative.py`

`--output-prefix results/roundtrip`を指定した場合：

```text
results/roundtrip_optimized_forward.csv
results/roundtrip_optimized_nodes.csv
results/roundtrip_relative_errors.csv
results/roundtrip_absolute_errors.csv
results/roundtrip_comparison.csv
```

| ファイル | 内容 |
|---|---|
| `*_optimized_forward.csv` | 全往路時刻に補間した最適化後軌跡 |
| `*_optimized_nodes.csv` | 最適化に使用したノード |
| `*_relative_errors.csv` | 最適化後相対変化と往路・復路相対拘束との差 |
| `*_absolute_errors.csv` | 最適化ノードと対応復路ノードの絶対位置姿勢差 |
| `*_comparison.csv` | 往路、対応復路、最適化後位置の比較 |

## Output CSV Columns

### Optimized Trajectory

```text
timestamp
p_w_b_x
p_w_b_y
p_w_b_z
q_w_b_x
q_w_b_y
q_w_b_z
q_w_b_w
```

### Relative Error

```text
timestamp_i
timestamp_j
opt_vs_fwd_rel_pos_error
opt_vs_rev_rel_pos_error
opt_vs_fwd_rel_rot_error_deg
opt_vs_rev_rel_rot_error_deg
```

### Absolute Error

非分割型でのみ出力されます。

```text
timestamp
opt_vs_rev_abs_pos_error
opt_vs_rev_abs_rot_error_deg
```

## Recommended Workflow

### Case A: 往路と仮想復路を別々にSLAMした場合

1. 元rosbagをSLAMし、往路軌跡CSVを作成する
2. reverse-only rosbagをSLAMし、仮想復路軌跡CSVを作成する
3. `optimize_forward_reverse_relative.py`を実行する
4. `*_relative_errors.csv`で拘束間の差を確認する
5. Ground Truthがある場合は、別の評価プログラムで最適化前後を比較する

### Case B: 往路と仮想復路を接続してSLAMした場合

1. 往路と仮想復路を連続させたrosbagを作成する
2. 一つのSLAM実行から往復軌跡CSVを作成する
3. 往路と復路の境界時刻または行番号を確認する
4. `optimize_roundtrip_absolute_relative.py`を実行する
5. `*_relative_errors.csv`と`*_absolute_errors.csv`を確認する

## Example Directory Structure

```text
.
├── README.md
├── optimize_forward_reverse_relative.py
├── optimize_roundtrip_absolute_relative.py
├── data
│   ├── forward.csv
│   ├── reverse.csv
│   └── roundtrip.csv
└── results
```

実行例：

```bash
mkdir -p results

python3 optimize_forward_reverse_relative.py \
  --forward-csv data/forward.csv \
  --reverse-csv data/reverse.csv \
  --output-prefix results/relative
```

```bash
python3 optimize_roundtrip_absolute_relative.py \
  --roundtrip-csv data/roundtrip.csv \
  --split-index 1500 \
  --output-prefix results/roundtrip
```

## Notes

- 入力軌跡の姿勢は、`[x, y, z, w]`順のクォータニオンとして扱われます。
- 姿勢はbody座標系からworld座標系への回転を想定しています。
- 時刻はスクリプト内部で単位変換されません。すべての入力CSVで同じ時間単位を使用してください。
- 往路と復路の共通時間範囲だけが最適化に使用されます。
- SLERPのため、時刻は厳密な単調増加である必要があります。補間前に重複時刻は除去されます。
- `--node-step`を小さくすると細かい補正が可能ですが、計算量が増加します。
- 出力先ディレクトリは自動作成されません。`--output-prefix`にディレクトリを含める場合は、事前に作成してください。
- `--use-loop`は追加拘束です。常に精度が向上するとは限らないため、結果を比較して使用してください。
- 分割型では、往路と復路の絶対座標を直接比較しません。
- 非分割型の絶対拘束は、往路と復路が同一座標系で推定されている場合にのみ有効です。

## Troubleshooting

### `CSVに必要な列がありません`

入力CSVのヘッダを確認してください。

```bash
head -n 1 forward.csv
```

必要な列：

```text
timestamp,p_w_b_x,p_w_b_y,p_w_b_z,q_w_b_x,q_w_b_y,q_w_b_z,q_w_b_w
```

### `forward/reverseの共通時刻範囲が不足しています`

- 往路と復路の時刻範囲を確認してください。
- reverse-only CSVでは`--reverse-time-mode invert`を試してください。
- すでに元時刻へ変換済みの場合は`--reverse-time-mode as_is`を使用してください。

### `補間対象時刻が軌跡の時刻範囲外です`

時刻対応後の復路軌跡が、往路の評価時刻範囲を完全に覆っていません。入力CSVの先頭・終端時刻、データ欠損、往復区間の分割位置を確認してください。

### 最適化に時間がかかる

`--node-step`を大きくしてください。

```bash
--node-step 10
```

または、最大評価回数を減らします。

```bash
--max-nfev 50
```

### 軌跡が大きく変形する

拘束が強すぎる可能性があります。対応する標準偏差を大きくしてください。

例：

```bash
--sigma-rev-pos 0.50
--sigma-rev-rot-deg 5.0
--sigma-abs-pos 0.50
--sigma-abs-rot-deg 15.0
```

## License

公開時には、利用条件に応じてライセンスを設定してください。

例：

```text
MIT License
```
