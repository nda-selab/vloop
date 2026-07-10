#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
forward trajectory と virtual reverse trajectory を別々に推定したCSVを用いて、
forward trajectory のみをオフライン最適化するスクリプト。

入力CSVの列:
timestamp,
p_w_b_x,p_w_b_y,p_w_b_z,
q_w_b_x,q_w_b_y,q_w_b_z,q_w_b_w

想定:
- forward CSV は元rosbagをSLAMして得た /Odometry
- reverse CSV は reverse-only rosbagをSLAMして得た /Odometry
- forward と reverse は別々に推定されるため、絶対座標系は一致しない可能性が高い
- そのため、reverse軌跡は絶対位置姿勢拘束ではなく、相対位置姿勢拘束として用いる

目的関数:
    forward relative-pose constraint
  + reverse relative-pose constraint
  + optional virtual loop constraint
"""

import argparse
import numpy as np
import pandas as pd

from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.optimize import least_squares


REQUIRED_COLUMNS = [
    "timestamp",
    "p_w_b_x", "p_w_b_y", "p_w_b_z",
    "q_w_b_x", "q_w_b_y", "q_w_b_z", "q_w_b_w"
]


def normalize_quaternions(q):
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q, axis=1, keepdims=True)
    n[n < 1e-12] = 1.0
    return q / n


def load_trajectory_csv(path):
    df = pd.read_csv(path)

    for c in REQUIRED_COLUMNS:
        if c not in df.columns:
            raise ValueError(f"CSVに必要な列がありません: {c}")

    df = df[REQUIRED_COLUMNS].copy()
    df = df.dropna()
    df = df.sort_values("timestamp").reset_index(drop=True)

    q = df[["q_w_b_x", "q_w_b_y", "q_w_b_z", "q_w_b_w"]].to_numpy()
    q = normalize_quaternions(q)

    df[["q_w_b_x", "q_w_b_y", "q_w_b_z", "q_w_b_w"]] = q

    return df

def add_reverse_orig_timestamp(reverse_df, forward_df, mode):
    """
    reverse-only bag の /Odometry timestamp を forward の元時間軸へ対応づける。

    mode:
        invert:
            reverse-only bag は、元の終端側から開始するため、
            timestampを反転して元時間軸に戻す。
        as_is:
            すでに元時間軸に対応するtimestampがCSVに入っている場合。
    """
    rev = reverse_df.copy()

    f_min = forward_df["timestamp"].min()
    f_max = forward_df["timestamp"].max()

    r_min = rev["timestamp"].min()
    r_max = rev["timestamp"].max()

    if mode == "invert":
        if abs(r_max - r_min) < 1e-9:
            raise RuntimeError("reverse CSVのtimestamp範囲が小さすぎます。")

        scale = (f_max - f_min) / (r_max - r_min)

        # reverse-onlyの先頭は元の終端に対応するため、時間を反転する
        rev["orig_timestamp"] = f_min + (r_max - rev["timestamp"]) * scale

    elif mode == "as_is":
        rev["orig_timestamp"] = rev["timestamp"]

    else:
        raise ValueError(f"未知の reverse time mode: {mode}")

    rev = rev.sort_values("orig_timestamp").reset_index(drop=True)

    return rev


def remove_duplicate_times(t, p, q):
    """
    Slerpは時刻が厳密に単調増加である必要があるため、重複時刻を除去する。
    """
    t = np.asarray(t)
    p = np.asarray(p)
    q = np.asarray(q)

    keep = np.ones(len(t), dtype=bool)
    keep[1:] = np.diff(t) > 1e-9

    return t[keep], p[keep], q[keep]


def interpolate_pose(query_t, src_t, src_p, src_q):
    """
    src_t上の位置姿勢を query_t に補間する。
    位置は線形補間、姿勢はSLERP。
    """
    src_t, src_p, src_q = remove_duplicate_times(src_t, src_p, src_q)

    if query_t.min() < src_t.min() or query_t.max() > src_t.max():
        raise ValueError(
            "補間対象時刻が復路軌跡の時刻範囲外です。"
            f" query=[{query_t.min():.6f}, {query_t.max():.6f}],"
            f" src=[{src_t.min():.6f}, {src_t.max():.6f}]"
        )

    p_interp = np.zeros((len(query_t), 3), dtype=np.float64)

    for k in range(3):
        p_interp[:, k] = np.interp(query_t, src_t, src_p[:, k])

    rot_src = R.from_quat(src_q)
    slerp = Slerp(src_t, rot_src)
    rot_interp = slerp(query_t)

    q_interp = rot_interp.as_quat()

    return p_interp, q_interp


def relative_measurement(p_i, r_i, p_j, r_j):
    """
    位置姿勢 i から位置姿勢 j への相対位置姿勢を計算する。

    dp_ij は i のbody座標系で見た j への相対位置。
    dr_ij は i から j への相対回転。
    """
    dp_ij = r_i.inv().apply(p_j - p_i)
    dr_ij = r_i.inv() * r_j
    return dp_ij, dr_ij

def compute_relative_measurements(p, q):
    """
    軌跡から隣接ノード間の相対位置姿勢拘束を作る。
    """
    r = R.from_quat(q)

    dp_list = []
    dr_list = []

    for i in range(len(p) - 1):
        dp, dr = relative_measurement(
            p[i], r[i],
            p[i + 1], r[i + 1]
        )
        dp_list.append(dp)
        dr_list.append(dr)

    return dp_list, dr_list

def unpack_state(x, p0, r0):
    """
    X0は固定し、X1以降を最適化変数とする。
    1ノードあたり:
        position 3
        rotation vector 3
    """
    n_var = len(x) // 6
    n = n_var + 1

    p = np.zeros((n, 3), dtype=np.float64)
    rotvec = np.zeros((n, 3), dtype=np.float64)

    p[0] = p0
    rotvec[0] = r0.as_rotvec()

    data = x.reshape(-1, 6)
    p[1:] = data[:, 0:3]
    rotvec[1:] = data[:, 3:6]

    r = R.from_rotvec(rotvec)
    return p, r


def pack_initial_state(p, q):
    """
    X0を固定するため、X1以降だけを変数化する。
    """
    r = R.from_quat(q)
    rotvec = r.as_rotvec()

    data = np.hstack([p[1:], rotvec[1:]])

    return data.reshape(-1)


def relative_residual(
    p_est_i,
    r_est_i,
    p_est_j,
    r_est_j,
    meas_dp,
    meas_dr
):
    """
    推定相対姿勢と測定相対姿勢の誤差を計算する。
    """
    dp_est, dr_est = relative_measurement(
        p_est_i, r_est_i,
        p_est_j, r_est_j
    )

    e_p = dp_est - meas_dp
    e_r = (meas_dr.inv() * dr_est).as_rotvec()

    return e_p, e_r

def build_residual_function(
    p_init,
    q_init,
    fwd_dp,
    fwd_dr,
    rev_dp,
    rev_dr,
    sigma_fwd_pos,
    sigma_fwd_rot,
    sigma_rev_pos,
    sigma_rev_rot,
    use_loop,
    loop_dp,
    loop_dr,
    sigma_loop_pos,
    sigma_loop_rot
):
    """
    forward軌跡のみを最適化する残差関数を作る。

    拘束1:
        forward relative-pose constraint
    拘束2:
        reverse relative-pose constraint
    拘束3:
        optional virtual loop constraint
    """
    r_init = R.from_quat(q_init)

    p0 = p_init[0].copy()
    r0 = r_init[0]

    n = len(p_init)

    def residual(x):
        p_est, r_est = unpack_state(x, p0, r0)

        res = []

        # 1. forward relative-pose constraint
        for i in range(n - 1):
            e_p, e_r = relative_residual(
                p_est[i], r_est[i],
                p_est[i + 1], r_est[i + 1],
                fwd_dp[i], fwd_dr[i]
            )

            res.extend(e_p / sigma_fwd_pos)
            res.extend(e_r / sigma_fwd_rot)

        # 2. reverse relative-pose constraint
        for i in range(n - 1):
            e_p, e_r = relative_residual(
                p_est[i], r_est[i],
                p_est[i + 1], r_est[i + 1],
                rev_dp[i], rev_dr[i]
            )

            res.extend(e_p / sigma_rev_pos)
            res.extend(e_r / sigma_rev_rot)

        # 3. optional virtual loop constraint
        #    reverse trajectory が示す「終端から始点へ戻る相対変換」と、
        #    最適化中の forward trajectory の X_N^{-1} X_0 を対応させる。
        if use_loop:
            e_p, e_r = relative_residual(
                p_est[-1], r_est[-1],
                p_est[0], r_est[0],
                loop_dp, loop_dr
            )

            res.extend(e_p / sigma_loop_pos)
            res.extend(e_r / sigma_loop_rot)

        return np.asarray(res, dtype=np.float64)

    return residual


def calc_relative_constraint_stats(name, dp_a, dr_a, dp_b, dr_b):
    """
    2種類の相対拘束の差分統計を表示する。
    """
    trans_errors = []
    rot_errors = []

    for a_dp, a_dr, b_dp, b_dr in zip(dp_a, dr_a, dp_b, dr_b):
        trans_errors.append(np.linalg.norm(a_dp - b_dp))

        e_r = (b_dr.inv() * a_dr).as_rotvec()
        rot_errors.append(np.linalg.norm(e_r))

    trans_errors = np.asarray(trans_errors)
    rot_errors = np.asarray(rot_errors)

    print(f"[INFO] {name}")
    print(f"       relative trans diff mean: {np.mean(trans_errors):.6f} m")
    print(f"       relative trans diff rmse: {np.sqrt(np.mean(trans_errors ** 2)):.6f} m")
    print(f"       relative trans diff max : {np.max(trans_errors):.6f} m")
    print(f"       relative rot diff mean  : {np.rad2deg(np.mean(rot_errors)):.6f} deg")
    print(f"       relative rot diff rmse  : {np.rad2deg(np.sqrt(np.mean(rot_errors ** 2))):.6f} deg")
    print(f"       relative rot diff max   : {np.rad2deg(np.max(rot_errors)):.6f} deg")

def compute_relative_error_dataframe(
    t_nodes,
    p_opt,
    q_opt,
    fwd_dp,
    fwd_dr,
    rev_dp,
    rev_dr
):
    """
    最適化後の相対移動が forward/reverse 相対拘束とどれだけずれているかをCSV化する。
    """
    r_opt = R.from_quat(q_opt)

    rows = []

    for i in range(len(t_nodes) - 1):
        dp_opt, dr_opt = relative_measurement(
            p_opt[i], r_opt[i],
            p_opt[i + 1], r_opt[i + 1]
        )

        e_fwd_p = np.linalg.norm(dp_opt - fwd_dp[i])
        e_rev_p = np.linalg.norm(dp_opt - rev_dp[i])

        e_fwd_r = np.linalg.norm((fwd_dr[i].inv() * dr_opt).as_rotvec())
        e_rev_r = np.linalg.norm((rev_dr[i].inv() * dr_opt).as_rotvec())

        rows.append({
            "timestamp_i": t_nodes[i],
            "timestamp_j": t_nodes[i + 1],
            "opt_vs_fwd_rel_pos_error": e_fwd_p,
            "opt_vs_rev_rel_pos_error": e_rev_p,
            "opt_vs_fwd_rel_rot_error_deg": np.rad2deg(e_fwd_r),
            "opt_vs_rev_rel_rot_error_deg": np.rad2deg(e_rev_r),
        })

    return pd.DataFrame(rows)


def calc_position_error_stats(p_a, p_b, name="error"):
    e = p_a - p_b
    norm = np.linalg.norm(e, axis=1)

    print(f"[INFO] {name} position error")
    print(f"       mean: {np.mean(norm):.6f} m")
    print(f"       rmse: {np.sqrt(np.mean(norm ** 2)):.6f} m")
    print(f"       max : {np.max(norm):.6f} m")


def main():
    parser = argparse.ArgumentParser(
        description=(
            "forward CSV と reverse-only CSV を用いて、"
            "relative-pose constraints により forward trajectory を最適化する。"
        )
    )

    parser.add_argument("--forward-csv", required=True, help="forward /Odometry CSV")
    parser.add_argument("--reverse-csv", required=True, help="reverse-only /Odometry CSV")
    parser.add_argument("--output-prefix", default="optimized_relative", help="出力CSVのprefix")

    parser.add_argument(
        "--reverse-time-mode",
        choices=["invert", "as_is"],
        default="invert",
        help=(
            "reverse CSVのtimestamp対応方法。"
            "reverse_only bagから得たCSVなら通常 invert。"
            "すでに元時間軸に変換済みなら as_is。"
        )
    )

    parser.add_argument(
        "--node-step",
        type=int,
        default=5,
        help="最適化に使うノード間引き間隔。1なら全点使用。大きいほど軽い。"
    )

    parser.add_argument(
        "--sigma-fwd-pos",
        type=float,
        default=0.05,
        help="forward相対位置拘束の標準偏差[m]。小さいほど強い。"
    )

    parser.add_argument(
        "--sigma-fwd-rot-deg",
        type=float,
        default=1.0,
        help="forward相対姿勢拘束の標準偏差[deg]。小さいほど強い。"
    )

    parser.add_argument(
        "--sigma-rev-pos",
        type=float,
        default=0.20,
        help="reverse相対位置拘束の標準偏差[m]。大きいほど弱い。"
    )

    parser.add_argument(
        "--sigma-rev-rot-deg",
        type=float,
        default=3.0,
        help="reverse相対姿勢拘束の標準偏差[deg]。大きいほど弱い。"
    )

    parser.add_argument(
        "--use-loop",
        action="store_true",
        help="reverse trajectory由来のvirtual loop constraintを追加する。"
    )

    parser.add_argument(
        "--sigma-loop-pos",
        type=float,
        default=0.50,
        help="virtual loop constraintの位置標準偏差[m]。大きいほど弱い。"
    )

    parser.add_argument(
        "--sigma-loop-rot-deg",
        type=float,
        default=10.0,
        help="virtual loop constraintの姿勢標準偏差[deg]。大きいほど弱い。"
    )

    parser.add_argument(
        "--max-nfev",
        type=int,
        default=100,
        help="least_squaresの最大反復回数。"
    )

    args = parser.parse_args()

    sigma_fwd_rot = np.deg2rad(args.sigma_fwd_rot_deg)
    sigma_rev_rot = np.deg2rad(args.sigma_rev_rot_deg)
    sigma_loop_rot = np.deg2rad(args.sigma_loop_rot_deg)

    print(f"[INFO] forward csv: {args.forward_csv}")
    print(f"[INFO] reverse csv: {args.reverse_csv}")
    print(f"[INFO] output prefix: {args.output_prefix}")
    print(f"[INFO] reverse time mode: {args.reverse_time_mode}")

    fwd_df = load_trajectory_csv(args.forward_csv)
    rev_raw_df = load_trajectory_csv(args.reverse_csv)
    rev_df = add_reverse_orig_timestamp(
        reverse_df=rev_raw_df,
        forward_df=fwd_df,
        mode=args.reverse_time_mode
    )

    fwd_df["orig_timestamp"] = fwd_df["timestamp"]

    print(f"[INFO] forward timestamp range: "
          f"{fwd_df['orig_timestamp'].min():.9f} -> {fwd_df['orig_timestamp'].max():.9f}")

    print(f"[INFO] reverse orig timestamp range: "
          f"{rev_df['orig_timestamp'].min():.9f} -> {rev_df['orig_timestamp'].max():.9f}")
    
    # 共通時刻範囲を使う
    overlap_min = max(
        fwd_df["orig_timestamp"].min(),
        rev_df["orig_timestamp"].min()
    )

    overlap_max = min(
        fwd_df["orig_timestamp"].max(),
        rev_df["orig_timestamp"].max()
    )

    fwd_df = fwd_df[
        (fwd_df["orig_timestamp"] >= overlap_min) &
        (fwd_df["orig_timestamp"] <= overlap_max)
    ].reset_index(drop=True)

    rev_df = rev_df[
        (rev_df["orig_timestamp"] >= overlap_min) &
        (rev_df["orig_timestamp"] <= overlap_max)
    ].reset_index(drop=True)

    if len(fwd_df) < 2 or len(rev_df) < 2:
        raise RuntimeError("forward/reverseの共通時刻範囲が不足しています。")

    print(f"[INFO] overlap: {overlap_min:.9f} -> {overlap_max:.9f}")
    print(f"[INFO] forward overlap samples: {len(fwd_df)}")
    print(f"[INFO] reverse overlap samples: {len(rev_df)}")

    # forwardの全時刻
    t_fwd_all = fwd_df["orig_timestamp"].to_numpy()

    p_fwd_all = fwd_df[["p_w_b_x", "p_w_b_y", "p_w_b_z"]].to_numpy()
    q_fwd_all = fwd_df[["q_w_b_x", "q_w_b_y", "q_w_b_z", "q_w_b_w"]].to_numpy()

    # reverseをforward時刻へ補間する
    t_rev = rev_df["orig_timestamp"].to_numpy()
    p_rev = rev_df[["p_w_b_x", "p_w_b_y", "p_w_b_z"]].to_numpy()
    q_rev = rev_df[["q_w_b_x", "q_w_b_y", "q_w_b_z", "q_w_b_w"]].to_numpy()

    p_rev_all, q_rev_all = interpolate_pose(
        query_t=t_fwd_all,
        src_t=t_rev,
        src_p=p_rev,
        src_q=q_rev
    )

    # ノード間引き
    step = max(1, args.node_step)
    idx_nodes = np.arange(0, len(fwd_df), step)

    if idx_nodes[-1] != len(fwd_df) - 1:
        idx_nodes = np.append(idx_nodes, len(fwd_df) - 1)

    t_nodes = t_fwd_all[idx_nodes]

    p_fwd_nodes = p_fwd_all[idx_nodes]
    q_fwd_nodes = q_fwd_all[idx_nodes]

    p_rev_nodes = p_rev_all[idx_nodes]
    q_rev_nodes = q_rev_all[idx_nodes]

    print(f"[INFO] optimization nodes: {len(idx_nodes)}")
    print(f"[INFO] node step: {step}")

    # forward / reverse の相対拘束を作成
    fwd_dp, fwd_dr = compute_relative_measurements(
        p_fwd_nodes,
        q_fwd_nodes
    )

    rev_dp, rev_dr = compute_relative_measurements(
        p_rev_nodes,
        q_rev_nodes
    )

    calc_relative_constraint_stats(
        "before optimization forward-vs-reverse relative constraint difference",
        fwd_dp, fwd_dr,
        rev_dp, rev_dr
    )

    # virtual loop constraint
    loop_dp = None
    loop_dr = None

    if args.use_loop:
        r_rev_nodes = R.from_quat(q_rev_nodes)

        # reverse trajectory が示す「forward終端からforward始点へ戻る相対変換」
        loop_dp, loop_dr = relative_measurement(
            p_rev_nodes[-1], r_rev_nodes[-1],
            p_rev_nodes[0], r_rev_nodes[0]
        )

        print("[INFO] use virtual loop constraint: enabled")
    else:
        print("[INFO] use virtual loop constraint: disabled")

    # 初期値
    x0 = pack_initial_state(p_fwd_nodes, q_fwd_nodes)

    residual_func = build_residual_function(
        p_init=p_fwd_nodes,
        q_init=q_fwd_nodes,
        fwd_dp=fwd_dp,
        fwd_dr=fwd_dr,
        rev_dp=rev_dp,
        rev_dr=rev_dr,
        sigma_fwd_pos=args.sigma_fwd_pos,
        sigma_fwd_rot=sigma_fwd_rot,
        sigma_rev_pos=args.sigma_rev_pos,
        sigma_rev_rot=sigma_rev_rot,
        use_loop=args.use_loop,
        loop_dp=loop_dp,
        loop_dr=loop_dr,
        sigma_loop_pos=args.sigma_loop_pos,
        sigma_loop_rot=sigma_loop_rot
    )

    print("[INFO] Start optimization...")

    result = least_squares(
        residual_func,
        x0,
        loss="huber",
        f_scale=1.0,
        max_nfev=args.max_nfev,
        verbose=2
    )

    print("[INFO] Optimization finished.")
    print(f"[INFO] success: {result.success}")
    print(f"[INFO] message: {result.message}")
    print(f"[INFO] cost: {result.cost:.6f}")

    r0 = R.from_quat(q_fwd_nodes)[0]
    p_opt_nodes, r_opt_nodes = unpack_state(
        result.x,
        p_fwd_nodes[0],
        r0
    )
    q_opt_nodes = r_opt_nodes.as_quat()

    opt_dp, opt_dr = compute_relative_measurements(
        p_opt_nodes,
        q_opt_nodes
    )

    calc_relative_constraint_stats(
        "after optimization optimized-vs-forward relative constraint difference",
        opt_dp, opt_dr,
        fwd_dp, fwd_dr
    )

    calc_relative_constraint_stats(
        "after optimization optimized-vs-reverse relative constraint difference",
        opt_dp, opt_dr,
        rev_dp, rev_dr
    )

    # 最適化ノードから全forward時刻へ補間
    p_opt_all, q_opt_all = interpolate_pose(
        query_t=t_fwd_all,
        src_t=t_nodes,
        src_p=p_opt_nodes,
        src_q=q_opt_nodes
    )

    # 補正後forward軌跡
    optimized = pd.DataFrame({
        "timestamp": t_fwd_all,
        "p_w_b_x": p_opt_all[:, 0],
        "p_w_b_y": p_opt_all[:, 1],
        "p_w_b_z": p_opt_all[:, 2],
        "q_w_b_x": q_opt_all[:, 0],
        "q_w_b_y": q_opt_all[:, 1],
        "q_w_b_z": q_opt_all[:, 2],
        "q_w_b_w": q_opt_all[:, 3],
    })

    optimized_path = f"{args.output_prefix}_optimized_forward.csv"
    optimized.to_csv(optimized_path, index=False)

     # 最適化ノード
    nodes = pd.DataFrame({
        "timestamp": t_nodes,
        "p_w_b_x": p_opt_nodes[:, 0],
        "p_w_b_y": p_opt_nodes[:, 1],
        "p_w_b_z": p_opt_nodes[:, 2],
        "q_w_b_x": q_opt_nodes[:, 0],
        "q_w_b_y": q_opt_nodes[:, 1],
        "q_w_b_z": q_opt_nodes[:, 2],
        "q_w_b_w": q_opt_nodes[:, 3],
    })

    nodes_path = f"{args.output_prefix}_optimized_nodes.csv"
    nodes.to_csv(nodes_path, index=False)

    # 相対拘束誤差CSV
    rel_error_df = compute_relative_error_dataframe(
        t_nodes=t_nodes,
        p_opt=p_opt_nodes,
        q_opt=q_opt_nodes,
        fwd_dp=fwd_dp,
        fwd_dr=fwd_dr,
        rev_dp=rev_dp,
        rev_dr=rev_dr
    )

    rel_error_path = f"{args.output_prefix}_relative_errors.csv"
    rel_error_df.to_csv(rel_error_path, index=False)

    # 比較用: forward と optimized の絶対位置のみ
    # reverseの絶対位置は別座標系の可能性が高いため比較対象に含めない
    comparison = pd.DataFrame({
        "timestamp": t_fwd_all,
        "p_fwd_x": p_fwd_all[:, 0],
        "p_fwd_y": p_fwd_all[:, 1],
        "p_fwd_z": p_fwd_all[:, 2],
        "p_opt_x": p_opt_all[:, 0],
        "p_opt_y": p_opt_all[:, 1],
        "p_opt_z": p_opt_all[:, 2],
    })

    comparison["fwd_opt_position_change"] = np.linalg.norm(
        comparison[["p_opt_x", "p_opt_y", "p_opt_z"]].to_numpy()
        - comparison[["p_fwd_x", "p_fwd_y", "p_fwd_z"]].to_numpy(),
        axis=1
    )

    comparison_path = f"{args.output_prefix}_comparison.csv"
    comparison.to_csv(comparison_path, index=False)

    print("[INFO] Saved:")
    print(f"       {optimized_path}")
    print(f"       {nodes_path}")
    print(f"       {rel_error_path}")
    print(f"       {comparison_path}")


if __name__ == "__main__":
    main()