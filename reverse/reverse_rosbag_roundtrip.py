#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import copy
import os
import rosbag
from rospy.rostime import Time

def update_header_stamp(msg, new_t):
    """
    msg.header.stamp を new_t に更新する。
    TFMessage 内部の transform.header.stamp も更新する。

    /tf_static を更新したくない場合は、呼び出し側で除外する。
    """
    try:
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):      
            msg.header.stamp = new_t

        if hasattr(msg, 'transforms'):
            for tr in msg.transforms:
                if hasattr(tr, 'header') and hasattr(tr.header, 'stamp'):
                    tr.header.stamp = new_t

    except Exception:
        pass

def update_velodyne_scan_stamp_preserve_order(msg, new_t):
    """
    /velodyne_packets 用

    スキャン内部のpacket順序は変えない
    packetごとの相対時刻も元スキャンと同じにする
    ただし，絶対時刻だけ new_t に付け替える
    """
    try:
        if not hasattr(msg, "header") or not hasattr(msg.header, "stamp"):
            return msg

        old_header_t = msg.header.stamp

        msg.header.stamp = new_t

        if not hasattr(msg, "packets"):
            return msg

        for pkt in msg.packets:
            if hasattr(pkt, "stamp"):
                offset = (pkt.stamp - old_header_t).to_sec()
                pkt.stamp = Time.from_sec(new_t.to_sec() + offset)

    except Exception as e:
        print(f"[WARN] Failed to update VelodyneScan packet stamps: {e}")

    return msg

def reverse_imu_msg(msg, mode):
    """
    復路用にIMU値を変換する。

    none        : 反転なし
    gyro        : 角速度のみ反転
    gyro_acc    : 角速度と加速度xyzを反転
    gyro_acc_xy : 角速度xyzと加速度xyのみ反転
    """
    if mode in ["gyro", "gyro_acc", "gyro_acc_xy"]:
        msg.angular_velocity.x *= -1.0
        msg.angular_velocity.y *= -1.0
        msg.angular_velocity.z *= -1.0

    if mode == "gyro_acc":
        msg.linear_acceleration.x *= -1.0
        msg.linear_acceleration.y *= -1.0
        msg.linear_acceleration.z *= -1.0

    if mode == "gyro_acc_xy":
        msg.linear_acceleration.x *= -1.0
        msg.linear_acceleration.y *= -1.0
        # zは重力成分を含む可能性が高いため反転しない

    return msg

def open_outbag(path, compression):
    if compression == "lz4":
        try:
            return rosbag.Bag(path, "w", compression="lz4")
        except Exception as e:
            print(f"[WARN] LZ4 unavailable: {e}")
            print("[WARN] Falling back to no compression.")
            return rosbag.Bag(path, "w")
        
    if compression == "bz2":
        return rosbag.Bag(path, "w", compression="bz2")
    
    return rosbag.Bag(path, "w")

def make_roundtrip_header_time(topic, msg, topic_end_header_time, fallback_t):
    """
    round-trip rosbag の復路用 header.stamp を作る。

    h_new = h_end_topic + (h_end_topic - h_old)

    これにより、往路の各トピックの最後のheader時刻から、
    復路のheader時刻が単調増加するように作られる。
    """
    if (
        topic in topic_end_header_time and
        hasattr(msg, "header") and
        hasattr(msg.header, "stamp")
    ):
        old_h = msg.header.stamp
        end_h = topic_end_header_time[topic]
        return Time.from_sec(end_h.to_sec() + (end_h - old_h).to_sec())

    return fallback_t

def make_reverse_only_header_time(
    topic,
    msg,
    topic_start_header_time,
    topic_end_header_time,
    fallback_t
):
    """
    reverse-only rosbag の header.stamp を作る。

    h_new = h_start_topic + (h_end_topic - h_old)

    元トピックの最後のメッセージが、reverse-only bag の先頭側に来る。
    header.stamp は元の選択区間の開始時刻付近から単調増加する。
    """
    if (
        topic in topic_start_header_time and
        topic in topic_end_header_time and
        hasattr(msg, "header") and
        hasattr(msg.header, "stamp")
    ):
        old_h = msg.header.stamp
        start_h = topic_start_header_time[topic]
        end_h = topic_end_header_time[topic]
        return Time.from_sec(start_h.to_sec() + (end_h - old_h).to_sec())

    return fallback_t

def update_message_stamp_by_topic(topic, msg, new_header_t):
    """
    topicに応じて message内部のstampを更新する。
    """
    if topic == "/velodyne_packets":
        return update_velodyne_scan_stamp_preserve_order(msg, new_header_t)

    update_header_stamp(msg, new_header_t)
    return msg
    
def read_selected_messages(args):
    """
    入力bagから指定区間・指定トピックのメッセージを読み込む。
    """
    skip_topics =set(args.skip_topics)

    messages = []
    bag_first_time = None
    t_start = None
    t_end = None
    start_time = None
    end_time_limit = None

    with rosbag.Bag(args.input, 'r') as inbag:
        for topic, msg, t in inbag.read_messages():
            if topic in skip_topics:
                continue
            
            # skip後の最初の有効メッセージをbag基準時刻とする
            if bag_first_time is None:
                bag_first_time = t

                start_time = Time.from_sec(
                    bag_first_time.to_sec() + max(0.0, args.start_offset)
                )

                if args.end_offset is not None:
                    if args.end_offset <= args.start_offset:
                        raise RuntimeError(
                            "--end-offset は --start-offset より大きい値にしてください。"
                        )

                    end_time_limit = Time.from_sec(
                        bag_first_time.to_sec() + args.end_offset
                    )

            # 指定開始時刻より前は使わない
            if start_time is not None and t < start_time:
                continue

            # 指定終了時刻を超えたら終了
            if end_time_limit is not None and t > end_time_limit:
                break

            if t_start is None:
                t_start = t

            t_end = t # 最後の時刻を更新

            messages.append((topic, msg, t))

    if not messages:
        raise RuntimeError("有効なメッセージがありません。skip-topicsやoffsetを確認してください。")

    return messages, bag_first_time, t_start, t_end

def analyze_topic_times(messages):
    """
    各トピックの最初・最後のheader.stampと、
    各トピックの最後のindexを記録する。
    """
    topic_start_header_time = {}
    topic_end_header_time = {}
    topic_last_index = {}

    for i, (topic, msg, t) in enumerate(messages):
        topic_last_index[topic] = i

        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            if topic not in topic_start_header_time:
                topic_start_header_time[topic] = msg.header.stamp

            topic_end_header_time[topic] = msg.header.stamp

    return topic_start_header_time, topic_end_header_time, topic_last_index

def write_static_topics_once(outbag, messages, static_topics, write_time):
    """
    /tf_static などの静的トピックを1回だけ書き込む。
    message内部のstampは基本的に保持する。
    """
    written = 0
    for topic, msg, t in messages:
        if topic not in static_topics:
            continue

        out_msg = copy.deepcopy(msg)
        outbag.write(topic, out_msg, write_time)
        written += 1

    return written

def write_forward_part(outbag, messages):
    """
    round-trip rosbag の往路を書き込む。
    bag時刻・header.stampともに元のまま保持する。
    """
    count = 0

    for topic, msg, t in messages:
        out_msg = copy.deepcopy(msg)
        outbag.write(topic, out_msg, t)
        count += 1

    return count

def write_reverse_part_for_roundtrip(
    outbag,
    messages,
    t_end,
    topic_end_header_time,
    topic_last_index,
    imu_topic,
    reverse_imu,
    static_topics
):
    """
    round-trip rosbag の復路を書き込む。

    往路の後ろに仮想復路を接続する。
    各トピックの最後のメッセージは、往路終端との重複を避けるため除外する。
    """
    count = 0

    for i, (topic, msg, t) in reversed(list(enumerate(messages))):

        # /tf_static などは復路側には再度書かない
        if topic in static_topics:
            continue

        # 各トピックの最後のメッセージを復路で重複させない
        if i == topic_last_index[topic]:
            continue

        out_msg = copy.deepcopy(msg)

        # round-trip の復路 bag 時刻
        # 往路終了 t_end の後ろに接続する
        new_bag_sec = t_end.to_sec() + (t_end - t).to_sec()
        new_bag_t = Time.from_sec(new_bag_sec)

        # round-trip の復路 header.stamp
        new_header_t = make_roundtrip_header_time(
            topic,
            out_msg,
            topic_end_header_time,
            new_bag_t
        )

        if topic == imu_topic:
            out_msg = reverse_imu_msg(out_msg, reverse_imu)

        out_msg = update_message_stamp_by_topic(topic, out_msg, new_header_t)

        outbag.write(topic, out_msg, new_bag_t)
        count += 1

    return count

def write_reverse_only_bag(
    outbag,
    messages,
    t_start,
    t_end,
    topic_start_header_time,
    topic_end_header_time,
    imu_topic,
    reverse_imu,
    static_topics
):
    """
    reverse-only rosbag を書き込む。

    出力bagには仮想復路のみを含める。
    元の最後のメッセージから再生され、bag時刻は t_start から単調増加する。

    new_bag_t = t_start + (t_end - t_old)
    """
    count = 0

    # 静的TFなどは先頭に1回書く
    static_count = write_static_topics_once(outbag, messages, static_topics, t_start)
    count += static_count

    for topic, msg, t in reversed(messages):

        # /tf_static などは上で1回だけ書いたのでスキップ
        if topic in static_topics:
            continue

        out_msg = copy.deepcopy(msg)

        # reverse-only の bag 時刻
        # 元の t_end が出力bagの t_start に対応する
        new_bag_sec = t_start.to_sec() + (t_end - t).to_sec()
        new_bag_t = Time.from_sec(new_bag_sec)

        # reverse-only の header.stamp
        # 各トピックのheader時刻も選択区間の開始側から単調増加するようにする
        new_header_t = make_reverse_only_header_time(
            topic,
            out_msg,
            topic_start_header_time,
            topic_end_header_time,
            new_bag_t
        )

        if topic == imu_topic:
            out_msg = reverse_imu_msg(out_msg, reverse_imu)

        out_msg = update_message_stamp_by_topic(topic, out_msg, new_header_t)

        outbag.write(topic, out_msg, new_bag_t)
        count += 1

    return count

def derive_output_paths(args):
    """
    出力ファイル名を決定する。
    """
    base, ext = os.path.splitext(args.input)
    if ext == "":
        ext = ".bag"

    roundtrip_path = args.output_roundtrip
    reverse_only_path = args.output_reverse_only

    if args.mode == "roundtrip":
        if args.output is not None:
            roundtrip_path = args.output
        elif roundtrip_path is None:
            roundtrip_path = base + "_roundtrip.bag"

    elif args.mode == "reverse_only":
        if args.output is not None:
            reverse_only_path = args.output
        elif reverse_only_path is None:
            reverse_only_path = base + "_reverse_only.bag"

    elif args.mode == "both":
        if roundtrip_path is None:
            roundtrip_path = base + "_roundtrip.bag"
        if reverse_only_path is None:
            reverse_only_path = base + "_reverse_only.bag"

    return roundtrip_path, reverse_only_path

def create_roundtrip_bag(
    output_path,
    messages,
    t_end,
    topic_end_header_time,
    topic_last_index,
    args
):
    print(f"[INFO] Creating round-trip rosbag: {output_path}")

    static_topics = set(args.static_topics)

    with open_outbag(output_path, args.compression) as outbag:
        print("[INFO] Writing forward part...")
        forward_count = write_forward_part(
            outbag,
            messages
        )

        print("[INFO] Writing virtual reverse part for round-trip...")
        reverse_count = write_reverse_part_for_roundtrip(
            outbag=outbag,
            messages=messages,
            t_end=t_end,
            topic_end_header_time=topic_end_header_time,
            topic_last_index=topic_last_index,
            imu_topic=args.imu_topic,
            reverse_imu=args.reverse_imu,
            static_topics=static_topics
        )

    print("[INFO] Round-trip rosbag done.")
    print(f"[INFO] Forward messages: {forward_count}")
    print(f"[INFO] Reverse messages: {reverse_count}")
    print(f"[INFO] Total messages  : {forward_count + reverse_count}")

def create_reverse_only_bag(
    output_path,
    messages,
    t_start,
    t_end,
    topic_start_header_time,
    topic_end_header_time,
    args
):
    print(f"[INFO] Creating reverse-only rosbag: {output_path}")

    static_topics = set(args.static_topics)

    with open_outbag(output_path, args.compression) as outbag:
        print("[INFO] Writing reverse-only part...")
        reverse_count = write_reverse_only_bag(
            outbag=outbag,
            messages=messages,
            t_start=t_start,
            t_end=t_end,
            topic_start_header_time=topic_start_header_time,
            topic_end_header_time=topic_end_header_time,
            imu_topic=args.imu_topic,
            reverse_imu=args.reverse_imu,
            static_topics=static_topics
        )
    
    print("[INFO] Reverse-only rosbag done.")
    print(f"[INFO] Reverse-only messages: {reverse_count}")

def main():
    parser = argparse.ArgumentParser(
        description="片道rosbagから、round-trip rosbag または reverse-only rosbag を作成する。"
    )

    parser.add_argument("--input",  required=True, help="入力bag")

    parser.add_argument(
        "--mode",
        choices=["roundtrip", "reverse_only", "both"],
        default="roundtrip",
        help="作成モード: roundtrip, reverse_only, both"
    )

    parser.add_argument(
        "--output",
        default=None,
        help="単一モード時の出力bag。roundtrip/reverse_onlyで使用"
    )

    parser.add_argument(
        "--output-roundtrip",
        default=None,
        help="round-trip rosbagの出力パス。mode=both時などに使用"
    )

    parser.add_argument(
        "--output-reverse-only",
        default=None,
        help="reverse_only rosbagの出力パス。mode=both時などに使用"
    )

    parser.add_argument(
        "--start-offset",
        type=float,
        default=0.0,
        help="入力bag開始時刻から何秒後を往路開始時刻にするか"
    )

    parser.add_argument(
        "--end-offset",
        type=float,
        default=None,
        help="入力bag開始時刻から何秒後を往路終了時刻にするか。未指定ならbag最後まで使用"
    )

    parser.add_argument(
        "--skip-topics",
        nargs="*",
        default=["/clock", "/rosout", "/rosout_agg"],
        help="除外するトピック"
    )

    parser.add_argument(
        "--static-topics",
        nargs="*",
        default=["/tf_static"],
        help="反転対象にしない静的トピック。reverse_onlyでは先頭に1回だけ書き込む"
    )

    parser.add_argument(
        "--imu-topic",
        default="/imu/data",
        help="IMUトピック名"
    )

    parser.add_argument(
        "--reverse-imu",
        choices=["none", "gyro", "gyro_acc", "gyro_acc_xy"],
        default="gyro",
        help="復路IMUの反転方法"
    )

    parser.add_argument(
        "--compression",
        choices=["none", "lz4", "bz2"],
        default="lz4",
        help="出力bagの圧縮方式"
    )
    
    args = parser.parse_args()

    print(f"[INFO] input : {args.input}")
    print(f"[INFO] mode  : {args.mode}")
    print(f"[INFO] skip topics  : {set(args.skip_topics)}")
    print(f"[INFO] static topics: {set(args.static_topics)}")
    print(f"[INFO] imu topic    : {args.imu_topic}")
    print(f"[INFO] reverse imu  : {args.reverse_imu}")
    print(f"[INFO] compression  : {args.compression}")

    messages, bag_first_time, t_start, t_end = read_selected_messages(args)

    (
        topic_start_header_time,
        topic_end_header_time,
        topic_last_index
    ) = analyze_topic_times(messages)

    duration = (t_end - t_start).to_sec()

    print(f"[INFO] bag first time: {bag_first_time.to_sec():.9f}")
    print(f"[INFO] selected start time: {t_start.to_sec():.9f}")
    print(f"[INFO] selected end time  : {t_end.to_sec():.9f}")
    print(f"[INFO] selected duration  : {duration:.9f} sec")
    print(f"[INFO] selected messages  : {len(messages)}")
    print(f"[INFO] requested start offset: {args.start_offset:.9f} sec")

    if args.end_offset is not None:
        print(f"[INFO] requested end offset  : {args.end_offset:.9f} sec")
    else:
        print("[INFO] requested end offset  : bag end")

    roundtrip_path, reverse_only_path = derive_output_paths(args)

    if args.mode in ["roundtrip", "both"]:
        create_roundtrip_bag(
            output_path=roundtrip_path,
            messages=messages,
            t_end=t_end,
            topic_end_header_time=topic_end_header_time,
            topic_last_index=topic_last_index,
            args=args
        )

    if args.mode in ["reverse_only", "both"]:
        create_reverse_only_bag(
            output_path=reverse_only_path,
            messages=messages,
            t_start=t_start,
            t_end=t_end,
            topic_start_header_time=topic_start_header_time,
            topic_end_header_time=topic_end_header_time,
            args=args
        )

    print("[INFO] All done.")

    if roundtrip_path is not None and args.mode in ["roundtrip", "both"]:
        print(f"[INFO] round-trip output : {roundtrip_path}")

    if reverse_only_path is not None and args.mode in ["reverse_only", "both"]:
        print(f"[INFO] reverse-only output: {reverse_only_path}")

if __name__ == "__main__":
    main()