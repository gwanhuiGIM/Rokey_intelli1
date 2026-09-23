#!/usr/bin/env python3
"""
ArUco 마커 생성 + ID 관리 툴 (소방점검 포인트용)

기능:
  1) 마커 PNG 생성 (단일 또는 범위) — 인쇄용 여백/라벨 포함
  2) 마커 ID <-> 점검 포인트 매핑 관리 (registry.json)
  3) 등록된 마커 목록 조회

사용 예:
  # ID 0~19 마커 20장 생성 (한 변 10cm, 300DPI 인쇄 기준)
  python3 aruco_marker_tool.py generate --ids 0-19 --size-cm 10 --out ./markers

  # 점검 포인트 등록
  python3 aruco_marker_tool.py register --id 42 \
      --name "3층 동측 소화전" --type extinguisher --zone Z3

  # 등록 목록 조회
  python3 aruco_marker_tool.py list
"""

import argparse
import json
import os

import cv2
import numpy as np

REGISTRY_FILE = "marker_registry.json"
DPI = 300  # 인쇄 기준 해상도
CM_PER_INCH = 2.54


def parse_ids(ids_str):
    """'0-19' 또는 '3,7,42' 형식 파싱"""
    ids = []
    for part in ids_str.split(","):
        part = part.strip()
        if "-" in part:
            start, end = part.split("-")
            ids.extend(range(int(start), int(end) + 1))
        else:
            ids.append(int(part))
    return sorted(set(ids))


def generate_markers(ids, size_cm, out_dir, dict_name="DICT_4X4_50"):
    dict_map = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    }
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_map[dict_name])
    os.makedirs(out_dir, exist_ok=True)

    # 마커 픽셀 크기 = 물리크기(inch) * DPI
    marker_px = int(size_cm / CM_PER_INCH * DPI)
    # quiet zone(흰 여백): 마커의 25% — 검출 안정성에 중요
    margin = marker_px // 4
    # 하단 라벨 영역
    label_h = marker_px // 5

    for marker_id in ids:
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_px)

        canvas_w = marker_px + margin * 2
        canvas_h = marker_px + margin * 2 + label_h
        canvas = np.full((canvas_h, canvas_w), 255, dtype=np.uint8)
        canvas[margin:margin + marker_px, margin:margin + marker_px] = marker_img

        # 사람 눈으로 식별할 수 있는 라벨 (ID + 사전 이름)
        label = f"ID {marker_id}  ({dict_name}, {size_cm}cm)"
        font_scale = marker_px / 600
        cv2.putText(
            canvas, label,
            (margin, canvas_h - label_h // 3),
            cv2.FONT_HERSHEY_SIMPLEX, font_scale, 0, 2, cv2.LINE_AA
        )

        path = os.path.join(out_dir, f"aruco_{dict_name}_id{marker_id:03d}.png")
        cv2.imwrite(path, canvas)
        print(f"생성: {path}")

    print(f"\n[안내] {DPI}DPI로 인쇄하면 마커 한 변이 정확히 {size_cm}cm가 됩니다.")
    print("       인쇄 시 '실제 크기(100%)' 옵션을 사용하세요. '페이지에 맞춤'은 크기가 틀어집니다.")
    print("       marker_length 파라미터에는 흰 여백 제외, 검은 마커 부분 길이만 넣으세요.")


def load_registry():
    if os.path.exists(REGISTRY_FILE):
        with open(REGISTRY_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    return {}


def save_registry(registry):
    with open(REGISTRY_FILE, "w", encoding="utf-8") as f:
        json.dump(registry, f, ensure_ascii=False, indent=2)


def register_marker(marker_id, name, equip_type, zone):
    registry = load_registry()
    key = str(marker_id)
    if key in registry:
        print(f"[경고] ID {marker_id}는 이미 등록됨: {registry[key]['name']} — 덮어씁니다.")
    registry[key] = {
        "name": name,
        "type": equip_type,
        "zone": zone,
    }
    save_registry(registry)
    print(f"등록 완료: ID {marker_id} -> {name} ({equip_type}, zone={zone})")


def list_markers():
    registry = load_registry()
    if not registry:
        print("등록된 마커가 없습니다.")
        return
    print(f"{'ID':>5} | {'이름':<24} | {'유형':<14} | zone")
    print("-" * 60)
    for key in sorted(registry, key=int):
        info = registry[key]
        print(f"{key:>5} | {info['name']:<24} | {info['type']:<14} | {info['zone']}")


def main():
    parser = argparse.ArgumentParser(description="ArUco 마커 생성/관리 툴")
    sub = parser.add_subparsers(dest="command", required=True)

    p_gen = sub.add_parser("generate", help="마커 PNG 생성")
    p_gen.add_argument("--ids", required=True, help="예: 0-19 또는 3,7,42")
    p_gen.add_argument("--size-cm", type=float, default=10.0, help="마커 한 변 물리 크기(cm)")
    p_gen.add_argument("--dict", default="DICT_4X4_50", help="ArUco 사전 이름")
    p_gen.add_argument("--out", default="./markers", help="출력 폴더")

    p_reg = sub.add_parser("register", help="점검 포인트 등록")
    p_reg.add_argument("--id", type=int, required=True)
    p_reg.add_argument("--name", required=True, help='예: "3층 동측 소화전"')
    p_reg.add_argument("--type", default="extinguisher",
                        help="extinguisher / hydrant / first_aid_kit / fire_door 등")
    p_reg.add_argument("--zone", default="", help="구역 ID (예: Z3)")

    sub.add_parser("list", help="등록된 마커 목록")

    args = parser.parse_args()

    if args.command == "generate":
        generate_markers(parse_ids(args.ids), args.size_cm, args.out, args.dict)
    elif args.command == "register":
        register_marker(args.id, args.name, args.type, args.zone)
    elif args.command == "list":
        list_markers()


if __name__ == "__main__":
    main()