"""tripinfo分析工具：解析SUMO tripinfo输出并统计等待时间指标"""

import os
import xml.etree.ElementTree as ET
from typing import Dict, List

from src.config import TRIPINFO_OUTPUT_FILE


class TripInfoAnalyzer:
    """处理SUMO tripinfo输出的辅助类"""

    def __init__(self, tripinfo_file: str = TRIPINFO_OUTPUT_FILE):
        self.tripinfo_file = tripinfo_file

    def _load_tripinfos(self) -> List[Dict]:
        """读取并解析tripinfo文件中的车辆行程信息"""
        if not os.path.exists(self.tripinfo_file):
            return []

        tree = ET.parse(self.tripinfo_file)
        root = tree.getroot()
        tripinfos = []

        for tripinfo in root.findall("tripinfo"):
            attrs = tripinfo.attrib
            tripinfos.append(
                {
                    "id": attrs.get("id", ""),
                    "depart": float(attrs.get("depart", 0.0)),
                    "arrival": float(attrs.get("arrival", 0.0)),
                    "duration": float(attrs.get("duration", 0.0)),
                    "waiting_time": float(attrs.get("waitingTime", 0.0)),
                }
            )

        return tripinfos

    def calculate_waiting_statistics(self) -> Dict:
        """根据tripinfo计算等待时间相关指标"""
        tripinfos = self._load_tripinfos()
        if not tripinfos:
            return {
                "total_vehicles": 0,
                "vehicles_with_wait": 0,
                "total_waiting_time": 0.0,
                "avg_waiting_time": 0.0,
                "ramp_wait_ratio": 0.0,
            }

        waited = [t for t in tripinfos if t["waiting_time"] > 0]
        ramp_total = [t for t in tripinfos if t["id"].startswith("r")]
        ramp_waited = [t for t in waited if t["id"].startswith("r")]

        total_wait = sum(t["waiting_time"] for t in waited)
        avg_wait = total_wait / len(waited) if waited else 0.0
        ramp_wait_ratio = (len(ramp_waited) / len(ramp_total) * 100) if ramp_total else 0.0

        return {
            "total_vehicles": len(tripinfos),
            "vehicles_with_wait": len(waited),
            "total_waiting_time": total_wait,
            "avg_waiting_time": avg_wait,
            "ramp_wait_ratio": ramp_wait_ratio,
            "ramp_total": len(ramp_total),
            "ramp_waited": len(ramp_waited),
        }

    def format_report(self) -> str:
        """生成等待时间统计的可打印文本"""
        stats = self.calculate_waiting_statistics()
        if stats["total_vehicles"] == 0:
            return (
                "未找到tripinfo输出，无法生成等待时间统计。"  # pragma: no cover - 运行期提示
            )

        report_lines = [
            "=== TripInfo等待时间统计 ===",
            f"总车辆数: {stats['total_vehicles']}",
            f"有等待的车辆数: {stats['vehicles_with_wait']}",
            f"总等待时间: {stats['total_waiting_time']:.2f} 秒",
            f"平均等待时间: {stats['avg_waiting_time']:.2f} 秒",
            f"匝道车总数: {stats['ramp_total']}",
            f"有等待的匝道车数: {stats['ramp_waited']}",
            f"等待过的匝道车占比: {stats['ramp_wait_ratio']:.2f}%",
        ]
        return "\n".join(report_lines)
