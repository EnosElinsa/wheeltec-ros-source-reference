#!/usr/bin/env python3
"""
map_utils.py — 地图位置管理工具
解析 map_mapping.yaml（name + position + orientation 结构），
支持按符号(A/B/C...)或中文名称查询。
"""

import os
import yaml
from typing import Optional, Dict, List


def load_map_mapping(pkg_path: str) -> dict:
    """
    加载地图映射文件。
    返回原始 dict: {符号: {name, position: {x, y, z}, orientation: {x, y, z, w}}}
    """
    map_file = os.path.join(pkg_path, 'config', 'map_mapping.yaml')
    if not os.path.exists(map_file):
        return {}
    with open(map_file, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return data


def list_locations(mapping: dict) -> List[str]:
    """
    返回所有已知位置的人类可读列表。
    格式: "A: 原点起点", "B: 水果店", ...

    Args:
        mapping: load_map_mapping() 返回的 dict
    Returns:
        ["A: 原点起点", "B: 水果店", ...]
    """
    lines = []
    for symbol, info in mapping.items():
        if isinstance(info, dict):
            name = info.get('name', '')
            lines.append(f'{symbol}: {name}')
    return lines


def get_location(mapping: dict, key: str) -> Optional[Dict]:
    """
    查找目标位置。
    支持按符号(A/B/C)或按中文名称(如"水果店")查找。

    Args:
        mapping: 地图映射字典
        key: 符号或中文名称

    Returns:
        找到则返回 {symbol, name, description, position: {x,y,z}, orientation: {x,y,z,w}}
        未找到返回 None
    """
    if not mapping:
        return None

    # 先按符号精确匹配
    if key in mapping:
        info = mapping[key]
        if isinstance(info, dict):
            return {'symbol': key, **info}

    # 再按名称匹配（去引号后精确匹配）
    key_clean = key.strip().strip('"\'')
    for symbol, info in mapping.items():
        if not isinstance(info, dict):
            continue
        name = info.get('name', '')
        # 去除引号后比较
        name_clean = name.strip('"\'')
        if name_clean == key_clean:
            return {'symbol': symbol, **info}

    return None


def get_description(mapping: dict, key: str) -> str:
    """获取指定位置的描述信息，无描述返回空字符串"""
    loc = get_location(mapping, key)
    if not loc:
        return ''
    return loc.get('description', '').strip('"\'')


def format_for_prompt(mapping: dict) -> str:
    """
    将地图映射格式化为可插入 Prompt 的文本。
    示例输出:
        'A': '原点起点'  # 描述: xxx
        'B': '水果店'
    """
    if not mapping:
        return '# 地图映射\n(暂无已配置的地图位置)\n'
    lines = ['# 地图映射\n']
    for symbol, info in mapping.items():
        if isinstance(info, dict):
            name = info.get('name', '')
            desc = info.get('description', '')
            if desc:
                lines.append(f"'{symbol}': '{name}'  # {desc}")
            else:
                lines.append(f"'{symbol}': '{name}',")
    return '\n'.join(lines) + '\n'