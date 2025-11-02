#!/usr/bin/env python3
"""
캘리브레이션 데이터 시각화 스크립트

사용법:
  python3 plot_calibration.py                    # 가장 최근 데이터 plot
  python3 plot_calibration.py 20251101_143025    # 특정 타임스탬프 plot
  python3 plot_calibration.py --list             # 저장된 데이터 목록
"""

import os
import sys
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
import argparse
from pathlib import Path


def load_calibration_data(data_dir, timestamp=None):
    """캘리브레이션 데이터 로드"""
    data_dir = Path(data_dir).expanduser()
    
    if timestamp is None:
        # 가장 최근 파일 찾기
        json_files = sorted(data_dir.glob('cal_*.json'))
        if not json_files:
            print(f"No calibration data found in {data_dir}")
            return None
        json_file = json_files[-1]
    else:
        json_file = data_dir / f'cal_{timestamp}.json'
        if not json_file.exists():
            print(f"File not found: {json_file}")
            return None
    
    print(f"Loading: {json_file}")
    
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    return data, json_file


def plot_calibration(data, save_path=None):
    """캘리브레이션 데이터 시각화"""
    points = np.array(data['points'])
    results = data.get('results', {})
    params = data.get('parameters', {})
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle(f"Calibration Analysis - {data['timestamp']}", fontsize=16, fontweight='bold')
    
    # ===== Plot 1: 전체 궤적 =====
    ax1 = axes[0, 0]
    ax1.plot(points[:, 0], points[:, 1], 'b-', linewidth=1, alpha=0.6, label='Trajectory')
    ax1.plot(points[:, 0], points[:, 1], 'b.', markersize=3, alpha=0.4)
    
    # 시작/끝점 표시
    ax1.plot(points[0, 0], points[0, 1], 'go', markersize=12, label='Start', zorder=5)
    ax1.plot(points[-1, 0], points[-1, 1], 'ro', markersize=12, label='End', zorder=5)
    
    # PCA 주축 표시
    if 'eigenvector' in results:
        center = points.mean(axis=0)
        ev = np.array(results['eigenvector'])
        eigenvalues = np.array(results['eigenvalues'])
        scale = np.sqrt(eigenvalues[0]) * 2
        
        ax1.arrow(center[0], center[1], ev[0]*scale, ev[1]*scale, 
                 head_width=0.05, head_length=0.08, fc='red', ec='red', 
                 linewidth=2, label='PCA Main Axis', zorder=10)
    
    # 방향 벡터 표시
    if results.get('direction_vector'):
        dv = np.array(results['direction_vector'])
        ax1.arrow(center[0], center[1], dv[0]*scale*0.8, dv[1]*scale*0.8,
                 head_width=0.05, head_length=0.08, fc='green', ec='green',
                 linewidth=2, label='Direction Vector', zorder=10, alpha=0.7)
    
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title('Robot Trajectory')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    ax1.legend()
    
    # ===== Plot 2: X/Y 좌표 vs 시간 =====
    ax2 = axes[0, 1]
    time_steps = np.arange(len(points)) * 0.1  # 0.1초 간격
    ax2.plot(time_steps, points[:, 0], 'b-', label='X position', linewidth=2)
    ax2.plot(time_steps, points[:, 1], 'r-', label='Y position', linewidth=2)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Position [m]')
    ax2.set_title('Position vs Time')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Forward/Backward 구간 표시
    forward_time = params.get('forward_time', 2.0)
    backward_time = params.get('backward_time', 2.0)
    ax2.axvline(x=forward_time, color='g', linestyle='--', alpha=0.5, label='Forward End')
    ax2.axvline(x=forward_time+backward_time, color='r', linestyle='--', alpha=0.5, label='Backward End')
    
    # ===== Plot 3: 궤적 분포 (히트맵) =====
    ax3 = axes[1, 0]
    
    # 2D 히스토그램
    hist, xedges, yedges = np.histogram2d(points[:, 0], points[:, 1], bins=30)
    extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]
    
    im = ax3.imshow(hist.T, extent=extent, origin='lower', cmap='hot', aspect='auto', interpolation='bilinear')
    ax3.plot(points[:, 0], points[:, 1], 'c-', linewidth=0.5, alpha=0.5)
    ax3.plot(points[0, 0], points[0, 1], 'go', markersize=10, label='Start')
    ax3.plot(points[-1, 0], points[-1, 1], 'ro', markersize=10, label='End')
    
    plt.colorbar(im, ax=ax3, label='Point Density')
    ax3.set_xlabel('X [m]')
    ax3.set_ylabel('Y [m]')
    ax3.set_title('Trajectory Density Map')
    ax3.legend()
    
    # ===== Plot 4: 결과 요약 =====
    ax4 = axes[1, 1]
    ax4.axis('off')
    
    # 텍스트 정보
    info_text = f"""
CALIBRATION RESULTS
{'='*40}

Parameters:
  • Forward Time: {params.get('forward_time', 'N/A')} s
  • Backward Time: {params.get('backward_time', 'N/A')} s
  • Forward Speed: {params.get('forward_speed', 'N/A')} m/s
  • Backward Speed: {params.get('backward_speed', 'N/A')} m/s

Data Collection:
  • Total Points: {results.get('num_points', len(points))}
  • Duration: {len(points)*0.1:.1f} s
  • Sampling Rate: ~10 Hz

PCA Analysis:
  • Detected Angle: {results.get('detected_angle_deg', 'N/A'):.2f}°
  • Yaw Error: {results.get('yaw_error_deg', 'N/A'):.2f}°
  • Eigenvalues: {results.get('eigenvalues', ['N/A'])}

Status:
  • Success: {data.get('success', 'Unknown')}
  • Reason: {data.get('failure_reason', 'N/A') if not data.get('success', True) else 'OK'}

Trajectory Stats:
  • X Range: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}] m
  • Y Range: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}] m
  • Total Distance: {np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)):.3f} m
    """
    
    ax4.text(0.05, 0.95, info_text, transform=ax4.transAxes,
            fontsize=10, verticalalignment='top', family='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Plot saved: {save_path}")
    
    plt.show()


def list_calibration_data(data_dir):
    """저장된 캘리브레이션 데이터 목록 출력"""
    data_dir = Path(data_dir).expanduser()
    json_files = sorted(data_dir.glob('cal_*.json'))
    
    if not json_files:
        print(f"No calibration data found in {data_dir}")
        return
    
    print(f"\nCalibration Data in {data_dir}")
    print("="*80)
    print(f"{'Timestamp':<20} {'Points':<10} {'Success':<10} {'Yaw Error':<15}")
    print("-"*80)
    
    for json_file in json_files:
        with open(json_file, 'r') as f:
            data = json.load(f)
        
        timestamp = data.get('timestamp', 'Unknown')
        num_points = len(data.get('points', []))
        success = 'Yes' if data.get('success', False) else 'No'
        yaw_error = data.get('results', {}).get('yaw_error_deg', 'N/A')
        
        if isinstance(yaw_error, (int, float)):
            yaw_error_str = f"{yaw_error:.2f}°"
        else:
            yaw_error_str = str(yaw_error)
        
        print(f"{timestamp:<20} {num_points:<10} {success:<10} {yaw_error_str:<15}")
    
    print("="*80)
    print(f"Total: {len(json_files)} calibration session(s)\n")


def main():
    parser = argparse.ArgumentParser(description='Calibration Data Visualization')
    parser.add_argument('timestamp', nargs='?', default=None,
                       help='Timestamp of calibration session (YYYYMMDD_HHMMSS)')
    parser.add_argument('--list', '-l', action='store_true',
                       help='List all available calibration data')
    parser.add_argument('--dir', '-d', default='~/calibration_data',
                       help='Data directory (default: ~/calibration_data)')
    parser.add_argument('--save', '-s', action='store_true',
                       help='Save plot as PNG')
    
    args = parser.parse_args()
    
    if args.list:
        list_calibration_data(args.dir)
        return
    
    data, json_file = load_calibration_data(args.dir, args.timestamp)
    
    if data is None:
        print("\nUse --list to see available calibration data")
        return
    
    save_path = None
    if args.save:
        save_path = json_file.with_suffix('.png')
    
    plot_calibration(data, save_path)


if __name__ == '__main__':
    main()
