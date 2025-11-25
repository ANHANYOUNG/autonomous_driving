#!/usr/bin/env python3

import os
import sys
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
import argparse
from pathlib import Path


def load_calibration_data(data_dir, timestamp=None):
    data_dir = Path(data_dir).expanduser()
    
    if timestamp is None:
        # 가장 최근 파일 찾기
        json_files = sorted(data_dir.glob('cal_*.json'))
        if not json_files:
            print(f"No calibration data found in {data_dir}")
            return None, None
        json_file = json_files[-1]
    else:
        json_file = data_dir / f'cal_{timestamp}.json'
        if not json_file.exists():
            print(f"File not found: {json_file}")
            return None, None
    
    print(f"Loading: {json_file}")
    
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    return data, json_file


def plot_calibration(data, save_path=None):
    # 데이터 구조: uwb_points와 imu_yaws 분리
    uwb_points = np.array(data.get('uwb_points', []))
    imu_yaws = np.array(data.get('imu_yaws', []))
    results = data.get('results', {})
    params = data.get('parameters', {})
    
    if len(uwb_points) == 0:
        print("No UWB data in calibration file")
        return
    
    # Forward/Backward 구간 분리 (최대 도달 거리 기준)
    start_pt = uwb_points[0]
    distances = np.linalg.norm(uwb_points - start_pt, axis=1)
    max_dist_idx = np.argmax(distances)  # 최대 거리 지점 (반환점)
    
    uwb_forward = uwb_points[:max_dist_idx+1]
    uwb_backward = uwb_points[max_dist_idx:]
    
    # IMU도 동일 비율로 분할
    n_total_imu = len(imu_yaws)
    n_forward_imu = int(n_total_imu * (max_dist_idx+1) / len(uwb_points))
    imu_forward = imu_yaws[:n_forward_imu]
    imu_backward = imu_yaws[n_forward_imu:]
    
    # 2x2 그리드 생성
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle(f"Calibration Analysis - {data['timestamp']}", fontsize=16, fontweight='bold')
    
    # ===== Plot 1: 전체 궤적 (Forward=파랑, Backward=빨강) =====
    ax1 = axes[0, 0]
    
    # Forward 경로 (파란색)
    ax1.plot(uwb_forward[:, 0], uwb_forward[:, 1], 'b-', linewidth=2, alpha=0.7, label='Forward', zorder=2)
    ax1.plot(uwb_forward[:, 0], uwb_forward[:, 1], 'b.', markersize=2, alpha=0.3)
    
    # Backward 경로 (빨간색)
    ax1.plot(uwb_backward[:, 0], uwb_backward[:, 1], 'r-', linewidth=2, alpha=0.7, label='Backward', zorder=2)
    ax1.plot(uwb_backward[:, 0], uwb_backward[:, 1], 'r.', markersize=2, alpha=0.3)
    
    # 시작/반환점/끝점
    ax1.plot(uwb_points[0, 0], uwb_points[0, 1], 'o', markersize=10, label='Start', zorder=5, 
             color='blue', markeredgewidth=2, markeredgecolor='blue')
    ax1.plot(uwb_points[max_dist_idx, 0], uwb_points[max_dist_idx, 1], 'o', markersize=10, label='Turn', zorder=5, 
             color='orange', markeredgewidth=2, markeredgecolor='orange')
    ax1.plot(uwb_points[-1, 0], uwb_points[-1, 1], 'o', markersize=10, label='End', zorder=5, 
             color='red', markeredgewidth=2, markeredgecolor='red')
    
    # PCA 방향 벡터 (전체 데이터 기준)
    if 'pca_direction_uwb' in results:
        center = uwb_points.mean(axis=0)
        pc1 = np.array(results['pca_direction_uwb'])
        
        forward_dist = results.get('forward_distance_m', np.linalg.norm(uwb_points.ptp(axis=0)) / 2.0)
        scale = forward_dist * 0.4
        
        uwb_angle_deg = results.get('uwb_angle_deg', 0)
        ax1.arrow(center[0], center[1], pc1[0]*scale, pc1[1]*scale, 
                 head_width=0.1, head_length=0.15, fc='purple', ec='indigo', 
                 linewidth=3, label=f"UWB PCA: {uwb_angle_deg:.1f}°", zorder=10)
    
    ax1.set_xlabel('X [m]', fontsize=11)
    ax1.set_ylabel('Y [m]', fontsize=11)
    ax1.set_title('Trajectory', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    ax1.legend(fontsize=9, loc='best')
    
    # ===== Plot 2: UWB 잔차 (Forward=파랑, Backward=빨강) =====
    ax2 = axes[0, 1]
    
    if 'pca_direction_uwb' in results:
        pc1 = np.array(results['pca_direction_uwb'])
        center = uwb_points.mean(axis=0)
        centered = uwb_points - center
        
        # 전체 데이터 잔차 (부호 유지 - 좌우 쏠림 확인용)
        residuals = np.cross(centered, pc1) * 1000  # mm 단위, 부호 유지
        
        # Forward 구간 (파란색)
        res_forward = residuals[:max_dist_idx+1]
        ax2.plot(range(len(res_forward)), res_forward, 'b-', linewidth=1.5, alpha=0.7, label='Forward')
        
        # Backward 구간 (빨간색)
        res_backward = residuals[max_dist_idx:]
        ax2.plot(range(max_dist_idx, len(residuals)), res_backward, 'r-', linewidth=1.5, alpha=0.7, label='Backward')
        
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        # 통계 정보 (절댓값 기준)
        mean_abs_res = np.mean(np.abs(residuals))
        std_res = np.std(residuals)
        
        # 2σ 범위 내 데이터 비율 계산
        within_2sigma = np.sum(np.abs(residuals) <= 2*std_res)
        total_points = len(residuals)
        coverage_2sigma = (within_2sigma / total_points) * 100
        
        ax2.axhline(y=mean_abs_res, color='g', linestyle='--', linewidth=2, label=f'Avg Error: {mean_abs_res:.1f} mm')
        ax2.axhline(y=-mean_abs_res, color='g', linestyle='--', linewidth=2)
        ax2.axhline(y=2*std_res, color='orange', linestyle=':', linewidth=1.5, label=f'±2σ: {2*std_res:.1f} mm ({coverage_2sigma:.1f}%)')
        ax2.axhline(y=-2*std_res, color='orange', linestyle=':', linewidth=1.5)
        
        # 반환점 표시
        ax2.axvline(x=max_dist_idx, color='y', linestyle='--', alpha=0.5, linewidth=2, label='Turn Point')
        
        # 2σ 커버리지를 results에 저장 (Summary에서 사용)
        results['uwb_2sigma_coverage_pct'] = float(coverage_2sigma)
        
        ax2.set_xlabel('Sample Index', fontsize=11)
        ax2.set_ylabel('Position Error [mm] (+ Right / - Left)', fontsize=11)
        ax2.set_title('UWB Position Error', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=9, loc='best')
    
    # ===== Plot 3: IMU 편차 (Forward=파랑, Backward=빨강) =====
    ax3 = axes[1, 0]
    
    imu_angle_deg = results.get('imu_angle_deg', 0)
    if imu_angle_deg != 0:
        imu_angle_rad = np.radians(imu_angle_deg)
        
        # 각도 편차 계산
        angle_diffs = np.array([((yaw - imu_angle_rad + np.pi) % (2*np.pi)) - np.pi for yaw in imu_yaws])
        angle_diffs_deg = np.degrees(angle_diffs)
        
        # 시간 축 (100Hz 가정)
        time_imu = np.arange(len(imu_yaws)) * 0.01
        time_forward_end = n_forward_imu * 0.01
        
        # Forward 구간 (파란색)
        ax3.plot(time_imu[:n_forward_imu], angle_diffs_deg[:n_forward_imu], 'b-', linewidth=1, alpha=0.7, label='Forward')
        
        # Backward 구간 (빨간색)
        ax3.plot(time_imu[n_forward_imu:], angle_diffs_deg[n_forward_imu:], 'r-', linewidth=1, alpha=0.7, label='Backward')
        
        ax3.axhline(y=0, color='k', linestyle='--', linewidth=2, label='Mean Direction')
        
        # 통계 정보
        std_imu = np.std(angle_diffs_deg)
        
        # 2σ 범위 내 데이터 비율 계산
        within_2sigma_imu = np.sum(np.abs(angle_diffs_deg) <= 2*std_imu)
        total_imu = len(angle_diffs_deg)
        coverage_2sigma_imu = (within_2sigma_imu / total_imu) * 100
        
        ax3.axhline(y=2*std_imu, color='orange', linestyle=':', linewidth=1.5, label=f'±2σ: {2*std_imu:.2f}° ({coverage_2sigma_imu:.1f}%)')
        ax3.axhline(y=-2*std_imu, color='orange', linestyle=':', linewidth=1.5)
        
        # 반환점 표시
        ax3.axvline(x=time_forward_end, color='y', linestyle='--', alpha=0.5, linewidth=2, label='Turn Point')
        
        # 2σ 커버리지를 results에 저장 (Summary에서 사용)
        results['imu_2sigma_coverage_pct'] = float(coverage_2sigma_imu)
        
        ax3.set_xlabel('Time [s]', fontsize=11)
        ax3.set_ylabel('Yaw Error [deg]', fontsize=11)
        ax3.set_title('IMU Yaw Error', fontsize=12, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        ax3.legend(fontsize=9, loc='best')
    
    # ===== Plot 4: 캘리브레이션 요약 =====
    ax4 = axes[1, 1]
    ax4.axis('off')
    
    # 결과 값 포맷팅
    yaw_offset_deg = results.get('yaw_offset_angle_deg', 'N/A')
    yaw_offset_str = f"{yaw_offset_deg:.2f}°" if isinstance(yaw_offset_deg, (int, float)) else str(yaw_offset_deg)
    
    forward_dist = results.get('forward_distance_m', 'N/A')
    forward_dist_str = f"{forward_dist:.2f}" if isinstance(forward_dist, (int, float)) else str(forward_dist)
    
    backward_dist = results.get('backward_distance_m', 'N/A')
    backward_dist_str = f"{backward_dist:.2f}" if isinstance(backward_dist, (int, float)) else str(backward_dist)
    
    # UWB Precision: 절댓값 평균으로 정밀도 표시
    uwb_precision = results.get('uwb_precision_mm', 'N/A')
    uwb_precision_str = f"{uwb_precision:.1f}" if isinstance(uwb_precision, (int, float)) else str(uwb_precision)
    
    imu_std = results.get('imu_std_dev_deg', 'N/A')
    imu_std_str = f"{imu_std:.2f}" if isinstance(imu_std, (int, float)) else str(imu_std)
    
    # 2σ 커버리지 비율
    uwb_2sigma_cov = results.get('uwb_2sigma_coverage_pct', 'N/A')
    uwb_2sigma_str = f"{uwb_2sigma_cov:.1f}%" if isinstance(uwb_2sigma_cov, (int, float)) else str(uwb_2sigma_cov)
    
    imu_2sigma_cov = results.get('imu_2sigma_coverage_pct', 'N/A')
    imu_2sigma_str = f"{imu_2sigma_cov:.1f}%" if isinstance(imu_2sigma_cov, (int, float)) else str(imu_2sigma_cov)
    
    uwb_angle_deg = results.get('uwb_angle_deg', 'N/A')
    uwb_angle_str = f"{uwb_angle_deg:.2f}" if isinstance(uwb_angle_deg, (int, float)) else str(uwb_angle_deg)
    
    imu_angle_deg = results.get('imu_angle_deg', 'N/A')
    imu_angle_str = f"{imu_angle_deg:.2f}" if isinstance(imu_angle_deg, (int, float)) else str(imu_angle_deg)
    
    success_icon = "✓" if data.get('success', False) else "✗"
    
    info_text = f"""
CALIBRATION SUMMARY
{'='*50}

Status: {success_icon} {'SUCCESS' if data.get('success', False) else 'FAILED'}
Timestamp: {data['timestamp']}

DATA COLLECTION
  • Forward Distance:   {forward_dist_str} m
  • Backward Distance:  {backward_dist_str} m
  • UWB Points:         {len(uwb_points)}
  • IMU Samples:        {len(imu_yaws)}
  • Turn Point:         Index {max_dist_idx}

ANGLE ANALYSIS
  • UWB Angle:          {uwb_angle_str}°
  • IMU Angle:          {imu_angle_str}°
  • Yaw Offset Angle:   {yaw_offset_str}
  
QUALITY METRICS
  • UWB 2σ Coverage:    {uwb_2sigma_str}
  • IMU 2σ Coverage:    {imu_2sigma_str}

CORRECTION
  • Offset Applied:     {yaw_offset_str}
  • Target Direction:   {uwb_angle_str}°

PARAMETERS
  • Forward:  {params.get('forward_time', 'N/A')} s @ {params.get('forward_speed', 'N/A')} m/s
  • Backward: {params.get('backward_time', 'N/A')} s @ {params.get('backward_speed', 'N/A')} m/s
    """
    
    ax4.text(0.05, 0.95, info_text, transform=ax4.transAxes,
            fontsize=10, verticalalignment='top', family='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat' if data.get('success', False) else 'lightcoral', alpha=0.3))
    
    plt.tight_layout()
    
    # PNG 저장 (항상 저장)
    if save_path is None:
        timestamp = data.get('timestamp', 'unknown')
        save_path = Path.home() / 'calibration_data' / f'cal_{timestamp}.png'
    
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"Plot saved: {save_path}")
    
    plt.show()


def list_calibration_data(data_dir):
    data_dir = Path(data_dir).expanduser()
    json_files = sorted(data_dir.glob('cal_*.json'))
    
    if not json_files:
        print(f"No calibration data found in {data_dir}")
        return
    
    print(f"\nCalibration Data in {data_dir}")
    print("="*100)
    print(f"{'Timestamp':<20} {'UWB Pts':<10} {'IMU Yaws':<10} {'Success':<10} {'Error (deg)':<15} {'Distance (m)':<15}")
    print("-"*100)
    
    for json_file in json_files:
        with open(json_file, 'r') as f:
            data = json.load(f)
        
        timestamp = data.get('timestamp', 'Unknown')
        num_uwb = len(data.get('uwb_points', []))
        num_imu = len(data.get('imu_yaws', []))
        success = 'Yes' if data.get('success', False) else 'No'
        
        results = data.get('results', {})
        error = results.get('incremental_error_deg', 'N/A')
        distance = results.get('forward_distance_m', results.get('travel_distance_m', 'N/A'))
        
        if isinstance(error, (int, float)):
            error_str = f"{error:.2f}°"
        else:
            error_str = str(error)
                
        if isinstance(distance, (int, float)):
            distance_str = f"{distance:.2f}"
        else:
            distance_str = str(distance)
        
        print(f"{timestamp:<20} {num_uwb:<10} {num_imu:<10} {success:<10} {error_str:<15} {distance_str:<15}")
    
    print("="*100)
    print(f"Total: {len(json_files)} calibration session(s)\n")


def main():
    parser = argparse.ArgumentParser(description='Calibration Data Visualization')
    parser.add_argument('timestamp', nargs='?', default=None,
                       help='Timestamp of calibration session (YYYYMMDD_HHMMSS)')
    parser.add_argument('--list', '-l', action='store_true',
                       help='List all available calibration data')
    parser.add_argument('--dir', '-d', default='~/calibration_data',
                       help='Data directory (default: ~/calibration_data)')
    parser.add_argument('--save', '-s', default=None,
                       help='Custom save path for PNG (default: auto-generated)')
    
    args = parser.parse_args()
    
    if args.list:
        list_calibration_data(args.dir)
        return
    
    data, json_file = load_calibration_data(args.dir, args.timestamp)
    
    if data is None:
        print("\nUse --list to see available calibration data")
        return
    
    save_path = args.save if args.save else None
    
    plot_calibration(data, save_path)


if __name__ == '__main__':
    main()