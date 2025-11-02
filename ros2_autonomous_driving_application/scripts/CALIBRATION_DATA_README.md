# Calibration Data Logging & Visualization

## 개요
캘리브레이션 실행 시 수집된 데이터를 자동으로 저장하고 분석할 수 있습니다.

## 데이터 저장 위치
```
~/calibration_data/
├── cal_20251101_143025.json  # 메타데이터 + 결과
├── cal_20251101_143025.npy   # 포인트 데이터 (NumPy 배열)
├── cal_20251101_150312.json
├── cal_20251101_150312.npy
└── ...
```

## 저장되는 데이터

### JSON 파일 내용:
- **timestamp**: 캘리브레이션 시작 시간
- **points**: 수집된 (x, y) 좌표 리스트
- **parameters**: 캘리브레이션 파라미터 (속도, 시간 등)
- **results**: PCA 분석 결과
  - detected_angle_deg: 검출된 각도
  - yaw_error_deg: Yaw 오차
  - eigenvalues: 고유값
  - eigenvector: 주 고유벡터
  - direction_vector: 이동 방향 벡터
  - covariance_matrix: 공분산 행렬
- **success**: 성공 여부
- **failure_reason**: 실패 원인 (있는 경우)

## 사용 방법

### 1. 캘리브레이션 실행
```bash
# state_machine.py를 실행하고 CALIBRATION 상태로 전환
# 데이터가 자동으로 ~/calibration_data/에 저장됩니다
```

로그 예시:
```
[CAL] Data saved: /home/dnc/calibration_data/cal_20251101_143025.json
[CAL] Points saved: /home/dnc/calibration_data/cal_20251101_143025.npy
```

### 2. 저장된 데이터 목록 확인
```bash
cd ~/autonomous_driving/src/ros2_autonomous_driving_application/scripts
python3 plot_calibration.py --list
```

출력 예시:
```
Calibration Data in ~/calibration_data
================================================================================
Timestamp            Points     Success    Yaw Error      
--------------------------------------------------------------------------------
20251101_143025      247        Yes        -2.35°         
20251101_150312      198        No         N/A            
20251101_152140      265        Yes        1.18°          
================================================================================
Total: 3 calibration session(s)
```

### 3. 데이터 시각화

#### 가장 최근 데이터 보기
```bash
python3 plot_calibration.py
```

#### 특정 타임스탬프 데이터 보기
```bash
python3 plot_calibration.py 20251101_143025
```

#### PNG 파일로 저장
```bash
python3 plot_calibration.py --save
# 또는
python3 plot_calibration.py 20251101_143025 --save
```

### 4. 생성되는 Plot 설명

**4개의 서브플롯:**

1. **Robot Trajectory (좌상단)**
   - 로봇의 실제 이동 궤적
   - 시작점(녹색), 끝점(빨간색)
   - PCA 주축(빨간색 화살표)
   - 방향 벡터(녹색 화살표)

2. **Position vs Time (우상단)**
   - X, Y 좌표의 시간에 따른 변화
   - Forward/Backward 구간 구분
   - 직선 주행 확인

3. **Trajectory Density Map (좌하단)**
   - 궤적 히트맵
   - 로봇이 많이 머문 위치 확인
   - 노이즈 분석

4. **Results Summary (우하단)**
   - 파라미터 요약
   - PCA 분석 결과
   - 통계 정보

## Python 패키지 요구사항
```bash
pip3 install matplotlib numpy
```

## 고급 사용

### Python 스크립트에서 직접 로드
```python
import json
import numpy as np

# JSON 로드
with open('~/calibration_data/cal_20251101_143025.json', 'r') as f:
    data = json.load(f)

# NumPy 배열 로드 (더 빠름)
points = np.load('~/calibration_data/cal_20251101_143025.npy')

# 분석
print(f"Points: {len(points)}")
print(f"Yaw error: {data['results']['yaw_error_deg']:.2f}°")
```

### 여러 세션 비교
```python
import glob
import json

files = sorted(glob.glob('~/calibration_data/cal_*.json'))
for f in files:
    with open(f, 'r') as fp:
        d = json.load(fp)
        print(f"{d['timestamp']}: {d['results'].get('yaw_error_deg', 'N/A')}°")
```

## 문제 해결

### "No calibration data found"
- 캘리브레이션을 최소 1회 실행해야 합니다
- 기본 경로: `~/calibration_data/`
- `--dir` 옵션으로 다른 경로 지정 가능

### "matplotlib not found"
```bash
pip3 install matplotlib
```

### 데이터 삭제
```bash
rm -rf ~/calibration_data/
```

## 팁

1. **정기적 백업**: 중요한 데이터는 백업하세요
```bash
cp -r ~/calibration_data/ ~/calibration_data_backup/
```

2. **비교 분석**: 여러 번 실행해서 재현성 확인
```bash
python3 plot_calibration.py --list  # 모든 세션 비교
```

3. **품질 확인 기준**:
   - Points > 200: 충분한 데이터
   - Yaw Error < 5°: 양호
   - Eigenvalue 비율 > 10:1: 직선 주행 확인
